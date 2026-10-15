/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file train_route_plan.cpp Route planning for trains over their whole order list. */

#include "stdafx.h"
#include "train_route_plan.h"
#include "train.h"
#include "order_base.h"
#include "station_base.h"
#include "waypoint_base.h"
#include "station_map.h"
#include "pbs.h"
#include "debug.h"
#include "settings_type.h"
#include "core/math_func.hpp"
#include "pathfinder/pathfinder_type.h"
#include "pathfinder/follow_track.hpp"
#include "pathfinder/yapf/yapf.h"
#include "map_func.h"
#include "tilearea_type.h"

#include <algorithm>
#include <chrono>
#include <map>
#include <optional>
#include <memory>
#include <tuple>

#include "safeguards.h"

namespace {

/** Upper limit of the extra costs of the targets, to avoid overflows in the pathfinder. */
static constexpr int ROUTE_PLAN_MAX_COST = 10000 * YAPF_TILE_LENGTH;

/** Remaining cost of a target from which the order list can't be completed. */
static constexpr int64_t ROUTE_PLAN_UNREACHABLE = INT64_MAX;

/** Properties of a train which affect its route plan. */
struct RoutePlanTrainParams {
	uint64_t railtypes = 0;         ///< Compatible railtypes.
	uint8_t owner = 0;              ///< Owner, for infrastructure sharing.
	uint16_t length_tiles = 0;      ///< Length in tiles, rounded up, for the platform length penalties.
	bool reverse_penalised = false; ///< Whether the train prefers not to reverse.

	auto Tie() const { return std::tie(this->railtypes, this->owner, this->length_tiles, this->reverse_penalised); }
	bool operator==(const RoutePlanTrainParams &other) const { return this->Tie() == other.Tie(); }
	bool operator<(const RoutePlanTrainParams &other) const { return this->Tie() < other.Tie(); }
};

/** How a train may depart from a stop. */
enum class RoutePlanDeparture : uint8_t {
	Forward,          ///< Only continue in the direction of arrival.
	ForwardOrReverse, ///< Continue or reverse.
	Reverse,          ///< Always reverse.
};

/** A stop of a route plan: a station or waypoint order. */
struct RoutePlanStop {
	StationID station;            ///< Station or waypoint.
	RoutePlanDeparture departure; ///< How the train may depart from this stop.

	auto Tie() const { return std::make_tuple(this->station.base(), this->departure); }
	bool operator==(const RoutePlanStop &other) const { return this->Tie() == other.Tie(); }
	bool operator<(const RoutePlanStop &other) const { return this->Tie() < other.Tie(); }
};

/** Key of a leg in the leg cache. */
struct RoutePlanLegKey {
	RoutePlanTrainParams params; ///< Properties of the train.
	RoutePlanStop from;          ///< The stop the leg starts at.
	StationID to;                ///< The station or waypoint the leg ends at.

	auto Tie() const { return std::make_tuple(this->params.Tie(), this->from.Tie(), this->to.base()); }
	bool operator<(const RoutePlanLegKey &other) const { return this->Tie() < other.Tie(); }
};

/** Area of the tiles a leg depends on. */
struct RoutePlanArea {
	uint x0 = UINT_MAX; ///< Lowest x coordinate.
	uint y0 = UINT_MAX; ///< Lowest y coordinate.
	uint x1 = 0;        ///< Highest x coordinate.
	uint y1 = 0;        ///< Highest y coordinate.

	void Add(TileIndex tile)
	{
		if (tile == INVALID_TILE) return;
		this->x0 = std::min(this->x0, TileX(tile));
		this->y0 = std::min(this->y0, TileY(tile));
		this->x1 = std::max(this->x1, TileX(tile));
		this->y1 = std::max(this->y1, TileY(tile));
	}

	void Add(const TileArea &area)
	{
		if (area.tile == INVALID_TILE) return;
		this->Add(area.tile);
		this->Add(TileXY(TileX(area.tile) + area.w - 1, TileY(area.tile) + area.h - 1));
	}

	/**
	 * Check whether an area intersects this area, extended by one tile:
	 * the track follower also examines the tiles next to the ones it records.
	 */
	bool Intersects(uint x0, uint y0, uint x1, uint y1) const
	{
		return x1 + 1 >= this->x0 && x0 <= this->x1 + 1 && y1 + 1 >= this->y0 && y0 <= this->y1 + 1;
	}

	bool Intersects(TileIndex tile) const
	{
		return this->Intersects(TileX(tile), TileY(tile), TileX(tile), TileY(tile));
	}

	bool Intersects(const TileArea &area) const
	{
		if (area.tile == INVALID_TILE) return false;
		return this->Intersects(TileX(area.tile), TileY(area.tile), TileX(area.tile) + area.w - 1, TileY(area.tile) + area.h - 1);
	}
};

/** Best path from a departure state to an arrival state of a leg. */
struct RoutePlanPath {
	uint16_t from; ///< Index of the departure state.
	uint16_t to;   ///< Index of the arrival state.
	int cost;      ///< Cost of the path.
};

/** A leg of a route plan: the paths from all platforms of a stop to all reachable platforms of the next stop. */
struct RoutePlanLeg {
	std::vector<TrainRoutePlanTarget> departures; ///< Arrival states at the previous stop, which the paths depart from.
	std::vector<TrainRoutePlanTarget> targets;    ///< Arrival states reached at the destination.
	std::vector<RoutePlanPath> paths;             ///< Best path from each departure state to each arrival state reached from it.
	std::vector<uint32_t> keys;                   ///< Sorted node keys of all paths.
	int margin = 0;                               ///< Largest cost margin used when searching the targets.
	RoutePlanArea area;                           ///< Area of all tiles the leg depends on.
};

/** Route plan of an order list. */
struct RoutePlan {
	std::vector<RoutePlanStop> stops;              ///< Stops of the order list.
	std::vector<VehicleOrderID> stop_orders;       ///< Order index of each stop, ascending.
	std::vector<const RoutePlanLeg *> legs;        ///< Leg towards each stop, from the previous stop.
	std::vector<std::vector<int>> target_costs;    ///< Extra cost of each target of each leg.
	std::vector<TrainRoutePlanGuide> guides;       ///< Pathfinder guidance towards each stop.
};

/** Key of a route plan in the route plan cache. */
struct RoutePlanKey {
	uint32_t order_list;         ///< Order list.
	RoutePlanTrainParams params; ///< Properties of the train.

	bool operator<(const RoutePlanKey &other) const { return std::tie(this->order_list, this->params) < std::tie(other.order_list, other.params); }
};

RoutePlanArea *_route_plan_recording = nullptr; ///< Area to record the tiles examined by the route plan searches in, if any.
uint64_t _route_plan_version = 1;       ///< Incremented whenever all cached route plans may have become invalid.
uint64_t _route_plan_cache_version = 0; ///< The value of #_route_plan_version when the caches were last validated.
std::map<RoutePlanLegKey, std::unique_ptr<RoutePlanLeg>> _route_plan_legs; ///< Cache of legs, shared between route plans.
std::map<RoutePlanKey, std::unique_ptr<RoutePlan>> _route_plans;           ///< Cache of route plans.

RoutePlanTrainParams GetRoutePlanTrainParams(const Train *v)
{
	RoutePlanTrainParams params;
	params.railtypes = v->compatible_railtypes.base();
	params.owner = v->owner.base();
	params.length_tiles = static_cast<uint16_t>(CeilDiv(v->gcache.cached_total_length, TILE_SIZE));
	params.reverse_penalised = _settings_game.difficulty.train_flip_reverse_allowed == TrainFlipReversingAllowed::None && !v->Last()->CanLeadTrain();
	return params;
}

/**
 * Get the route plan stop of an order.
 * @param order The order.
 * @return The stop, or std::nullopt if the order is not a stop of a route plan.
 */
std::optional<RoutePlanStop> GetRoutePlanStop(const Order &order)
{
	if (order.IsType(OT_GOTO_STATION)) {
		const Station *st = Station::GetIfValid(order.GetDestination().ToStationID());
		if (st == nullptr || !st->facilities.Test(StationFacility::Train)) return std::nullopt;
		bool stop = !(order.GetNonStopType() & ONSF_NO_STOP_AT_DESTINATION_STATION);
		return RoutePlanStop{ st->index, stop ? RoutePlanDeparture::ForwardOrReverse : RoutePlanDeparture::Forward };
	}
	if (order.IsType(OT_GOTO_WAYPOINT)) {
		const Waypoint *wp = Waypoint::GetIfValid(order.GetDestination().ToStationID());
		if (wp == nullptr || !wp->facilities.Test(StationFacility::Train)) return std::nullopt;
		bool reverse = order.GetWaypointFlags().Test(OrderWaypointFlag::Reverse);
		return RoutePlanStop{ wp->index, reverse ? RoutePlanDeparture::Reverse : RoutePlanDeparture::Forward };
	}
	return std::nullopt;
}

/**
 * Check whether a route plan still matches the stops of an order list.
 * @param plan The route plan.
 * @param orders The order list.
 * @return true iff the stops are unchanged.
 */
bool RoutePlanMatchesOrders(const RoutePlan &plan, const OrderList &orders)
{
	size_t k = 0;
	for (VehicleOrderID i = 0; i < orders.GetNumOrders(); i++) {
		std::optional<RoutePlanStop> stop = GetRoutePlanStop(*orders.GetOrderAt(i));
		if (!stop.has_value()) continue;
		if (k >= plan.stops.size() || plan.stop_orders[k] != i || !(plan.stops[k] == *stop)) return false;
		k++;
	}
	return k == plan.stops.size();
}

/**
 * Get the first tile of the platform which ends at an arrival state.
 * @param target The arrival state.
 * @return The first platform tile, in the direction of travel.
 */
TileIndex GetRoutePlanPlatformStart(const TrainRoutePlanTarget &target)
{
	if (!IsRailStationTile(target.tile)) return target.tile;

	const TileIndexDiff diff = TileOffsByDiagDir(ReverseDiagDir(TrackdirToExitdir(target.td)));
	TileIndex start = target.tile;
	while (IsCompatibleTrainStationTile(start + diff, target.tile)) start += diff;
	return start;
}

/**
 * Get all arrival states at a station or waypoint which can be reached: for every platform, both ends,
 * if track leads into the platform from the other end.
 * @param v The train.
 * @param station The station or waypoint.
 * @param[out] states The arrival states.
 */
void GetRoutePlanArrivalStates(const Train *v, StationID station, std::vector<TrainRoutePlanTarget> &states)
{
	const BaseStation *st = BaseStation::Get(station);
	if (st->train_station.tile == INVALID_TILE) return;

	for (TileIndex tile : st->train_station) {
		if (!st->TileBelongsToRailStation(tile) || IsStationTileBlocked(tile)) continue;

		const DiagDirection dir = AxisToDiagDir(GetRailStationAxis(tile));
		for (DiagDirection arrival_dir : { dir, ReverseDiagDir(dir) }) {
			/* Only the end of a platform in the direction of travel. */
			if (IsRailStationTile(tile) && IsCompatibleTrainStationTile(TileAddByDiagDir(tile, arrival_dir), tile)) continue;

			const TrainRoutePlanTarget state{ tile, DiagDirToDiagTrackdir(arrival_dir) };

			/* The platform must be reachable from the other end. */
			CFollowTrackRailRoutePlan ft(v);
			if (!ft.Follow(GetRoutePlanPlatformStart(state), ReverseTrackdir(state.td))) continue;

			states.push_back(state);
		}
	}
}

/**
 * Search all paths of a leg of a route plan.
 * @param v The train.
 * @param key The leg.
 * @return The leg.
 */
std::unique_ptr<RoutePlanLeg> ComputeRoutePlanLeg(const Train *v, const RoutePlanLegKey &key)
{
	const auto start_time = std::chrono::steady_clock::now();

	auto leg = std::make_unique<RoutePlanLeg>();

	/* Record the area of all tiles examined while searching the leg.
	 * The departure states and the cost estimates depend on the layout of both stations. */
	_route_plan_recording = &leg->area;
	leg->area.Add(BaseStation::Get(key.from.station)->train_station);
	leg->area.Add(BaseStation::Get(key.to)->train_station);

	GetRoutePlanArrivalStates(v, key.from.station, leg->departures);

	for (size_t i = 0; i < leg->departures.size() && i < UINT16_MAX; i++) {
		const TrainRoutePlanTarget &departure = leg->departures[i];

		YapfRoutePlanSearchOrigin origin{ INVALID_TILE, INVALID_TRACKDIR, INVALID_TILE, INVALID_TRACKDIR, key.params.reverse_penalised };
		if (key.from.departure != RoutePlanDeparture::Reverse) {
			origin.forward_tile = departure.tile;
			origin.forward_td = departure.td;
		}
		if (key.from.departure != RoutePlanDeparture::Forward) {
			origin.reverse_tile = GetRoutePlanPlatformStart(departure);
			origin.reverse_td = ReverseTrackdir(departure.td);
		}

		YapfRoutePlanSearchResult result;
		YapfTrainRoutePlanSearch(v, origin, key.to, result);

		leg->margin = std::max(leg->margin, result.margin);
		for (const YapfRoutePlanSearchTarget &target : result.targets) {
			auto target_it = std::ranges::find_if(leg->targets, [&](const TrainRoutePlanTarget &t) { return t.tile == target.tile && t.td == target.td; });
			if (target_it == leg->targets.end()) {
				if (leg->targets.size() >= UINT16_MAX) continue;
				target_it = leg->targets.insert(leg->targets.end(), { target.tile, target.td });
			}
			leg->paths.push_back({ static_cast<uint16_t>(i), static_cast<uint16_t>(target_it - leg->targets.begin()), target.cost });
		}
		leg->keys.insert(leg->keys.end(), result.keys.begin(), result.keys.end());
	}

	_route_plan_recording = nullptr;

	std::sort(leg->keys.begin(), leg->keys.end());
	leg->keys.erase(std::unique(leg->keys.begin(), leg->keys.end()), leg->keys.end());
	leg->keys.shrink_to_fit();

	Debug(yapf, 3, "Route plan leg: station {} -> {}, {} departures, {} targets, {} paths, {} keys, {} us",
			key.from.station, key.to, leg->departures.size(), leg->targets.size(), leg->paths.size(), leg->keys.size(),
			std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count());

	return leg;
}

/**
 * Check whether two legs have the same result.
 * @param a The first leg.
 * @param b The second leg.
 * @return true iff the legs are equal.
 */
bool RoutePlanLegsEqual(const RoutePlanLeg &a, const RoutePlanLeg &b)
{
	auto target_eq = [](const TrainRoutePlanTarget &x, const TrainRoutePlanTarget &y) { return x.tile == y.tile && x.td == y.td; };
	auto path_eq = [](const RoutePlanPath &x, const RoutePlanPath &y) { return x.from == y.from && x.to == y.to && x.cost == y.cost; };
	return std::ranges::equal(a.departures, b.departures, target_eq) && std::ranges::equal(a.targets, b.targets, target_eq) &&
			std::ranges::equal(a.paths, b.paths, path_eq) && a.keys == b.keys && a.margin == b.margin;
}

/**
 * Get the leg of a route plan, from the cache or by searching all paths of the leg.
 * @param v The train.
 * @param key The leg.
 * @return The leg.
 */
const RoutePlanLeg *GetRoutePlanLeg(const Train *v, const RoutePlanLegKey &key)
{
	auto it = _route_plan_legs.find(key);
	if (it != _route_plan_legs.end()) {
		if (GetDebugLevel(DebugLevelID::yapfdesync) > 0 || GetDebugLevel(DebugLevelID::desync) >= 2) {
			/* The cached leg must be the same as a new one, as the cache isn't saved. */
			if (!RoutePlanLegsEqual(*it->second, *ComputeRoutePlanLeg(v, key))) {
				Debug(desync, 0, "CACHE ERROR: route plan leg: station {} -> {}", key.from.station, key.to);
			}
		}
		return it->second.get();
	}

	return _route_plan_legs.emplace(key, ComputeRoutePlanLeg(v, key)).first->second.get();
}

/**
 * Determine the remaining cost from each target of a leg to the end of the order list.
 * @param leg The leg whose targets to evaluate.
 * @param next_leg The leg which follows.
 * @param next_remaining The remaining cost from each target of the next leg.
 * @param[out] remaining The remaining cost from each target of the leg, or ROUTE_PLAN_UNREACHABLE if the order list can't be completed from it.
 */
void CalculateRoutePlanRemainingCosts(const RoutePlanLeg &leg, const RoutePlanLeg &next_leg, const std::vector<int64_t> &next_remaining, std::vector<int64_t> &remaining)
{
	remaining.assign(leg.targets.size(), ROUTE_PLAN_UNREACHABLE);

	/* If the next stop can't be reached at all, nothing is known about the remaining cost. */
	if (next_leg.paths.empty()) {
		std::fill(remaining.begin(), remaining.end(), 0);
		return;
	}

	for (size_t j = 0; j < leg.targets.size(); j++) {
		const TrainRoutePlanTarget &target = leg.targets[j];
		auto departure_it = std::ranges::find_if(next_leg.departures, [&](const TrainRoutePlanTarget &d) { return d.tile == target.tile && d.td == target.td; });
		if (departure_it == next_leg.departures.end()) continue;

		const uint16_t departure = static_cast<uint16_t>(departure_it - next_leg.departures.begin());
		for (const RoutePlanPath &path : next_leg.paths) {
			if (path.from != departure || next_remaining[path.to] == ROUTE_PLAN_UNREACHABLE) continue;
			remaining[j] = std::min(remaining[j], path.cost + next_remaining[path.to]);
		}
	}
}

/**
 * Compute the route plan of an order list.
 * @param v The train.
 * @param plan The route plan, with stops already set.
 */
void ComputeRoutePlan(const Train *v, RoutePlan &plan)
{
	const RoutePlanTrainParams params = GetRoutePlanTrainParams(v);
	const size_t n = plan.stops.size();

	/* Leg k leads from stop k - 1 to stop k, the leg towards the first stop starts at the last stop. */
	plan.legs.resize(n);
	for (size_t k = 0; k < n; k++) {
		plan.legs[k] = GetRoutePlanLeg(v, { params, plan.stops[(k + n - 1) % n], plan.stops[k].station });
	}

	/* Dynamic programming over the legs, from the end to the start of the order list:
	 * the remaining cost from each target to the end of the order list (i.e. arriving at the first stop again).
	 * The remaining costs of the targets of the first stop are the cost of the next pass of the order list. */
	std::vector<std::vector<int64_t>> remaining(n);
	for (size_t k = 0; k < n; k++) remaining[k].assign(plan.legs[k]->targets.size(), 0);
	for (size_t k = n - 1; k >= 1; k--) {
		const size_t next = (k + 1) % n;
		CalculateRoutePlanRemainingCosts(*plan.legs[k], *plan.legs[next], remaining[next], remaining[k]);
	}
	{
		const size_t next = 1 % n;
		std::vector<int64_t> next_remaining = remaining[next];
		CalculateRoutePlanRemainingCosts(*plan.legs[0], *plan.legs[next], next_remaining, remaining[0]);
	}

	/* Convert the remaining costs to extra costs of the targets, relative to the best target. */
	plan.target_costs.resize(n);
	plan.guides.resize(n);
	for (size_t k = 0; k < n; k++) {
		const RoutePlanLeg &leg = *plan.legs[k];
		std::vector<int> &costs = plan.target_costs[k];

		int64_t best = ROUTE_PLAN_UNREACHABLE;
		for (int64_t r : remaining[k]) best = std::min(best, r);

		int worst_finite = 0;
		costs.resize(leg.targets.size());
		for (size_t j = 0; j < leg.targets.size(); j++) {
			if (remaining[k][j] == ROUTE_PLAN_UNREACHABLE) continue;
			costs[j] = static_cast<int>(std::min<int64_t>(remaining[k][j] - best, ROUTE_PLAN_MAX_COST));
			worst_finite = std::max(worst_finite, costs[j]);
		}

		/* Any free planned platform is preferred to any occupied one, see TrainRoutePlanGuide::GetTargetCost. */
		const int occupied_penalty = std::min(leg.margin + worst_finite + YAPF_TILE_LENGTH, ROUTE_PLAN_MAX_COST);

		/* Targets from which the order list can't be completed are only used if nothing else is possible. */
		for (size_t j = 0; j < leg.targets.size(); j++) {
			if (remaining[k][j] == ROUTE_PLAN_UNREACHABLE) costs[j] = std::min(2 * occupied_penalty, ROUTE_PLAN_MAX_COST);
		}

		TrainRoutePlanGuide &guide = plan.guides[k];
		guide.keys = leg.keys;
		guide.targets = leg.targets;
		guide.target_costs = costs;
		guide.unknown_target_cost = worst_finite;
		guide.occupied_penalty = occupied_penalty;

		if (GetDebugLevel(DebugLevelID::yapf) >= 4) {
			std::string targets;
			for (size_t j = 0; j < leg.targets.size(); j++) {
				fmt::format_to(std::back_inserter(targets), " [{}x{} td {}: {}]", TileX(leg.targets[j].tile), TileY(leg.targets[j].tile), leg.targets[j].td, costs[j]);
			}
			Debug(yapf, 4, "Route plan stop {} (order {}, station {}): occupied penalty {}, targets:{}", k, plan.stop_orders[k], plan.stops[k].station, occupied_penalty, targets);
		}
	}
}

/**
 * Get the route plan of a train, from the cache or by computing it.
 * @param v The train.
 * @return The route plan, or nullptr if the train has no stops.
 */
const RoutePlan *GetRoutePlan(const Train *v)
{
	if (_route_plan_cache_version != _route_plan_version) {
		_route_plans.clear();
		_route_plan_legs.clear();
		_route_plan_cache_version = _route_plan_version;
	}

	const OrderList *orders = v->orders;
	if (orders == nullptr) return nullptr;

	std::unique_ptr<RoutePlan> &plan = _route_plans[{ orders->index.base(), GetRoutePlanTrainParams(v) }];
	if (plan != nullptr && RoutePlanMatchesOrders(*plan, *orders)) return plan.get();

	plan = std::make_unique<RoutePlan>();
	for (VehicleOrderID i = 0; i < orders->GetNumOrders(); i++) {
		std::optional<RoutePlanStop> stop = GetRoutePlanStop(*orders->GetOrderAt(i));
		if (!stop.has_value()) continue;
		plan->stops.push_back(*stop);
		plan->stop_orders.push_back(i);
	}
	if (!plan->stops.empty()) ComputeRoutePlan(v, *plan);

	return plan.get();
}

/**
 * Check whether the platform of an arrival state is occupied or reserved by another train.
 * @param v The train.
 * @param target The arrival state.
 * @return true iff another train occupies or reserved the platform.
 */
bool IsRoutePlanPlatformOccupied(const Train *v, const TrainRoutePlanTarget &target)
{
	const TileIndexDiff diff = TileOffsByDiagDir(ReverseDiagDir(TrackdirToExitdir(target.td)));
	const bool is_station = IsRailStationTile(target.tile);
	TileIndex tile = target.tile;
	do {
		if (HasStationReservation(tile)) {
			const Train *owner = GetTrainForReservation(tile, GetRailStationTrack(tile));
			if (owner == nullptr || owner->First() != v) return true;
		}
		if (HasVehicleOnTile<VehicleType::Train>(tile, [&](const Train *u) { return u->First() != v; })) return true;

		tile += diff;
	} while (is_station && IsCompatibleTrainStationTile(tile, target.tile));

	return false;
}

} // namespace

/**
 * Check whether a pathfinder node is part of a planned path of this leg.
 * @param tile The tile of the node.
 * @param td The trackdir of the node.
 * @return true iff the node is planned.
 */
bool TrainRoutePlanGuide::IsPlannedNode(TileIndex tile, Trackdir td) const
{
	return std::binary_search(this->keys.begin(), this->keys.end(), GetTrainRoutePlanNodeKey(tile, td));
}

/**
 * Get the extra cost of arriving at a platform of the destination: the remaining planned cost to the end of
 * the order list relative to the best platform, and a penalty if the platform is occupied or reserved by another train.
 * @param v The train.
 * @param tile The platform end tile reached.
 * @param td The direction of travel.
 * @return The extra cost.
 */
int TrainRoutePlanGuide::GetTargetCost(const Train *v, TileIndex tile, Trackdir td) const
{
	int cost = this->unknown_target_cost;
	for (size_t i = 0; i < this->targets.size(); i++) {
		if (this->targets[i].tile == tile && this->targets[i].td == td) {
			cost = this->target_costs[i];
			break;
		}
	}
	if (IsRoutePlanPlatformOccupied(v, { tile, td })) cost += this->occupied_penalty;
	return cost;
}

/**
 * Get the route plan guidance for the pathfinder, towards the current destination of a train.
 * The route plan is computed if necessary.
 * @param v The train.
 * @return The guidance, or nullptr if there is no route plan for the current destination.
 */
const TrainRoutePlanGuide *GetTrainRoutePlanGuide(const Train *v)
{
	if (!_settings_game.pf.train_route_plan) return nullptr;
	if (!v->current_order.IsType(OT_GOTO_STATION) && !v->current_order.IsType(OT_GOTO_WAYPOINT)) return nullptr;

	const RoutePlan *plan = GetRoutePlan(v);
	if (plan == nullptr) return nullptr;

	auto it = std::ranges::lower_bound(plan->stop_orders, v->cur_real_order_index);
	if (it == plan->stop_orders.end() || *it != v->cur_real_order_index) return nullptr;

	const size_t k = it - plan->stop_orders.begin();
	if (plan->stops[k].station != v->current_order.GetDestination().ToStationID()) return nullptr;

	return &plan->guides[k];
}

/**
 * Record tiles examined by a route plan search.
 * @param tile1 A tile, or INVALID_TILE.
 * @param tile2 A tile, or INVALID_TILE; the tiles in a straight line between both tiles were examined as well.
 */
void RecordTrainRoutePlanTiles(TileIndex tile1, TileIndex tile2)
{
	if (_route_plan_recording == nullptr) return;
	_route_plan_recording->Add(tile1);
	_route_plan_recording->Add(tile2);
}

/** Invalidate all route plans, e.g. because the settings changed. */
void InvalidateTrainRoutePlans()
{
	_route_plan_version++;
}

/**
 * Invalidate the route plans which depend on a tile, because the track layout of the tile changed.
 * @param tile The tile, or INVALID_TILE to invalidate all route plans.
 */
void InvalidateTrainRoutePlans(TileIndex tile)
{
	if (tile == INVALID_TILE || tile >= Map::Size()) {
		InvalidateTrainRoutePlans();
		return;
	}
	if (_route_plan_legs.empty()) return;

	/* Building a station only notifies one tile of each platform, so consider the whole station. */
	StationID station = StationID::Invalid();
	TileArea station_area{};
	if (HasStationTileRail(tile)) {
		const BaseStation *st = BaseStation::GetByTile(tile);
		station = st->index;
		station_area = st->train_station;
	}

	bool invalidated = false;
	for (auto it = _route_plan_legs.begin(); it != _route_plan_legs.end();) {
		const RoutePlanLegKey &key = it->first;
		const RoutePlanArea &area = it->second->area;
		if (area.Intersects(tile) || (station != StationID::Invalid() && (key.from.station == station || key.to == station || area.Intersects(station_area)))) {
			it = _route_plan_legs.erase(it);
			invalidated = true;
		} else {
			++it;
		}
	}

	/* The route plans refer to the legs, but are cheap to recompute from the legs which remain. */
	if (invalidated) _route_plans.clear();
}

/** Clear the route plan caches, e.g. when a new game is started or loaded. */
void ClearTrainRoutePlanCaches()
{
	_route_plans.clear();
	_route_plan_legs.clear();
	_route_plan_version++;
	_route_plan_cache_version = _route_plan_version;
}
