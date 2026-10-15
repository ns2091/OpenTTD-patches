/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file yapf_destrail.hpp Determining the destination for rail vehicles. */

#ifndef YAPF_DESTRAIL_HPP
#define YAPF_DESTRAIL_HPP

#include "../../train.h"
#include "../../train_route_plan.h"
#include "../pathfinder_func.h"
#include "../pathfinder_type.h"

class CYapfDestinationRailBase {
protected:
	RailTypes compatible_railtypes;

public:
	void SetDestination(const Train *v, bool override_rail_type = false)
	{
		this->compatible_railtypes = v->compatible_railtypes;
		if (override_rail_type) this->compatible_railtypes.Set(GetAllCompatibleRailTypes(v->railtypes));
	}

	bool IsCompatibleRailType(RailType rt)
	{
		return this->compatible_railtypes.Test(rt);
	}

	RailTypes GetCompatibleRailTypes() const
	{
		return this->compatible_railtypes;
	}

	/**
	 * Get the extra cost of reaching a destination.
	 * @param tile The destination tile reached.
	 * @param td The trackdir with which the destination was reached.
	 * @return The extra cost.
	 */
	inline int PfDestinationExtraCost([[maybe_unused]] TileIndex tile, [[maybe_unused]] Trackdir td) const
	{
		return 0;
	}
};

template <class Types>
class CYapfDestinationAnyDepotRailT : public CYapfDestinationRailBase {
public:
	typedef typename Types::Tpf Tpf; ///< the pathfinder class (derived from THIS class)
	typedef typename Types::NodeList::Item Node; ///< this will be our node type
	typedef typename Node::Key Key; ///< key to hash tables

	/** @copydoc CYapfBaseT::Yapf */
	Tpf &Yapf()
	{
		return *static_cast<Tpf *>(this);
	}

	/** @copydoc CYapfBaseT::PfDetectDestinationFunc */
	inline bool PfDetectDestination(Node &n)
	{
		return this->PfDetectDestination(n.GetLastTile(), n.GetLastTrackdir());
	}

	/** @copydoc CYapfBaseT::PfDetectDestinationTileFunc */
	inline bool PfDetectDestination(TileIndex tile, [[maybe_unused]] Trackdir td)
	{
		return IsRailDepotTile(tile);
	}

	/** @copydoc CYapfBaseT::PfCalcEstimateFunc */
	inline bool PfCalcEstimate(Node &n)
	{
		n.estimate = n.cost;
		return true;
	}

	inline int TeleportCost(TileIndex cur_tile, TileIndex prev_tile)
	{
		return 0;
	}
};

template <class Types>
class CYapfDestinationAnySafeTileRailT : public CYapfDestinationRailBase {
public:
	typedef typename Types::Tpf Tpf; ///< the pathfinder class (derived from THIS class)
	typedef typename Types::NodeList::Item Node; ///< this will be our node type
	typedef typename Node::Key Key; ///< key to hash tables
	typedef typename Types::TrackFollower TrackFollower; ///< TrackFollower. Need to typedef for gcc 2.95

	/** @copydoc CYapfBaseT::Yapf */
	Tpf &Yapf()
	{
		return *static_cast<Tpf *>(this);
	}

	/** @copydoc CYapfBaseT::PfDetectDestinationFunc */
	inline bool PfDetectDestination(Node &n)
	{
		return this->PfDetectDestination(n.GetLastTile(), n.GetLastTrackdir());
	}

	/** @copydoc CYapfBaseT::PfDetectDestinationTileFunc */
	inline bool PfDetectDestination(TileIndex tile, Trackdir td)
	{
		return IsSafeWaitingPosition(Yapf().GetVehicle(), tile, td, true, !TrackFollower::Allow90degTurns()) &&
				IsWaitingPositionFree(Yapf().GetVehicle(), tile, td, !TrackFollower::Allow90degTurns());
	}

	/** @copydoc CYapfBaseT::PfCalcEstimateFunc */
	inline bool PfCalcEstimate(Node &n)
	{
		n.estimate = n.cost;
		return true;
	}

	inline int TeleportCost(TileIndex cur_tile, TileIndex prev_tile)
	{
		return 0;
	}
};

template <class Types>
class CYapfDestinationTileOrStationRailT : public CYapfDestinationRailBase {
public:
	typedef typename Types::Tpf Tpf; ///< the pathfinder class (derived from THIS class)
	typedef typename Types::NodeList::Item Node; ///< this will be our node type
	typedef typename Node::Key Key; ///< key to hash tables

protected:
	TileIndex dest_tile;
	TrackdirBits dest_trackdirs;
	StationID dest_station_id;
	bool any_depot;
	const TrainRoutePlanGuide *route_plan_guide = nullptr; ///< Route plan of the leg towards the destination, if any.
	bool route_plan_restrict = false;                      ///< Whether to only follow the planned paths where they exist.

	/** @copydoc CYapfBaseT::Yapf */
	Tpf &Yapf()
	{
		return *static_cast<Tpf *>(this);
	}

public:
	void SetDestination(const Train *v)
	{
		this->any_depot = false;
		switch (v->current_order.GetType()) {
			case OT_GOTO_WAYPOINT:
				if (!Waypoint::Get(v->current_order.GetDestination().ToStationID())->IsSingleTile()) {
					/* In case of 'complex' waypoints we need to do a look
					 * ahead. This look ahead messes a bit about, which
					 * means that it 'corrupts' the cache. To prevent this
					 * we disable caching when we're looking for a complex
					 * waypoint. */
					Yapf().DisableCache(true);
				}
				[[fallthrough]];

			case OT_GOTO_STATION:
				this->dest_tile = CalcClosestStationTile(v->current_order.GetDestination().ToStationID(), v->GetMovingFront()->tile, v->current_order.IsType(OT_GOTO_STATION) ? StationType::Rail : StationType::RailWaypoint);
				this->dest_station_id = v->current_order.GetDestination().ToStationID();
				this->dest_trackdirs = INVALID_TRACKDIR_BIT;
				break;

			case OT_GOTO_DEPOT:
				if (v->current_order.GetDepotActionType() & ODATFB_NEAREST_DEPOT) {
					this->any_depot = true;
				}
				[[fallthrough]];

			default:
				this->dest_tile = (v->dest_tile == INVALID_TILE) ? TileIndex{} : v->dest_tile;
				this->dest_station_id = StationID::Invalid();
				this->dest_trackdirs = GetTileTrackdirBits(this->dest_tile, TransportType::Rail, 0);
				break;
		}
		this->CYapfDestinationRailBase::SetDestination(v);
	}

	/**
	 * Set the route plan guidance of the leg towards the destination.
	 * @param guide The route plan guidance, or nullptr.
	 * @param plan_only Whether to only follow the planned paths where they exist.
	 */
	void SetRoutePlanGuide(const TrainRoutePlanGuide *guide, bool plan_only)
	{
		this->route_plan_guide = guide;
		this->route_plan_restrict = plan_only && guide != nullptr;
	}

	/**
	 * Restrict the choices at a junction to the planned paths, if any of the choices is part of a planned path.
	 * @param follower The track follower which has followed the track up to the junction.
	 */
	template <class TrackFollower>
	inline void RoutePlanFilterChoices(TrackFollower &follower) const
	{
		if (!this->route_plan_restrict) return;
		if (KillFirstBit(follower.new_td_bits) == TRACKDIR_BIT_NONE) return;

		TrackdirBits planned = TRACKDIR_BIT_NONE;
		for (Trackdir td : SetTrackdirBitIterator(follower.new_td_bits)) {
			if (this->route_plan_guide->IsPlannedNode(follower.new_tile, td)) planned |= TrackdirToTrackdirBits(td);
		}
		if (planned != TRACKDIR_BIT_NONE) follower.new_td_bits = planned;
	}

	/**
	 * Get the extra cost of reaching a destination: the remaining cost of the route plan, and the penalty for occupied platforms.
	 * @param tile The destination tile reached.
	 * @param td The trackdir with which the destination was reached.
	 * @return The extra cost.
	 */
	inline int PfDestinationExtraCost(TileIndex tile, Trackdir td)
	{
		if (this->route_plan_guide == nullptr || this->dest_station_id == StationID::Invalid()) return 0;
		return this->route_plan_guide->GetTargetCost(Yapf().GetVehicle(), tile, td);
	}

	/** @copydoc CYapfBaseT::PfDetectDestinationFunc */
	inline bool PfDetectDestination(Node &n)
	{
		return this->PfDetectDestination(n.GetLastTile(), n.GetLastTrackdir());
	}

	/** @copydoc CYapfBaseT::PfDetectDestinationTileFunc */
	inline bool PfDetectDestination(TileIndex tile, Trackdir td)
	{
		if (this->dest_station_id != StationID::Invalid()) {
			return HasStationTileRail(tile)
				&& (GetStationIndex(tile) == this->dest_station_id)
				&& (GetRailStationTrack(tile) == TrackdirToTrack(td));
		}

		if (this->any_depot) {
			return IsRailDepotTile(tile);
		}

		return (tile == this->dest_tile) && HasTrackdir(this->dest_trackdirs, td);
	}

	/** @copydoc CYapfBaseT::PfCalcEstimateFunc */
	inline bool PfCalcEstimate(Node &n)
	{
		if (this->PfDetectDestination(n)) {
			n.estimate = n.cost;
			return true;
		}

		n.estimate = n.cost + OctileDistanceCost(n.GetLastTile(), n.GetLastTrackdir(), this->dest_tile);
		assert(n.estimate >= n.parent->estimate);
		return true;
	}

	inline int TeleportCost(TileIndex cur_tile, TileIndex prev_tile)
	{
		auto calculate_distance_cost = [&](TileIndex t, int d_adjust) -> int {
			int x1 = 2 * TileX(t);
			int y1 = 2 * TileY(t);
			int x2 = 2 * TileX(this->dest_tile);
			int y2 = 2 * TileY(this->dest_tile);
			int dx = abs(x1 - x2) + d_adjust;
			int dy = abs(y1 - y2);
			int dmin = std::min(dx, dy) + d_adjust; // up to 2x track exit dir tile offsets in opposite directions
			int dxy = abs(dx - dy) + d_adjust; // "
			return dmin * YAPF_TILE_CORNER_LENGTH + (dxy - 1) * (YAPF_TILE_LENGTH / 2);
		};
		return std::max<int>(0, calculate_distance_cost(prev_tile, 8) - calculate_distance_cost(cur_tile, 0));
	}
};

/** Destination provider for train route planning: all platforms of a station or waypoint. */
template <class Types>
class CYapfDestinationRoutePlanRailT : public CYapfDestinationRailBase {
public:
	typedef typename Types::Tpf Tpf; ///< the pathfinder class (derived from THIS class)
	typedef typename Types::NodeList::Item Node; ///< this will be our node type
	typedef typename Node::Key Key; ///< key to hash tables

protected:
	StationID dest_station_id = StationID::Invalid();
	int dest_x0 = 0; ///< Lowest x coordinate of the destination, in half tiles.
	int dest_y0 = 0; ///< Lowest y coordinate of the destination, in half tiles.
	int dest_x1 = 0; ///< Highest x coordinate of the destination, in half tiles.
	int dest_y1 = 0; ///< Highest y coordinate of the destination, in half tiles.

	/** @copydoc CYapfBaseT::Yapf */
	Tpf &Yapf()
	{
		return *static_cast<Tpf *>(this);
	}

public:
	void SetDestination(const Train *v, StationID station)
	{
		this->CYapfDestinationRailBase::SetDestination(v);
		this->dest_station_id = station;

		const BaseStation *st = BaseStation::Get(station);
		TileArea ta = st->train_station;
		if (ta.tile == INVALID_TILE) ta = TileArea(st->xy, 1, 1);
		this->dest_x0 = 2 * TileX(ta.tile);
		this->dest_y0 = 2 * TileY(ta.tile);
		this->dest_x1 = 2 * (TileX(ta.tile) + ta.w - 1);
		this->dest_y1 = 2 * (TileY(ta.tile) + ta.h - 1);
	}

	/** @copydoc CYapfBaseT::PfDetectDestinationFunc */
	inline bool PfDetectDestination(Node &n)
	{
		return this->PfDetectDestination(n.GetLastTile(), n.GetLastTrackdir());
	}

	/** @copydoc CYapfBaseT::PfDetectDestinationTileFunc */
	inline bool PfDetectDestination(TileIndex tile, Trackdir td)
	{
		return HasStationTileRail(tile)
			&& (GetStationIndex(tile) == this->dest_station_id)
			&& (GetRailStationTrack(tile) == TrackdirToTrack(td));
	}

	/**
	 * Calculate the cost estimate: the octile distance to the nearest point of the destination area.
	 * This is a lower bound of the distance to every platform of the destination.
	 * The estimate is never lower than the estimate of the parent node, so that destination nodes
	 * are found in order of their cost, and closed nodes are never improved upon.
	 * @copydoc CYapfBaseT::PfCalcEstimateFunc
	 */
	inline bool PfCalcEstimate(Node &n)
	{
		int estimate = n.cost;
		if (!this->PfDetectDestination(n)) {
			static constexpr int dg_dir_to_x_offs[] = {-1, 0, 1, 0};
			static constexpr int dg_dir_to_y_offs[] = {0, 1, 0, -1};

			const DiagDirection exitdir = TrackdirToExitdir(n.GetLastTrackdir());
			const int x = 2 * TileX(n.GetLastTile()) + dg_dir_to_x_offs[static_cast<int>(exitdir)];
			const int y = 2 * TileY(n.GetLastTile()) + dg_dir_to_y_offs[static_cast<int>(exitdir)];
			const int dx = (x < this->dest_x0) ? (this->dest_x0 - x) : (x > this->dest_x1 ? x - this->dest_x1 : 0);
			const int dy = (y < this->dest_y0) ? (this->dest_y0 - y) : (y > this->dest_y1 ? y - this->dest_y1 : 0);
			const int dmin = std::min(dx, dy);
			const int dxy = abs(dx - dy);
			estimate += std::max<int>(0, dmin * YAPF_TILE_CORNER_LENGTH + (dxy - 1) * (YAPF_TILE_LENGTH / 2));
		}
		n.estimate = std::max(estimate, n.parent->estimate);
		return true;
	}

	inline int TeleportCost(TileIndex, TileIndex)
	{
		return 0;
	}
};

#endif /* YAPF_DESTRAIL_HPP */
