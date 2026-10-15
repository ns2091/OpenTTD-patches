/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/**
 * @file train_route_plan.h Route planning for trains over their whole order list.
 *
 * A route plan is computed for the sequence of station and waypoint orders of a train's order list.
 * For each leg (from one stop to the next) the shortest paths from every platform of the previous stop
 * to every reachable platform of the next stop are searched, using only the static parts of the
 * pathfinder costs (track layout, curves, slopes, signal types, stations, ...).
 * The cost to complete the order list from each platform is then determined by dynamic programming
 * over the legs, from the start to the end of the order list.
 *
 * When running, the pathfinder only follows planned paths where they exist, and when the train can
 * choose between platforms of its destination, the platforms are used in order of their total planned
 * cost, preferring platforms which are not occupied or reserved by other trains.
 *
 * Route plans only depend on static game state, they are cached and not saved.
 * The searches of each leg record the area of the tiles they examine, and the leg is
 * invalidated whenever the track layout changes within that area, or at its stations.
 * All route plans are invalidated when the settings change.
 */

#ifndef TRAIN_ROUTE_PLAN_H
#define TRAIN_ROUTE_PLAN_H

#include "tile_type.h"
#include "track_type.h"
#include <span>

struct Train;

/** An arrival state at a stop: the platform end tile reached, and the direction of travel. */
struct TrainRoutePlanTarget {
	TileIndex tile; ///< Platform end tile (as reached by the pathfinder).
	Trackdir td;    ///< Direction of travel.
};

/**
 * Get the key used in route plans to identify a pathfinder node.
 * @param tile The tile of the node.
 * @param td The trackdir of the node.
 * @return The key.
 */
inline uint32_t GetTrainRoutePlanNodeKey(TileIndex tile, Trackdir td)
{
	return (tile.base() << 4) | td;
}

/** Pathfinder guidance for the leg towards the current destination of a train, derived from its route plan. */
struct TrainRoutePlanGuide {
	std::span<const uint32_t> keys;                ///< Sorted node keys of the planned paths of this leg.
	std::span<const TrainRoutePlanTarget> targets; ///< Reachable arrival states at the destination.
	std::span<const int> target_costs;             ///< Extra cost of each target: the remaining planned cost relative to the best target.
	int unknown_target_cost = 0;                   ///< Extra cost for arrival states which are not part of the plan.
	int occupied_penalty = 0;                      ///< Penalty for platforms which are occupied or reserved by other trains.

	/**
	 * Check whether there are any planned paths towards the destination.
	 * @return true iff there are planned paths.
	 */
	bool HasPlannedPaths() const
	{
		return !this->keys.empty();
	}

	bool IsPlannedNode(TileIndex tile, Trackdir td) const;
	int GetTargetCost(const Train *v, TileIndex tile, Trackdir td) const;
};

const TrainRoutePlanGuide *GetTrainRoutePlanGuide(const Train *v);
void RecordTrainRoutePlanTiles(TileIndex tile1, TileIndex tile2);
void InvalidateTrainRoutePlans();
void InvalidateTrainRoutePlans(TileIndex tile);
void ClearTrainRoutePlanCaches();

#endif /* TRAIN_ROUTE_PLAN_H */
