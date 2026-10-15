/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file yapf.h Entry point for OpenTTD to YAPF. */

#ifndef YAPF_H
#define YAPF_H

#include "../../direction_type.h"
#include "../../track_type.h"
#include "../../vehicle_type.h"
#include "../../ship.h"
#include "../../roadveh.h"
#include "../../station_type.h"
#include "../pathfinder_type.h"
#include <vector>

/**
 * Finds the best path for given ship using YAPF.
 * @param v        the ship that needs to find a path
 * @param tile     the tile to find the path from (should be next tile the ship is about to enter)
 * @param path_found [out] Whether a path has been found (true) or has been guessed (false)
 * @param path_cache Cache of a previously chosen path.
 * @return         the best trackdir for next turn or INVALID_TRACK if the path could not be found
 */
Track YapfShipChooseTrack(const Ship *v, TileIndex tile, bool &path_found, ShipPathCache &path_cache);

/**
 * Returns true if it is better to reverse the ship before leaving depot using YAPF.
 * @param v the ship leaving the depot
 * @param trackdir [out] the best of all possible reversed trackdirs
 * @return true if reversing is better
 */
bool YapfShipCheckReverse(const Ship *v, Trackdir *trackdir);

/**
 * Finds the best path for given road vehicle using YAPF.
 * @param v         the RV that needs to find a path
 * @param tile      the tile to find the path from (should be next tile the RV is about to enter)
 * @param enterdir  diagonal direction which the RV will enter this new tile from
 * @param trackdirs available trackdirs on the new tile (to choose from)
 * @param path_found [out] Whether a path has been found (true) or has been guessed (false)
 * @param path_cache Cache of a previously chosen path.
 * @return          the best trackdir for next turn or INVALID_TRACKDIR if the path could not be found
 */
Trackdir YapfRoadVehicleChooseTrack(const RoadVehicle *v, TileIndex tile, DiagDirection enterdir, TrackdirBits trackdirs, bool &path_found, RoadVehPathCache &path_cache);

/**
 * Finds the best path for given train using YAPF.
 * @param v        the train that needs to find a path
 * @param tile     the tile to find the path from (should be next tile the train is about to enter)
 * @param enterdir diagonal direction which the RV will enter this new tile from
 * @param tracks   available trackdirs on the new tile (to choose from)
 * @param path_found [out] Whether a path has been found (true) or has been guessed (false)
 * @param reserve_track indicates whether YAPF should try to reserve the found path
 * @param target   [out] the target tile of the reservation, free is set to true if path was reserved
 * @param dest     [out] the final tile of the best path found
 * @return         the best track for next turn
 */
Track YapfTrainChooseTrack(const Train *v, TileIndex tile, DiagDirection enterdir, TrackBits tracks, bool &path_found, bool reserve_track, struct PBSTileInfo *target, TileIndex *dest);

/**
 * Used when user sends road vehicle to the nearest depot or if road vehicle needs servicing using YAPF.
 * @param v            vehicle that needs to go to some depot
 * @param max_penalty  max distance (in pathfinder penalty) from the current vehicle position
 *                     (used also as optimization - the pathfinder can stop path finding if max_penalty
 *                     was reached and no depot was seen)
 * @return             the data about the depot
 */
FindDepotData YapfRoadVehicleFindNearestDepot(const RoadVehicle *v, int max_penalty);

/**
 * Used when user sends train to the nearest depot or if train needs servicing using YAPF.
 * @param v            train that needs to go to some depot
 * @param max_distance max distance (int pathfinder penalty) from the current train position
 *                     (used also as optimization - the pathfinder can stop path finding if max_penalty
 *                     was reached and no depot was seen)
 * @return             the data about the depot
 */
FindDepotData YapfTrainFindNearestDepot(const Train *v, int max_distance);

/**
 * Returns true if it is better to reverse the train before leaving station using YAPF.
 * @param v the train leaving the station
 * @return true if reversing is better
 */
bool YapfTrainCheckReverse(const Train *v);

bool YapfTrainCheckDepotReverse(const Train *v, TileIndex forward_depot, TileIndex reverse_depot);

/**
 * Try to extend the reserved path of a train to the nearest safe tile using YAPF.
 *
 * @param v    The train that needs to find a safe tile.
 * @param tile Last tile of the current reserved path.
 * @param td   Last trackdir of the current reserved path.
 * @param override_railtype Should all physically compatible railtypes be searched, even if the vehicle can't run on them on its own?
 * @return True if the path could be extended to a safe tile.
 */
bool YapfTrainFindNearestSafeTile(const Train *v, TileIndex tile, Trackdir td, bool override_railtype);

/** Origin of a train route plan search: the departure from an arrival state at a stop. */
struct YapfRoutePlanSearchOrigin {
	TileIndex forward_tile;  ///< Start tile when continuing in the direction of arrival.
	Trackdir forward_td;     ///< Start trackdir when continuing in the direction of arrival.
	TileIndex reverse_tile;  ///< Start tile when reversing, or INVALID_TILE if the train may not reverse.
	Trackdir reverse_td;     ///< Start trackdir when reversing.
	bool reverse_penalised;  ///< Whether the train prefers not to reverse (it can't be driven from the other end).
};

/** An arrival state found by a train route plan search. */
struct YapfRoutePlanSearchTarget {
	TileIndex tile; ///< Platform end tile.
	Trackdir td;    ///< Direction of travel.
	int cost;       ///< Cost of the best path to this arrival state.
};

/** Result of a train route plan search. */
struct YapfRoutePlanSearchResult {
	std::vector<YapfRoutePlanSearchTarget> targets; ///< Arrival states found, in order of increasing cost.
	std::vector<uint32_t> keys;                     ///< Node keys of the best paths to the targets (unsorted, may contain duplicates).
	int margin = 0;                                 ///< Cost margin beyond the best target within which other targets were searched.
};

/**
 * Search the paths from a departure state to all reachable arrival states at a station or waypoint, using static costs only.
 * The result only depends on the static game state and the railtypes, owner, length and reversing ability of the train.
 * @param v The train to search the paths for.
 * @param origin The departure state.
 * @param dest The destination station or waypoint.
 * @param[out] result The arrival states and paths found.
 */
void YapfTrainRoutePlanSearch(const Train *v, const YapfRoutePlanSearchOrigin &origin, StationID dest, YapfRoutePlanSearchResult &result);

#endif /* YAPF_H */
