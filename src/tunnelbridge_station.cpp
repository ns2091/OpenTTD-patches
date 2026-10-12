/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file tunnelbridge_station.cpp Stations whose platform is the inside of a rail tunnel or bridge. */

#include "stdafx.h"
#include "tunnelbridge_station.h"
#include "station_base.h"
#include "train.h"
#include "command_type.h"
#include "core/format.hpp"
#include "3rdparty/cpp-btree/btree_map.h"

#include <algorithm>

#include "safeguards.h"

/** Station of each tunnel/bridge station platform, keyed by the northern end tile of the tunnel/bridge. */
static btree::btree_map<TileIndex, StationID> _tunnel_bridge_stations;

/**
 * Get the station of a tunnel or bridge station platform.
 * @param t One of the end tiles of the tunnel or bridge.
 * @pre IsTunnelBridgeStationTile(t)
 * @return The station.
 */
StationID GetTunnelBridgeStationID(TileIndex t)
{
	auto it = _tunnel_bridge_stations.find(GetTunnelBridgeStationKey(t));
	assert_tile(it != _tunnel_bridge_stations.end(), t);
	return it->second;
}

/**
 * Is the train vehicle on the platform of a tunnel or bridge station, i.e. inside the wormhole of a tunnel or bridge station?
 * @param v The train vehicle.
 * @return true if the vehicle is on a tunnel or bridge station platform.
 */
bool IsTrainOnTunnelBridgeStationPlatform(const Train *v)
{
	return v->track == TRACK_BIT_WORMHOLE && IsTunnelBridgeStationTile(v->tile);
}

/**
 * Get the station of the platform that a train vehicle is on.
 * This is either a station tile (rail station or waypoint), or the inside of a tunnel or bridge station.
 * @param v The train vehicle.
 * @return The station, or StationID::Invalid() if the vehicle is not on a platform.
 */
StationID GetTrainVehicleStationID(const Train *v)
{
	if (IsTileType(v->tile, TileType::Station)) return GetStationIndex(v->tile);
	if (IsTrainOnTunnelBridgeStationPlatform(v)) return GetTunnelBridgeStationID(v->tile);
	return StationID::Invalid();
}

/**
 * Get the position of a train vehicle on a tunnel or bridge station platform.
 * @param v The train vehicle.
 * @param[out] station_ahead The length of the platform ahead of the vehicle, including the tile the vehicle is on, in 1/16th tiles.
 * @param[out] station_length The length of the platform in 1/16th tiles.
 * @pre IsTrainOnTunnelBridgeStationPlatform(v)
 */
void GetTunnelBridgeStationPlatformPosition(const Train *v, int *station_ahead, int *station_length)
{
	const TileIndex other_end = GetOtherTunnelBridgeEnd(v->tile);
	const DiagDirection tb_dir = GetTunnelBridgeDirection(v->tile);
	const DiagDirection dir = DirToDiagDirAlongAxis(v->GetMovingDirection(), DiagDirToAxis(tb_dir));
	const TileIndex ahead_end = (dir == tb_dir) ? other_end : v->tile;
	*station_ahead = DistanceManhattan(TileVirtXY(v->x_pos, v->y_pos), ahead_end) * TILE_SIZE;
	*station_length = GetTunnelBridgeLength(v->tile, other_end) * TILE_SIZE;
}

/**
 * Get the station of a tunnel or bridge station platform which has just been passed through when arriving at the given tile/trackdir.
 * @param tile The tile.
 * @param td The trackdir on the tile.
 * @return The station of the platform, or StationID::Invalid() if the tile/trackdir is not the exit of a tunnel or bridge station.
 */
StationID GetTunnelBridgeStationOfReservationEnd(TileIndex tile, Trackdir td)
{
	if (!IsTunnelBridgeStationTile(tile) || !TrackdirExitsTunnelBridge(tile, td)) return StationID::Invalid();
	return GetTunnelBridgeStationID(tile);
}

/**
 * Is the given tile/trackdir the exit of a tunnel or bridge station platform of the given station?
 * Pathfinders treat the exit of a tunnel/bridge platform as reaching the platform, in the same way as the end tile of a station platform.
 * @param tile The tile.
 * @param td The trackdir on the tile.
 * @param station The station.
 * @return true if the tile/trackdir is the exit of a tunnel or bridge station platform of the station.
 */
bool IsTunnelBridgeStationDestination(TileIndex tile, Trackdir td, StationID station)
{
	return GetTunnelBridgeStationOfReservationEnd(tile, td) == station;
}

/**
 * Make a rail tunnel or bridge into a platform of the given station.
 * This does not update the station rect, catchment, or facilities.
 * @param t One of the end tiles of the tunnel or bridge.
 * @param st The station.
 */
void AddTunnelBridgeStation(TileIndex t, Station *st)
{
	const TileIndex other_end = GetOtherTunnelBridgeEnd(t);
	const TileIndex key = std::min(t, other_end);
	SetTunnelBridgeStationFlag(t, true);
	SetTunnelBridgeStationFlag(other_end, true);
	_tunnel_bridge_stations[key] = st->index;
	st->tunnel_bridge_platforms.push_back(key);
}

/**
 * Remove the station platform from a rail tunnel or bridge.
 * This does not update the station rect, catchment, or facilities.
 * @param t One of the end tiles of the tunnel or bridge.
 */
void RemoveTunnelBridgeStation(TileIndex t)
{
	const TileIndex other_end = GetOtherTunnelBridgeEnd(t);
	const TileIndex key = std::min(t, other_end);
	auto it = _tunnel_bridge_stations.find(key);
	assert_tile(it != _tunnel_bridge_stations.end(), t);
	Station *st = Station::GetIfValid(it->second);
	if (st != nullptr) {
		auto &platforms = st->tunnel_bridge_platforms;
		platforms.erase(std::remove(platforms.begin(), platforms.end(), key), platforms.end());
	}
	_tunnel_bridge_stations.erase(it);
	SetTunnelBridgeStationFlag(t, false);
	SetTunnelBridgeStationFlag(other_end, false);
}

/** Clear all tunnel/bridge station data, used when starting a new game or loading a game. */
void ClearTunnelBridgeStations()
{
	_tunnel_bridge_stations.clear();
}

/**
 * Insert a tunnel/bridge station platform when loading a game.
 * @param key The northern end tile of the tunnel/bridge.
 * @param st The station.
 */
void InsertTunnelBridgeStationOnLoad(TileIndex key, StationID st)
{
	_tunnel_bridge_stations[key] = st;
}

/** Rebuild the per-station lists of tunnel/bridge platforms and include the platforms in the station rects, after loading a game. */
void RebuildTunnelBridgeStationCaches()
{
	for (Station *st : Station::Iterate()) {
		st->tunnel_bridge_platforms.clear();
	}
	for (auto it = _tunnel_bridge_stations.begin(); it != _tunnel_bridge_stations.end();) {
		Station *st = Station::GetIfValid(it->second);
		if (st == nullptr || !IsTunnelBridgeStationTile(it->first)) {
			/* Inconsistent entry, drop it. */
			if (IsTileType(it->first, TileType::TunnelBridge) && GetTunnelBridgeTransportType(it->first) == TransportType::Rail) {
				SetTunnelBridgeStationFlag(it->first, false);
				SetTunnelBridgeStationFlag(GetOtherTunnelBridgeEnd(it->first), false);
			}
			it = _tunnel_bridge_stations.erase(it);
			continue;
		}
		st->tunnel_bridge_platforms.push_back(it->first);

		/* The station rect is not saved, and is rebuilt from the station tiles when loading, so include the platform here. */
		const TileArea area = GetTunnelBridgeStationArea(it->first);
		st->rect.BeforeAddRect(area.tile, area.w, area.h, StationRect::ADD_FORCE);
		++it;
	}
}

/**
 * Get the number of tunnel/bridge station platforms.
 * @return The number of tunnel/bridge station platforms.
 */
size_t GetTunnelBridgeStationCount()
{
	return _tunnel_bridge_stations.size();
}

/**
 * Call the handler for each tunnel/bridge station platform, in order of the northern end tile.
 * @param handler The handler, which is passed the northern end tile and the station.
 */
void IterateTunnelBridgeStations(std::function<void(TileIndex, StationID)> handler)
{
	for (const auto &it : _tunnel_bridge_stations) {
		handler(it.first, it.second);
	}
}

/**
 * Check whether a station has a tunnel/bridge platform which overlaps the given rectangle.
 * @param st The station.
 * @param left Minimal tile X edge of the rectangle.
 * @param top Minimal tile Y edge of the rectangle.
 * @param right Maximal tile X edge of the rectangle (inclusive).
 * @param bottom Maximal tile Y edge of the rectangle (inclusive).
 * @return true if a tunnel/bridge platform (including its end tiles) overlaps the rectangle.
 */
bool StationHasTunnelBridgePlatformInArea(const Station *st, int left, int top, int right, int bottom)
{
	for (TileIndex key : st->tunnel_bridge_platforms) {
		const TileArea area = GetTunnelBridgeStationArea(key);
		const int x = TileX(area.tile);
		const int y = TileY(area.tile);
		if (x + (int)area.w - 1 < left || x > right) continue;
		if (y + (int)area.h - 1 < top || y > bottom) continue;
		return true;
	}
	return false;
}

/**
 * Get the area covered by the rail station tiles and the tunnel/bridge platforms of a station.
 * @param st The station.
 * @param area The area of the rail station tiles (may be empty).
 * @return The bounding box of the rail station tiles and the tunnel/bridge platforms (including their end tiles).
 */
TileArea GetRailStationAreaIncludingTunnelBridgePlatforms(const BaseStation *st, TileArea area)
{
	if (!Station::IsExpected(st)) return area;
	for (TileIndex key : Station::From(st)->tunnel_bridge_platforms) {
		const TileArea tb_area = GetTunnelBridgeStationArea(key);
		if (area.tile == INVALID_TILE) {
			area = tb_area;
		} else {
			area.Add(tb_area.tile);
			area.Add(TileAddXY(tb_area.tile, tb_area.w - 1, tb_area.h - 1));
		}
	}
	return area;
}

/**
 * Check the tunnel/bridge station caches for consistency.
 * @param log The function to call for each inconsistency found.
 */
void ValidateTunnelBridgeStationCaches(std::function<void(std::string_view)> log)
{
	for (const auto &it : _tunnel_bridge_stations) {
		const TileIndex key = it.first;
		if (!IsTunnelBridgeStationTile(key)) {
			log(fmt::format("Tunnel/bridge station: entry for tile 0x{:X} which is not a tunnel/bridge station tile", key));
			continue;
		}
		const TileIndex other_end = GetOtherTunnelBridgeEnd(key);
		if (other_end < key) log(fmt::format("Tunnel/bridge station: entry for tile 0x{:X} is not keyed by the northern end", key));
		if (!IsTunnelBridgeStationTile(other_end)) log(fmt::format("Tunnel/bridge station: other end 0x{:X} of tile 0x{:X} is not flagged", other_end, key));
		const Station *st = Station::GetIfValid(it.second);
		if (st == nullptr) {
			log(fmt::format("Tunnel/bridge station: entry for tile 0x{:X} has invalid station {}", key, it.second));
			continue;
		}
		if (std::count(st->tunnel_bridge_platforms.begin(), st->tunnel_bridge_platforms.end(), key) != 1) {
			log(fmt::format("Tunnel/bridge station: tile 0x{:X} not in platform list of station {}", key, it.second));
		}
		if (!st->facilities.Test(StationFacility::Train)) {
			log(fmt::format("Tunnel/bridge station: station {} of tile 0x{:X} has no train facility", it.second, key));
		}
		const TileArea area = GetTunnelBridgeStationArea(key);
		if (!st->rect.PtInExtendedRect(TileX(area.tile), TileY(area.tile)) || !st->rect.PtInExtendedRect(TileX(area.tile) + area.w - 1, TileY(area.tile) + area.h - 1)) {
			log(fmt::format("Tunnel/bridge station: tile 0x{:X} is outside the rect of station {}", key, it.second));
		}
	}
	for (const Station *st : Station::Iterate()) {
		for (TileIndex key : st->tunnel_bridge_platforms) {
			auto it = _tunnel_bridge_stations.find(key);
			if (it == _tunnel_bridge_stations.end() || it->second != st->index) {
				log(fmt::format("Tunnel/bridge station: station {} has tile 0x{:X} in its platform list, which does not belong to it", st->index, key));
			}
		}
	}
	for (TileIndex t(0); t < Map::Size(); t++) {
		if (IsTunnelBridgeStationTile(t) && _tunnel_bridge_stations.find(GetTunnelBridgeStationKey(t)) == _tunnel_bridge_stations.end()) {
			log(fmt::format("Tunnel/bridge station: flagged tile 0x{:X} has no entry", t));
		}
	}
}
