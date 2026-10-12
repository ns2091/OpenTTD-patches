/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/**
 * @file tunnelbridge_station.h Stations whose platform is the inside of a rail tunnel or bridge.
 *
 * The whole middle part of a rail tunnel or bridge (excluding the two end tiles) can be made into a platform of a station.
 * Trains stop and load/unload inside the wormhole of such a tunnel or bridge.
 *
 * Both end tiles of the tunnel/bridge have a flag bit set in the map (tunnels: m6 bit 2, bridges: m5 bit 4).
 * The station ID is stored in a side table keyed by the northern end tile, which is saved in its own chunk.
 * Each station has a (not saved) list of its tunnel/bridge platforms, which is rebuilt after loading.
 */

#ifndef TUNNELBRIDGE_STATION_H
#define TUNNELBRIDGE_STATION_H

#include "tunnelbridge_map.h"
#include "tunnelbridge.h"
#include "station_type.h"
#include "tilearea_type.h"
#include "track_func.h"

#include <functional>
#include <string_view>
#include <vector>

struct Train;
struct Station;

/**
 * Is this end tile of a tunnel or bridge part of a rail tunnel/bridge whose middle part is a station platform?
 * @param t The tunnel or bridge end tile.
 * @pre IsTileType(t, TileType::TunnelBridge)
 * @return true if the tunnel or bridge is a station platform.
 */
inline bool IsTunnelBridgeStation(TileIndex t)
{
	dbg_assert_tile(IsTileType(t, TileType::TunnelBridge), t);
	if (GetTunnelBridgeTransportType(t) != TransportType::Rail) return false;
	return IsTunnel(t) ? HasBit(_me[t].m6, 2) : HasBit(_m[t].m5, 4);
}

/**
 * Is this tile an end tile of a rail tunnel/bridge whose middle part is a station platform?
 * @param t The tile.
 * @return true if the tile is an end tile of a tunnel or bridge station platform.
 */
inline bool IsTunnelBridgeStationTile(TileIndex t)
{
	return IsTileType(t, TileType::TunnelBridge) && IsTunnelBridgeStation(t);
}

/**
 * Set whether this rail tunnel/bridge end tile is part of a tunnel or bridge station platform.
 * @param t The tunnel or bridge end tile.
 * @param station Whether the tunnel/bridge is a station platform.
 * @pre IsTileType(t, TileType::TunnelBridge) && GetTunnelBridgeTransportType(t) == TransportType::Rail
 */
inline void SetTunnelBridgeStationFlag(TileIndex t, bool station)
{
	dbg_assert_tile(IsTileType(t, TileType::TunnelBridge) && GetTunnelBridgeTransportType(t) == TransportType::Rail, t);
	if (IsTunnel(t)) {
		AssignBit(_me[t].m6, 2, station);
	} else {
		AssignBit(_m[t].m5, 4, station);
	}
}

/**
 * Get the northern end tile of a tunnel or bridge, this is used as the key of a tunnel/bridge station platform.
 * @param t One of the end tiles of the tunnel or bridge.
 * @return The end tile with the lowest tile index.
 */
inline TileIndex GetTunnelBridgeStationKey(TileIndex t)
{
	return std::min(t, GetOtherTunnelBridgeEnd(t));
}

/**
 * Get the area covered by a tunnel or bridge, including the end tiles.
 * @param t One of the end tiles of the tunnel or bridge.
 * @return The tile area.
 */
inline TileArea GetTunnelBridgeStationArea(TileIndex t)
{
	return TileArea(t, GetOtherTunnelBridgeEnd(t));
}

/**
 * Get the platform length of a tunnel or bridge station platform, this is the length of the middle part of the tunnel/bridge.
 * @param t One of the end tiles of the tunnel or bridge.
 * @return The platform length in tiles.
 */
inline uint GetTunnelBridgeStationPlatformLength(TileIndex t)
{
	return GetTunnelBridgeLength(t, GetOtherTunnelBridgeEnd(t));
}

StationID GetTunnelBridgeStationID(TileIndex t);

/**
 * Get the station of a tunnel or bridge end tile, if any.
 * @param t The tile.
 * @return The station of the tunnel/bridge station platform, or StationID::Invalid().
 */
inline StationID GetTunnelBridgeStationIDIfAny(TileIndex t)
{
	return IsTunnelBridgeStationTile(t) ? GetTunnelBridgeStationID(t) : StationID::Invalid();
}

bool IsTrainOnTunnelBridgeStationPlatform(const Train *v);
StationID GetTrainVehicleStationID(const Train *v);
void GetTunnelBridgeStationPlatformPosition(const Train *v, int *station_ahead, int *station_length);
bool IsTunnelBridgeStationDestination(TileIndex tile, Trackdir td, StationID station);
StationID GetTunnelBridgeStationOfReservationEnd(TileIndex tile, Trackdir td);

void AddTunnelBridgeStation(TileIndex t, Station *st);
void RemoveTunnelBridgeStation(TileIndex t);
void ClearTunnelBridgeStations();
void InsertTunnelBridgeStationOnLoad(TileIndex key, StationID st);
void RebuildTunnelBridgeStationCaches();
size_t GetTunnelBridgeStationCount();
void IterateTunnelBridgeStations(std::function<void(TileIndex, StationID)> handler);
bool StationHasTunnelBridgePlatformInArea(const Station *st, int left, int top, int right, int bottom);
TileArea GetRailStationAreaIncludingTunnelBridgePlatforms(const struct BaseStation *st, TileArea area);
void ValidateTunnelBridgeStationCaches(std::function<void(std::string_view)> log);

void DoRemoveTunnelBridgeStation(TileIndex tile);

#endif /* TUNNELBRIDGE_STATION_H */
