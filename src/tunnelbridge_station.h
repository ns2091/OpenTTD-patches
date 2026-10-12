/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/**
 * @file tunnelbridge_station.h Stations whose platform is the inside of a rail tunnel or bridge.
 *
 * A contiguous part of the middle tiles of a rail tunnel or bridge (excluding the two end tiles) can be made into a platform of a station.
 * Trains stop and load/unload inside the wormhole of such a tunnel or bridge.
 * Each tunnel or bridge can have at most one platform.
 *
 * Both end tiles of the tunnel/bridge have a flag bit set in the map (tunnels: m6 bit 2, bridges: m5 bit 4).
 * The station ID and the platform range are stored in a side table keyed by the northern end tile, which is saved in its own chunk.
 * Each station has a (not saved) list of its tunnel/bridge platforms, which is rebuilt after loading.
 *
 * The middle tiles of a tunnel or bridge are identified by their offset from the northern end tile, the first middle tile has offset 1.
 *
 * Signals cannot be placed on tunnels and bridges with a platform.
 * The pathfinder treats the platform as if there were block signals at its entrance and at its exit.
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

/** A station platform inside a tunnel or bridge. */
struct TunnelBridgeStationPlatform {
	StationID station = StationID::Invalid(); ///< The station.
	uint16_t start = 0;                       ///< Offset of the first platform tile from the northern end tile of the tunnel/bridge.
	uint16_t length = 0;                      ///< Number of platform tiles.

	/**
	 * Get the offset of the last platform tile from the northern end tile of the tunnel/bridge.
	 * @return The offset of the last platform tile.
	 */
	inline uint Last() const { return this->start + this->length - 1; }

	/**
	 * Is the middle tile with the given offset from the northern end tile part of the platform?
	 * @param offset The offset from the northern end tile.
	 * @return true if the tile is part of the platform.
	 */
	inline bool ContainsOffset(uint offset) const { return offset >= this->start && offset <= this->Last(); }
};

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
 * Get the offset of a tile from the northern end tile of a tunnel or bridge.
 * @param t One of the end tiles of the tunnel or bridge.
 * @param tile The tile, which must be on the line of the tunnel or bridge.
 * @return The offset, the first middle tile has offset 1.
 */
inline uint GetTunnelBridgeTileOffset(TileIndex t, TileIndex tile)
{
	return DistanceManhattan(GetTunnelBridgeStationKey(t), tile);
}

const TunnelBridgeStationPlatform &GetTunnelBridgeStationPlatform(TileIndex t);
TileIndex GetTunnelBridgeTileAtOffset(TileIndex t, uint offset);
TileArea GetTunnelBridgeStationArea(TileIndex t);
uint GetTunnelBridgeStationPlatformLength(TileIndex t);
uint GetTunnelBridgeStationPlatformDistanceFromEnd(TileIndex end);
bool IsTunnelBridgeStationPlatformTile(TileIndex t, TileIndex tile);
bool IsTunnelBridgeStationSectionOccupied(TileIndex t, uint first, uint last, const Train *ignore);
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
bool IsTunnelBridgeStationPlatformOccupied(TileIndex t, const Train *ignore);
bool IsTunnelBridgeStationExitSectionOccupied(TileIndex exit, const Train *ignore);
StationID GetTrainVehicleStationID(const Train *v);
void GetTunnelBridgeStationPlatformPosition(const Train *v, int *station_ahead, int *station_length);
bool IsTunnelBridgeStationDestination(TileIndex tile, Trackdir td, StationID station);
StationID GetTunnelBridgeStationOfReservationEnd(TileIndex tile, Trackdir td);

void AddTunnelBridgeStation(TileIndex t, Station *st, uint start, uint length);
void SetTunnelBridgeStationPlatformRange(TileIndex t, uint start, uint length);
void RemoveTunnelBridgeStation(TileIndex t);
void ClearTunnelBridgeStations();
void InsertTunnelBridgeStationOnLoad(TileIndex key, const TunnelBridgeStationPlatform &platform);
void RebuildTunnelBridgeStationCaches();
size_t GetTunnelBridgeStationCount();
void IterateTunnelBridgeStations(std::function<void(TileIndex, const TunnelBridgeStationPlatform &)> handler);
bool StationHasTunnelBridgePlatformInArea(const Station *st, int left, int top, int right, int bottom);
TileArea GetRailStationAreaIncludingTunnelBridgePlatforms(const struct BaseStation *st, TileArea area);
void ValidateTunnelBridgeStationCaches(std::function<void(std::string_view)> log);

void DoRemoveTunnelBridgeStation(TileIndex tile);

#endif /* TUNNELBRIDGE_STATION_H */
