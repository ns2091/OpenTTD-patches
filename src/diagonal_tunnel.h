/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/**
 * @file diagonal_tunnel.h Diagonal rail tunnels.
 *
 * A diagonal tunnel is a rail tunnel which runs along the half-tile (upper/lower/left/right) track pieces,
 * i.e. diagonally with respect to the tile grid (towards one of the N, E, S or W directions).
 *
 * The portals of a diagonal tunnel are plain rail tiles with the diagonal tunnel portal flag set.
 * A portal tile has a slope with exactly one corner raised, the tunnel(s) lead into the hill towards the raised corner.
 * The (flat) track pieces adjacent to the raised corner are the portal tracks, each of which is the portal of a separate tunnel.
 * Therefore a single portal tile can contain the portals of two parallel tunnels.
 *
 * The tunnel itself passes underneath the half tiles along the line of the portal track, at the height of the portal tile.
 *
 * Trains inside a diagonal tunnel are hidden, have the GroundVehicleFlag::DiagonalTunnel flag set,
 * and retain the tile and track of the portal through which they entered the tunnel.
 * For the purposes of track occupancy, signalling and reservation they are therefore treated as being on that portal track.
 */

#ifndef DIAGONAL_TUNNEL_H
#define DIAGONAL_TUNNEL_H

#include "rail_map.h"
#include "slope_func.h"
#include "direction_func.h"
#include "track_func.h"
#include "map_func.h"

#include <array>
#include <functional>
#include <optional>

/** A diagonal tunnel, this is cached information which is derived from the map. */
struct DiagonalTunnel {
	TileIndex tile_a;   ///< Portal tile at the start of the tunnel.
	TileIndex tile_b;   ///< Portal tile at the end of the tunnel.
	Track track_a;      ///< Portal track at tile_a.
	Track track_b;      ///< Portal track at tile_b.
	Direction dir;      ///< Direction from tile_a into the tunnel, towards tile_b.
	uint8_t height;     ///< Height level of the tunnel (and of the lowest corners of the portal tiles).
	uint16_t length;    ///< Number of half tiles underneath which the tunnel passes (excluding the portal tiles).

	bool ns_strip;      ///< Whether this tunnel is in a north-south strip (left/right tracks), or an east-west strip (upper/lower tracks).
	int strip;          ///< Strip ID of this tunnel.
	int pos_min;        ///< Minimum strip position of a half tile underneath which the tunnel passes (excluding the portal tiles).
	int pos_max;        ///< Maximum strip position of a half tile underneath which the tunnel passes (excluding the portal tiles).

	/**
	 * Is the given portal tile and track one of the ends of this tunnel?
	 * @param tile Portal tile.
	 * @param track Portal track.
	 * @return true if this is an end of the tunnel.
	 */
	inline bool IsEnd(TileIndex tile, Track track) const
	{
		return (tile == this->tile_a && track == this->track_a) || (tile == this->tile_b && track == this->track_b);
	}

	/**
	 * Get the other end of the tunnel.
	 * @param tile Portal tile of one end.
	 * @return Portal tile of the other end.
	 */
	inline TileIndex GetOtherEnd(TileIndex tile) const
	{
		return tile == this->tile_a ? this->tile_b : this->tile_a;
	}

	/**
	 * Get the portal track of the other end of the tunnel.
	 * @param tile Portal tile of one end.
	 * @return Portal track of the other end.
	 */
	inline Track GetOtherEndTrack(TileIndex tile) const
	{
		return tile == this->tile_a ? this->track_b : this->track_a;
	}

	/**
	 * Is the given corner a corner of one of the portal tiles?
	 * @param corner_x X coordinate of the corner.
	 * @param corner_y Y coordinate of the corner.
	 * @return true if the corner is a corner of one of the portal tiles.
	 */
	inline bool IsPortalCorner(uint corner_x, uint corner_y) const
	{
		auto test = [&](TileIndex t) -> bool {
			return (corner_x == TileX(t) || corner_x == TileX(t) + 1) && (corner_y == TileY(t) || corner_y == TileY(t) + 1);
		};
		return test(this->tile_a) || test(this->tile_b);
	}
};

/** Information about the far end of a diagonal tunnel, as seen from a train entering at the near end. */
struct DiagonalTunnelFarEnd {
	TileIndex tile;         ///< Far portal tile.
	Trackdir trackdir;      ///< Trackdir on the far portal tile, leading out of the tunnel.
	DiagDirection enterdir; ///< Direction of movement when entering the far portal tile from the tunnel.
	uint length;            ///< Number of half tiles underneath which the tunnel passes (excluding the portal tiles).
};

/**
 * Get the direction leading into the tunnels of a diagonal tunnel portal tile with the given slope.
 * @param tileh Tile slope.
 * @return Direction into the tunnel, or Direction::Invalid if the slope is not suitable for a diagonal tunnel portal.
 */
inline Direction GetDiagonalTunnelDirectionFromSlope(Slope tileh)
{
	switch (tileh) {
		case SLOPE_N: return Direction::N;
		case SLOPE_E: return Direction::E;
		case SLOPE_S: return Direction::S;
		case SLOPE_W: return Direction::W;
		default: return Direction::Invalid;
	}
}

/**
 * Get the slope of a diagonal tunnel portal tile which leads into a diagonal tunnel in the given direction.
 * @param dir Direction into the tunnel, must be one of N, E, S, W.
 * @return Slope with the corner in direction dir raised.
 */
inline Slope GetDiagonalTunnelSlopeFromDirection(Direction dir)
{
	switch (dir) {
		case Direction::N: return SLOPE_N;
		case Direction::E: return SLOPE_E;
		case Direction::S: return SLOPE_S;
		case Direction::W: return SLOPE_W;
		default: NOT_REACHED();
	}
}

/**
 * Get the tracks which may be diagonal tunnel portal tracks for tunnels in the given direction.
 * @param dir Direction into the tunnel, must be one of N, E, S, W.
 * @return Track bits.
 */
inline TrackBits GetDiagonalTunnelPortalTrackBits(Direction dir)
{
	return (dir == Direction::E || dir == Direction::W) ? TRACK_BIT_HORZ : TRACK_BIT_VERT;
}

/**
 * Get the direction leading into the tunnels of a diagonal tunnel portal tile.
 * @param tile Portal tile.
 * @return Direction into the tunnel(s).
 * @pre IsDiagonalTunnelPortalTile(tile)
 */
inline Direction GetDiagonalTunnelPortalDirection(TileIndex tile)
{
	dbg_assert_tile(IsDiagonalTunnelPortalTile(tile), tile);
	return GetDiagonalTunnelDirectionFromSlope(GetTileSlope(tile));
}

/**
 * Get the trackdir on a diagonal tunnel portal tile leading into the tunnel of the given track.
 * @param tile Portal tile.
 * @param track Portal track.
 * @return Trackdir into the tunnel.
 * @pre IsDiagonalTunnelPortalTile(tile)
 */
inline Trackdir GetDiagonalTunnelEntryTrackdir(TileIndex tile, Track track)
{
	return TrackDirectionToTrackdir(track, GetDiagonalTunnelPortalDirection(tile));
}

/**
 * Does the given trackdir lead into a diagonal tunnel?
 * @param tile Tile.
 * @param td Trackdir.
 * @return true if the tile is a diagonal tunnel portal and td is one of its portal tracks, leading into the tunnel.
 */
inline bool TrackdirEntersDiagonalTunnel(TileIndex tile, Trackdir td)
{
	if (!IsDiagonalTunnelPortalTile(tile)) return false;
	if (IsDiagonalTrackdir(td)) return false;
	return HasTrack(tile, TrackdirToTrack(td)) && TrackdirToDirection(td) == GetDiagonalTunnelPortalDirection(tile);
}

/**
 * Does the given trackdir lead out of a diagonal tunnel?
 * @param tile Tile.
 * @param td Trackdir.
 * @return true if the tile is a diagonal tunnel portal and td is one of its portal tracks, leading out of the tunnel.
 */
inline bool TrackdirExitsDiagonalTunnel(TileIndex tile, Trackdir td)
{
	return TrackdirEntersDiagonalTunnel(tile, ReverseTrackdir(td));
}

/**
 * Get the portal tracks of a diagonal tunnel portal tile whose tunnel is on the given side of the tile.
 * The tunnel side of a portal track can't be entered from the adjacent tile.
 * @param tile Portal tile.
 * @param side Side of the tile.
 * @return Portal tracks (which may or may not be present on the tile) leading into a tunnel on the given side.
 * @pre IsDiagonalTunnelPortalTile(tile)
 */
inline TrackBits GetDiagonalTunnelPortalTrackBitsOnTunnelSide(TileIndex tile, DiagDirection side)
{
	const Direction dir = GetDiagonalTunnelPortalDirection(tile);
	TrackBits result = TRACK_BIT_NONE;
	for (Track tr : SetTrackBitIterator(GetDiagonalTunnelPortalTrackBits(dir))) {
		if (TrackdirToExitdir(TrackDirectionToTrackdir(tr, dir)) == side) result |= TrackToTrackBits(tr);
	}
	return result;
}

/**
 * Get the next half tile along the line of a diagonal tunnel.
 * @param tile The current tile.
 * @param track The current track (half tile), must be one of upper/lower/left/right.
 * @param dir The direction of travel, must be one of N, E, S, W.
 * @return The next tile and track, the tile may be outside of the map.
 */
inline std::pair<TileIndex, Track> GetNextDiagonalTunnelHalfTile(TileIndex tile, Track track, Direction dir)
{
	DiagDirection exitdir = TrackdirToExitdir(TrackDirectionToTrackdir(track, dir));
	return { TileAddByDiagDir(tile, exitdir), TrackToOppositeTrack(track) };
}

/**
 * Get the corners of the triangle of a half tile.
 * The corners are returned as tile indexes of the tiles whose northern corner is the given corner.
 * @param tile Tile.
 * @param track Track identifying the half tile, must be one of upper/lower/left/right.
 * @return Corners of the triangle.
 */
inline std::array<TileIndex, 3> GetHalfTileTriangleCorners(TileIndex tile, Track track)
{
	const TileIndex n = tile;
	const TileIndex w = tile + TileDiffXY(1, 0);
	const TileIndex s = tile + TileDiffXY(1, 1);
	const TileIndex e = tile + TileDiffXY(0, 1);
	switch (track) {
		case TRACK_UPPER: return { w, n, e };
		case TRACK_LOWER: return { w, s, e };
		case TRACK_LEFT:  return { n, w, s };
		case TRACK_RIGHT: return { n, e, s };
		default: NOT_REACHED();
	}
}

const DiagonalTunnel *GetDiagonalTunnel(TileIndex tile, Track track);
std::optional<DiagonalTunnelFarEnd> GetDiagonalTunnelFarEnd(TileIndex tile, Trackdir td);

/**
 * Get the other portal tile of a diagonal tunnel.
 * @param tile Portal tile.
 * @param track Portal track.
 * @return Other portal tile, or INVALID_TILE if there is no such tunnel.
 */
inline TileIndex GetOtherDiagonalTunnelEnd(TileIndex tile, Track track)
{
	const DiagonalTunnel *dt = GetDiagonalTunnel(tile, track);
	return dt != nullptr ? dt->GetOtherEnd(tile) : INVALID_TILE;
}

int GetDiagonalTunnelHalfTilesAhead(const DiagonalTunnel *dt, TileIndex near_tile, int x, int y, bool towards_far_end);
bool IsDiagonalTunnelInWay(TileIndex tile, int z, TrackBits half_tiles);
bool IsDiagonalTunnelDamagedByTerraform(TileIndex tile, int z_n, int z_w, int z_s, int z_e);

void RebuildDiagonalTunnelCache();
void ClearDiagonalTunnelCache();
const DiagonalTunnel *AddDiagonalTunnel(TileIndex tile, Track track);
void RemoveDiagonalTunnel(const DiagonalTunnel *dt);
uint GetDiagonalTunnelCount();
uint GetDiagonalTunnelInfrastructureCount(const DiagonalTunnel *dt);
void AddDiagonalTunnelPortalTileInfrastructureCounts(TileIndex tile, struct Company *c, int multiplier);

void ValidateDiagonalTunnelCache(std::function<void(std::string_view)> log);

#endif /* DIAGONAL_TUNNEL_H */
