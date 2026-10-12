/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file diagonal_tunnel.cpp Diagonal rail tunnels: cache of tunnels derived from the map, and related helpers. */

#include "stdafx.h"
#include "diagonal_tunnel.h"
#include "company_base.h"
#include "debug.h"
#include "economy_type.h"
#include "core/format.hpp"
#include "3rdparty/robin_hood/robin_hood.h"
#include "3rdparty/cpp-btree/btree_map.h"

#include <tuple>
#include <vector>

#include "safeguards.h"

/**
 * The half tiles along which diagonal tunnels run form strips.
 * Half tiles of the upper/lower tracks form east-west strips, half tiles of the left/right tracks form north-south strips.
 * Within a strip, consecutive half tiles have consecutive positions.
 */
struct HalfTileStripPos {
	bool ns;   ///< Whether this is a north-south strip.
	int strip; ///< Strip ID.
	int pos;   ///< Position within the strip.
};

static HalfTileStripPos GetHalfTileStripPos(TileIndex tile, Track track)
{
	const int x = TileX(tile);
	const int y = TileY(tile);
	switch (track) {
		case TRACK_UPPER: return { false, x + y,     y - x };
		case TRACK_LOWER: return { false, x + y + 1, y - x };
		case TRACK_LEFT:  return { true,  x - y,     x + y };
		case TRACK_RIGHT: return { true,  x - y - 1, x + y };
		default: NOT_REACHED();
	}
}

static uint64_t GetStripKey(bool ns, int strip, uint height)
{
	return (static_cast<uint64_t>(ns ? 1 : 0) << 48) | (static_cast<uint64_t>(static_cast<uint32_t>(strip + (1 << 30))) << 8) | static_cast<uint64_t>(height & 0xFF);
}

static uint64_t GetPortalKey(TileIndex tile, Track track)
{
	return (static_cast<uint64_t>(tile.base()) << 3) | static_cast<uint64_t>(track);
}

/** All diagonal tunnels, keyed by the portal key of end A. */
static robin_hood::unordered_node_map<uint64_t, DiagonalTunnel> _diagonal_tunnels;

/** Lookup of diagonal tunnels by the portal key of either end. */
static robin_hood::unordered_flat_map<uint64_t, DiagonalTunnel *> _diagonal_tunnel_portals;

/** Index of diagonal tunnels by strip and height, used for checking whether tunnels are in the way. */
static btree::btree_multimap<uint64_t, DiagonalTunnel *> _diagonal_tunnel_strip_index;

/**
 * Get the diagonal tunnel of a portal track.
 * @param tile Portal tile.
 * @param track Portal track.
 * @return The tunnel, or nullptr if there is no such tunnel.
 */
const DiagonalTunnel *GetDiagonalTunnel(TileIndex tile, Track track)
{
	auto it = _diagonal_tunnel_portals.find(GetPortalKey(tile, track));
	if (it == _diagonal_tunnel_portals.end()) return nullptr;
	return it->second;
}

/**
 * Get information about the far end of the diagonal tunnel which a trackdir leads into.
 * @param tile Tile.
 * @param td Trackdir.
 * @return Information about the far end, or std::nullopt if td does not lead into a diagonal tunnel.
 */
std::optional<DiagonalTunnelFarEnd> GetDiagonalTunnelFarEnd(TileIndex tile, Trackdir td)
{
	if (!TrackdirEntersDiagonalTunnel(tile, td)) return std::nullopt;

	const DiagonalTunnel *dt = GetDiagonalTunnel(tile, TrackdirToTrack(td));
	if (dt == nullptr) return std::nullopt;

	const TileIndex far_tile = dt->GetOtherEnd(tile);
	const Trackdir far_entry_td = TrackDirectionToTrackdir(dt->GetOtherEndTrack(tile), ReverseDir(TrackdirToDirection(td)));

	DiagonalTunnelFarEnd result;
	result.tile = far_tile;
	result.trackdir = ReverseTrackdir(far_entry_td);
	result.enterdir = ReverseDiagDir(TrackdirToExitdir(far_entry_td));
	result.length = dt->length;
	return result;
}

/**
 * Search along the line of a diagonal tunnel portal track for the matching far portal.
 * @param tile Portal tile.
 * @param track Portal track.
 * @return Tuple of other portal tile, other portal track and tunnel length, or std::nullopt if not found.
 */
static std::optional<std::tuple<TileIndex, Track, uint>> FindDiagonalTunnelOtherEnd(TileIndex tile, Track track)
{
	const Direction dir = GetDiagonalTunnelPortalDirection(tile);
	if (dir == Direction::Invalid) return std::nullopt;
	const Direction end_dir = ReverseDir(dir);
	const int height = GetTileZ(tile);
	const uint limit = Map::SizeX() + Map::SizeY();

	TileIndex t = tile;
	Track tr = track;
	for (uint length = 0; length <= limit; length++) {
		std::tie(t, tr) = GetNextDiagonalTunnelHalfTile(t, tr, dir);
		if (!IsValidTile(t)) break;
		if (IsDiagonalTunnelPortalTile(t) && HasTrack(t, tr) && GetTileZ(t) == height && GetDiagonalTunnelPortalDirection(t) == end_dir) {
			return std::make_tuple(t, tr, length);
		}
	}
	return std::nullopt;
}

/**
 * Add a diagonal tunnel to the cache.
 * Both portals must already be present on the map.
 * @param tile Portal tile of either end.
 * @param track Portal track of either end.
 * @return The added tunnel, or nullptr if the other end could not be found.
 */
const DiagonalTunnel *AddDiagonalTunnel(TileIndex tile, Track track)
{
	auto other = FindDiagonalTunnelOtherEnd(tile, track);
	if (!other.has_value()) return nullptr;

	auto [other_tile, other_track, length] = *other;

	/* Normalise the order of the ends */
	if (GetPortalKey(other_tile, other_track) < GetPortalKey(tile, track)) {
		std::swap(tile, other_tile);
		std::swap(track, other_track);
	}

	const uint64_t key_a = GetPortalKey(tile, track);
	const uint64_t key_b = GetPortalKey(other_tile, other_track);
	assert(_diagonal_tunnel_portals.find(key_a) == _diagonal_tunnel_portals.end());
	assert(_diagonal_tunnel_portals.find(key_b) == _diagonal_tunnel_portals.end());

	DiagonalTunnel &dt = _diagonal_tunnels[key_a];
	dt.tile_a = tile;
	dt.tile_b = other_tile;
	dt.track_a = track;
	dt.track_b = other_track;
	dt.dir = GetDiagonalTunnelPortalDirection(tile);
	dt.height = static_cast<uint8_t>(GetTileZ(tile));
	dt.length = static_cast<uint16_t>(length);

	const HalfTileStripPos pos_a = GetHalfTileStripPos(tile, track);
	const HalfTileStripPos pos_b = GetHalfTileStripPos(other_tile, other_track);
	assert(pos_a.ns == pos_b.ns && pos_a.strip == pos_b.strip);
	dt.ns_strip = pos_a.ns;
	dt.strip = pos_a.strip;
	dt.pos_min = std::min(pos_a.pos, pos_b.pos) + 1;
	dt.pos_max = std::max(pos_a.pos, pos_b.pos) - 1;

	_diagonal_tunnel_portals[key_a] = &dt;
	_diagonal_tunnel_portals[key_b] = &dt;
	_diagonal_tunnel_strip_index.insert({ GetStripKey(dt.ns_strip, dt.strip, dt.height), &dt });

	return &dt;
}

/**
 * Remove a diagonal tunnel from the cache.
 * @param dt Tunnel to remove, this pointer is invalid after this call.
 */
void RemoveDiagonalTunnel(const DiagonalTunnel *dt)
{
	const uint64_t strip_key = GetStripKey(dt->ns_strip, dt->strip, dt->height);
	for (auto it = _diagonal_tunnel_strip_index.lower_bound(strip_key); it != _diagonal_tunnel_strip_index.end() && it->first == strip_key; ++it) {
		if (it->second == dt) {
			_diagonal_tunnel_strip_index.erase(it);
			break;
		}
	}

	const uint64_t key_a = GetPortalKey(dt->tile_a, dt->track_a);
	_diagonal_tunnel_portals.erase(GetPortalKey(dt->tile_b, dt->track_b));
	_diagonal_tunnel_portals.erase(key_a);
	_diagonal_tunnels.erase(key_a);
}

/** Clear the diagonal tunnel cache. */
void ClearDiagonalTunnelCache()
{
	_diagonal_tunnel_strip_index.clear();
	_diagonal_tunnel_portals.clear();
	_diagonal_tunnels.clear();
}

/** Rebuild the diagonal tunnel cache from the map. */
void RebuildDiagonalTunnelCache()
{
	ClearDiagonalTunnelCache();

	for (TileIndex tile(0); tile < Map::Size(); ++tile) {
		if (!IsDiagonalTunnelPortalTile(tile)) continue;

		for (Track track : SetTrackBitIterator(GetTrackBits(tile))) {
			if (GetDiagonalTunnel(tile, track) != nullptr) continue;
			if (AddDiagonalTunnel(tile, track) == nullptr) {
				Debug(misc, 0, "Diagonal tunnel portal at tile 0x{:X}, track {} has no matching far end", tile, track);
			}
		}
	}
}

/**
 * Get the number of diagonal tunnels.
 * @return Number of tunnels.
 */
uint GetDiagonalTunnelCount()
{
	return static_cast<uint>(_diagonal_tunnels.size());
}

/**
 * Get the infrastructure count of the part of a diagonal tunnel underneath the ground (i.e. excluding the portal tracks).
 * @param dt Tunnel.
 * @return Number of track bits for the infrastructure count.
 */
uint GetDiagonalTunnelInfrastructureCount(const DiagonalTunnel *dt)
{
	return dt->length * TUNNELBRIDGE_TRACKBIT_FACTOR;
}

/**
 * Add the infrastructure counts of the underground parts of the diagonal tunnels which are attributed to this portal tile.
 * The underground part of the tunnel is attributed to end A.
 * @param tile Portal tile.
 * @param c Company to add the counts to, may be nullptr.
 * @param multiplier Multiplier, typically 1 or -1.
 */
void AddDiagonalTunnelPortalTileInfrastructureCounts(TileIndex tile, Company *c, int multiplier)
{
	if (c == nullptr) return;
	for (Track track : SetTrackBitIterator(GetTrackBits(tile))) {
		const DiagonalTunnel *dt = GetDiagonalTunnel(tile, track);
		if (dt == nullptr || dt->tile_a != tile || dt->track_a != track) continue;
		c->infrastructure.rail[GetRailTypeByTrack(tile, track)] += multiplier * static_cast<int>(GetDiagonalTunnelInfrastructureCount(dt));
	}
}

/**
 * Get the half tiles which overlap with the given half tile.
 * @param track Half tile track.
 * @return Overlapping half tiles (including itself).
 */
static TrackBits GetOverlappingHalfTiles(Track track)
{
	return (TRACK_BIT_HORZ | TRACK_BIT_VERT) & ~TrackToTrackBits(TrackToOppositeTrack(track));
}

/**
 * Is there a diagonal tunnel underneath the given tile, at the given height, which is in the way of the given half tiles?
 * @param tile Tile.
 * @param z Height level.
 * @param half_tiles Half tiles to check, axial tracks indicate that the whole tile is to be checked.
 * @return true if there is such a tunnel.
 */
bool IsDiagonalTunnelInWay(TileIndex tile, int z, TrackBits half_tiles)
{
	if (_diagonal_tunnel_strip_index.empty()) return false;
	if (half_tiles & TRACK_BIT_CROSS) half_tiles = TRACK_BIT_ALL;

	for (Track track : { TRACK_UPPER, TRACK_LOWER, TRACK_LEFT, TRACK_RIGHT }) {
		if ((half_tiles & GetOverlappingHalfTiles(track)) == 0) continue;

		const HalfTileStripPos pos = GetHalfTileStripPos(tile, track);
		const uint64_t key = GetStripKey(pos.ns, pos.strip, z);
		for (auto it = _diagonal_tunnel_strip_index.lower_bound(key); it != _diagonal_tunnel_strip_index.end() && it->first == key; ++it) {
			const DiagonalTunnel *dt = it->second;
			if (pos.pos >= dt->pos_min && pos.pos <= dt->pos_max) return true;
		}
	}
	return false;
}

/**
 * Would changing the heights of the corners of the given tile damage a diagonal tunnel underneath it?
 * The corners of the half tiles underneath which a diagonal tunnel passes must be higher than the tunnel,
 * except for the corners which are shared with the portal tiles.
 * @param tile Tile.
 * @param z_n New height of the northern corner.
 * @param z_w New height of the western corner.
 * @param z_s New height of the southern corner.
 * @param z_e New height of the eastern corner.
 * @return true if a diagonal tunnel would be damaged.
 */
bool IsDiagonalTunnelDamagedByTerraform(TileIndex tile, int z_n, int z_w, int z_s, int z_e)
{
	if (_diagonal_tunnel_strip_index.empty()) return false;

	auto get_corner_height = [&](TileIndex corner) -> int {
		if (corner == tile) return z_n;
		if (corner == tile + TileDiffXY(1, 0)) return z_w;
		if (corner == tile + TileDiffXY(1, 1)) return z_s;
		return z_e;
	};

	for (Track track : { TRACK_UPPER, TRACK_LOWER, TRACK_LEFT, TRACK_RIGHT }) {
		const HalfTileStripPos pos = GetHalfTileStripPos(tile, track);
		const uint64_t key_begin = GetStripKey(pos.ns, pos.strip, 0);
		const uint64_t key_end = GetStripKey(pos.ns, pos.strip, 0xFF);
		for (auto it = _diagonal_tunnel_strip_index.lower_bound(key_begin); it != _diagonal_tunnel_strip_index.end() && it->first <= key_end; ++it) {
			const DiagonalTunnel *dt = it->second;
			if (pos.pos < dt->pos_min || pos.pos > dt->pos_max) continue;

			for (TileIndex corner : GetHalfTileTriangleCorners(tile, track)) {
				if (get_corner_height(corner) > dt->height) continue;
				if (dt->IsPortalCorner(TileX(corner), TileY(corner))) continue;
				return true;
			}
		}
	}
	return false;
}

/**
 * Get the number of half tiles inside a diagonal tunnel which are ahead of a position inside the tunnel.
 * @param dt The tunnel.
 * @param near_tile The portal tile through which the vehicle entered the tunnel.
 * @param x X position inside the tunnel.
 * @param y Y position inside the tunnel.
 * @param towards_far_end Whether the vehicle is moving towards the far end of the tunnel, or back towards the near end.
 * @return Number of half tiles inside the tunnel strictly between the half tile at the given position and the portal which is being moved towards.
 */
int GetDiagonalTunnelHalfTilesAhead(const DiagonalTunnel *dt, TileIndex near_tile, int x, int y, bool towards_far_end)
{
	const TileIndex virt_tile = TileVirtXY(x, y);
	const int cur_pos = dt->ns_strip ? (TileX(virt_tile) + TileY(virt_tile)) : (TileY(virt_tile) - TileX(virt_tile));
	const TileIndex target_tile = towards_far_end ? dt->GetOtherEnd(near_tile) : near_tile;
	const int target_pos = dt->ns_strip ? (TileX(target_tile) + TileY(target_tile)) : (TileY(target_tile) - TileX(target_tile));
	return std::max(0, std::abs(target_pos - cur_pos) - 1);
}

/**
 * Check the diagonal tunnel cache against the map.
 * @param log Logging function for mismatches.
 */
void ValidateDiagonalTunnelCache(std::function<void(std::string_view)> log)
{
	using Entry = std::tuple<TileIndex, Track, TileIndex, Track, uint, uint, Direction>;
	auto snapshot = []() -> std::vector<Entry> {
		std::vector<Entry> entries;
		entries.reserve(_diagonal_tunnels.size());
		for (const auto &it : _diagonal_tunnels) {
			const DiagonalTunnel &dt = it.second;
			entries.emplace_back(dt.tile_a, dt.track_a, dt.tile_b, dt.track_b, dt.length, dt.height, dt.dir);
		}
		std::sort(entries.begin(), entries.end());
		return entries;
	};

	std::vector<Entry> old_entries = snapshot();
	RebuildDiagonalTunnelCache();
	std::vector<Entry> new_entries = snapshot();

	if (old_entries != new_entries) {
		log(fmt::format("diagonal tunnel cache mismatch: old count: {}, new count: {}", old_entries.size(), new_entries.size()));
		for (const Entry &e : old_entries) {
			if (std::find(new_entries.begin(), new_entries.end(), e) == new_entries.end()) {
				log(fmt::format("  only in old: tile 0x{:X}, track {} -> tile 0x{:X}, track {}, length {}, height {}", std::get<0>(e), std::get<1>(e), std::get<2>(e), std::get<3>(e), std::get<4>(e), std::get<5>(e)));
			}
		}
		for (const Entry &e : new_entries) {
			if (std::find(old_entries.begin(), old_entries.end(), e) == old_entries.end()) {
				log(fmt::format("  only in new: tile 0x{:X}, track {} -> tile 0x{:X}, track {}, length {}, height {}", std::get<0>(e), std::get<1>(e), std::get<2>(e), std::get<3>(e), std::get<4>(e), std::get<5>(e)));
			}
		}
	}
}
