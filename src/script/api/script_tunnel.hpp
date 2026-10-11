/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file script_tunnel.hpp Everything to query and build tunnels. */

#ifndef SCRIPT_TUNNEL_HPP
#define SCRIPT_TUNNEL_HPP

#include "script_vehicle.hpp"
#include "script_rail.hpp"

/**
 * Class that handles all tunnel related functions.
 * @api ai game
 */
class ScriptTunnel : public ScriptObject {
public:
	/**
	 * All tunnel related errors.
	 *
	 * @see ScriptErrorType
	 */
	enum ErrorMessages {

		/** Base for bridge related errors */
		ERR_TUNNEL_BASE = ScriptError::ERR_CAT_TUNNEL << ScriptError::ERR_CAT_BIT_SIZE,

		/** Can't build tunnels on water */
		ERR_TUNNEL_CANNOT_BUILD_ON_WATER,            // [STR_ERROR_CAN_T_BUILD_ON_WATER]

		/** The start tile must slope either North, South, West or East */
		ERR_TUNNEL_START_SITE_UNSUITABLE,            // [STR_ERROR_SITE_UNSUITABLE_FOR_TUNNEL]

		/** Another tunnel is in the way */
		ERR_TUNNEL_ANOTHER_TUNNEL_IN_THE_WAY,        // [STR_ERROR_ANOTHER_TUNNEL_IN_THE_WAY]

		/** Unable to excavate land at the end to create the tunnel's exit */
		ERR_TUNNEL_END_SITE_UNSUITABLE,              // [STR_ERROR_UNABLE_TO_EXCAVATE_LAND]
	};

	/**
	 * Check whether the tile is an entrance to a tunnel.
	 * @param tile The tile to check.
	 * @pre ScriptMap::IsValidTile(tile).
	 * @return True if and only if the tile is the beginning or end of a tunnel.
	 */
	static bool IsTunnelTile(TileIndex tile);

	/**
	 * Get the tile that exits on the other end of a (would be) tunnel starting
	 *  at tile. If there is no 'simple' inclined slope at the start tile,
	 *  this function will return ScriptMap::TILE_INVALID.
	 * @param tile The tile that is an entrance to a tunnel or the tile where you may want to build a tunnel.
	 * @pre ScriptMap::IsValidTile(tile).
	 * @return The TileIndex that is the other end of the (would be) tunnel, or
	 *  ScriptMap::TILE_INVALID if no other end was found (can't build tunnel).
	 * @note Even if this function returns a valid tile, that is no guarantee
	 *  that building a tunnel will succeed. Use BuildTunnel in ScriptTestMode to
	 *  check whether a tunnel can actually be build.
	 */
	static TileIndex GetOtherTunnelEnd(TileIndex tile);

	/**
	 * Internal function to help BuildTunnel in case of road.
	 * @api -all
	 */
	static bool _BuildTunnelRoad1();

	/**
	 * Internal function to help BuildTunnel in case of road.
	 * @api -all
	 */
	static bool _BuildTunnelRoad2();

	/**
	 * Builds a tunnel starting at start. The direction of the tunnel depends
	 *  on the slope of the start tile. Tunnels can be created for either
	 *  rails or roads; use the appropriate ScriptVehicle::VehicleType.
	 * As an extra for road, this functions builds two half-pieces of road on
	 *  each end of the tunnel, making it easier for you to connect it to your
	 *  network.
	 * @param start Where to start the tunnel.
	 * @param vehicle_type The vehicle-type of tunnel to build.
	 * @pre ScriptMap::IsValidTile(start).
	 * @pre (vehicle_type == ScriptVehicle::VT_ROAD && ScriptRoad::IsRoadTypeAvailable(ScriptRoad::GetCurrentRoadType())) ||
	 *      (vehicle_type == ScriptVehicle::VT_RAIL && ScriptRail::IsRailTypeAvailable(ScriptRail::GetCurrentRailType())).
	 * @game @pre ScriptCompanyMode::IsValid() || vehicle_type == ScriptVehicle::VT_ROAD.
	 * @exception ScriptError::ERR_AREA_NOT_CLEAR
	 * @exception ScriptTunnel::ERR_TUNNEL_CANNOT_BUILD_ON_WATER
	 * @exception ScriptTunnel::ERR_TUNNEL_START_SITE_UNSUITABLE
	 * @exception ScriptTunnel::ERR_TUNNEL_ANOTHER_TUNNEL_IN_THE_WAY
	 * @exception ScriptTunnel::ERR_TUNNEL_END_SITE_UNSUITABLE
	 * @return Whether the tunnel has been/can be build or not.
	 * @note The slope of a tile can be determined by ScriptTile::GetSlope(TileIndex).
	 * @note No matter if the road pieces were build or not, if building the
	 *  tunnel succeeded, this function returns true.
	 * @game @note Building a tunnel as deity (ScriptCompanyMode::IsDeity()) results in a tunnel owned by towns.
	 */
	static bool BuildTunnel(ScriptVehicle::VehicleType vehicle_type, TileIndex start);

	/**
	 * Check whether the tile is an entrance to one or two diagonal tunnels.
	 * Diagonal tunnels are rail tunnels which run along the half tile track pieces (ScriptRail::RAILTRACK_NW_NE,
	 *  ScriptRail::RAILTRACK_SW_SE, ScriptRail::RAILTRACK_NW_SW and ScriptRail::RAILTRACK_NE_SE), i.e. diagonally with respect to the tile grid.
	 * The entrance tile of a diagonal tunnel is a rail tile, each track of which is the entrance of a separate diagonal tunnel.
	 * @param tile The tile to check.
	 * @pre ScriptMap::IsValidTile(tile).
	 * @return True if and only if the tile is the entrance of one or two diagonal tunnels.
	 * @note This is a non-standard addition to the script API.
	 */
	static bool IsDiagonalTunnelTile(TileIndex tile);

	/**
	 * Get the tile at the other end of a diagonal tunnel.
	 * @param tile The tile that is an entrance to a diagonal tunnel.
	 * @param track The entrance track of the diagonal tunnel on tile.
	 * @pre IsDiagonalTunnelTile(tile).
	 * @return The tile at the other end of the diagonal tunnel, or ScriptMap::TILE_INVALID if there is no such tunnel.
	 * @note This is a non-standard addition to the script API.
	 */
	static TileIndex GetOtherDiagonalTunnelEnd(TileIndex tile, ScriptRail::RailTrack track);

	/**
	 * Builds a diagonal rail tunnel starting at start, using the current rail type.
	 * The start tile must have a slope with exactly one corner raised. The tunnel leads into the hill towards the raised corner,
	 *  starting from one of the two half tile track pieces which are adjacent to the raised corner.
	 * The other end of the tunnel is where the ground along the line of the tunnel descends back to the level of the start tile.
	 * The entrance tracks of diagonal tunnels can be removed using ScriptRail::RemoveRailTrack, this removes the whole tunnel.
	 * @param start Where to start the tunnel.
	 * @param track The entrance track of the tunnel on the start tile.
	 * @pre ScriptMap::IsValidTile(start).
	 * @pre track is one of ScriptRail::RAILTRACK_NW_NE, ScriptRail::RAILTRACK_SW_SE, ScriptRail::RAILTRACK_NW_SW or ScriptRail::RAILTRACK_NE_SE.
	 * @pre ScriptRail::IsRailTypeAvailable(ScriptRail::GetCurrentRailType()).
	 * @pre ScriptCompanyMode::IsValid().
	 * @exception ScriptError::ERR_AREA_NOT_CLEAR
	 * @exception ScriptTunnel::ERR_TUNNEL_CANNOT_BUILD_ON_WATER
	 * @exception ScriptTunnel::ERR_TUNNEL_ANOTHER_TUNNEL_IN_THE_WAY
	 * @exception ScriptTunnel::ERR_TUNNEL_END_SITE_UNSUITABLE
	 * @return Whether the tunnel has been/can be build or not.
	 * @note This is a non-standard addition to the script API.
	 */
	static bool BuildDiagonalTunnel(TileIndex start, ScriptRail::RailTrack track);

	/**
	 * Remove the tunnel whose entrance is located at tile.
	 * @param tile The tile that is an entrance to a tunnel.
	 * @pre ScriptMap::IsValidTile(tile) && IsTunnelTile(tile).
	 * @game @pre ScriptCompanyMode::IsValid().
	 * @exception ScriptError::ERR_OWNED_BY_ANOTHER_COMPANY
	 * @return Whether the tunnel has been/can be removed or not.
	 */
	static bool RemoveTunnel(TileIndex tile);
};

#endif /* SCRIPT_TUNNEL_HPP */
