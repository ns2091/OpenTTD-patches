/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file tunnelbridge_station_sl.cpp Code handling saving and loading of tunnel/bridge station platforms */

#include "../stdafx.h"
#include "../tunnelbridge_station.h"
#include "saveload.h"

#include "../safeguards.h"

/** Saved data of a tunnel/bridge station platform, the array index is the northern end tile of the tunnel/bridge. */
struct TunnelBridgeStationStub {
	StationID station = StationID::Invalid();
};

static const NamedSaveLoad _tunnel_bridge_station_desc[] = {
	NSL("station", SLE_VAR(TunnelBridgeStationStub, station, SLE_UINT16)),
};

static void Load_TBST()
{
	SaveLoadTableData slt = SlTableHeaderOrRiff(_tunnel_bridge_station_desc);

	int index;
	while ((index = SlIterateArray()) != -1) {
		TunnelBridgeStationStub stub{};
		SlObjectLoadFiltered(&stub, slt);
		InsertTunnelBridgeStationOnLoad(TileIndex(index), stub.station);
	}
}

static void Save_TBST()
{
	SaveLoadTableData slt = SlTableHeader(_tunnel_bridge_station_desc);

	IterateTunnelBridgeStations([&](TileIndex tile, StationID station) {
		TunnelBridgeStationStub stub{ station };
		SlSetArrayIndex(tile.base());
		SlObjectSaveFiltered(&stub, slt);
	});
}

extern const ChunkHandler tunnel_bridge_station_chunk_handlers[] = {
	{ 'TBST', Save_TBST, Load_TBST, nullptr, nullptr, ChunkType::SparseTable },
};

extern const ChunkHandlerTable _tunnel_bridge_station_chunk_handlers(tunnel_bridge_station_chunk_handlers);
