/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file airport_capacity.cpp Estimation of the capacity of airports, compared to the scheduled aircraft movements. */

#include "stdafx.h"
#include "airport_capacity.h"
#include "aircraft.h"
#include "date_func.h"
#include "newgrf_airport.h"
#include "order_base.h"
#include "station_base.h"

#include "safeguards.h"

/** Index of the statistics and standard values of aeroplanes and helicopters. */
enum AircraftClass : uint8_t {
	AC_AEROPLANE  = 0, ///< Aeroplanes.
	AC_HELICOPTER = 1, ///< Helicopters.
	AC_END,
};

/** Minimum number of landings at an airport before its measured block usage is used instead of the standard values. */
static constexpr uint32_t MIN_MEASURED_LANDINGS = 8;

/**
 * A group of equivalent runways of an airport layout, of which each flight uses one.
 * For heliports these are the landing area or the helipads instead.
 */
struct RunwayGroup {
	AirportBlocks blocks; ///< Blocks of the runways in this group.
	uint8_t count;        ///< Number of runways in this group, which can be used at the same time.
	/**
	 * Standard time in ticks each flight occupies a runway of this group, by aeroplanes and helicopters, at plane speed 1/1 and 1/4.
	 * These were measured with a typical jet aeroplane and helicopter, without time for loading. Slower aircraft take longer.
	 */
	std::array<std::array<uint16_t, 2>, AC_END> standard_ticks;
};

/** Runway groups of each default airport layout. */
static const std::vector<RunwayGroup> _airport_runway_groups[] = {
	/* AT_SMALL: the airport is busy from landing until the aircraft reaches its terminal. */
	{ { AirportBlock::AirportBusy, 1, {{ {470, 540}, {345, 726} }} } },
	/* AT_LARGE */
	{ { AirportBlock::RunwayInOut, 1, {{ {306, 357}, {0, 0} }} } },
	/* AT_HELIPORT: helicopters can only land when the helipad is free. */
	{ { {AirportBlock::AirportBusy, AirportBlock::Helipad1}, 1, {{ {0, 0}, {118, 246} }} } },
	/* AT_METROPOLITAN: landing aircraft also cross the take-off runway. */
	{ { AirportBlock::RunwayIn, 1, {{ {207, 230}, {0, 0} }} }, { AirportBlock::RunwayOut, 1, {{ {211, 253}, {0, 0} }} } },
	/* AT_INTERNATIONAL */
	{ { AirportBlock::RunwayIn, 1, {{ {216, 225}, {0, 0} }} }, { AirportBlock::RunwayOut, 1, {{ {33, 74}, {0, 0} }} } },
	/* AT_COMMUTER */
	{ { AirportBlock::RunwayInOut, 1, {{ {226, 330}, {0, 0} }} } },
	/* AT_HELIDEPOT: helicopters wait in the air for the landing area, and go to the hangar when the helipad is busy. */
	{ { AirportBlock::PreHelipad, 1, {{ {0, 0}, {17, 34} }} }, { AirportBlock::Helipad1, 1, {{ {0, 0}, {99, 266} }} } },
	/* AT_INTERCON */
	{ { {AirportBlock::RunwayIn, AirportBlock::RunwayIn2}, 2, {{ {288, 372}, {0, 0} }} }, { {AirportBlock::RunwayOut, AirportBlock::RunwayOut2}, 2, {{ {93, 140}, {0, 0} }} } },
	/* AT_HELISTATION */
	{ { AirportBlock::PreHelipad, 1, {{ {0, 0}, {53, 140} }} }, { {AirportBlock::Helipad1, AirportBlock::Helipad2, AirportBlock::Helipad3}, 3, {{ {0, 0}, {99, 266} }} } },
	/* AT_OILRIG: same as the heliport. */
	{ { {AirportBlock::AirportBusy, AirportBlock::Helipad1}, 1, {{ {0, 0}, {118, 246} }} } },
};
static_assert(lengthof(_airport_runway_groups) == NEW_AIRPORT_OFFSET);

void Airport::RecordBlockReservation(AirportBlocks reserved, bool helicopter)
{
	if (this->block_usage == nullptr) this->block_usage = std::make_unique<AirportBlockUsageStats>();
	AirportBlockUsageStats &stats = *this->block_usage;
	for (AirportBlock block : reserved.IterateSetBits()) {
		uint index = to_underlying(block);
		if (index >= AirportBlockUsageStats::NUM_TRACKED_BLOCKS) continue;
		/* Add one, as zero means not reserved. */
		stats.reserve_tick[index] = _scaled_tick_counter + 1;
		stats.reserve_helicopter[index] = helicopter;
	}
}

void Airport::RecordBlockRelease(AirportBlocks released)
{
	if (this->block_usage == nullptr) return;
	AirportBlockUsageStats &stats = *this->block_usage;
	for (AirportBlock block : released.IterateSetBits()) {
		uint index = to_underlying(block);
		if (index >= AirportBlockUsageStats::NUM_TRACKED_BLOCKS || stats.reserve_tick[index] == 0) continue;
		uint32_t &usage = stats.usage[stats.reserve_helicopter[index] ? AC_HELICOPTER : AC_AEROPLANE][index];
		usage = static_cast<uint32_t>(std::min<uint64_t>(usage + _scaled_tick_counter + 1 - stats.reserve_tick[index], UINT32_MAX));
		stats.reserve_tick[index] = 0;
	}
}

void Airport::RecordLanding(bool helicopter)
{
	if (this->block_usage == nullptr) this->block_usage = std::make_unique<AirportBlockUsageStats>();
	AirportBlockUsageStats &stats = *this->block_usage;
	AircraftClass ac = helicopter ? AC_HELICOPTER : AC_AEROPLANE;
	if (++stats.landings[ac] >= AirportBlockUsageStats::MAX_LANDINGS) {
		stats.landings[ac] /= 2;
		for (uint32_t &usage : stats.usage[ac]) usage /= 2;
	}
}

/**
 * Get the period over which the movements of an airport are counted.
 * This is a month, or the equivalent in wallclock mode.
 * @return The length of the period in ticks.
 */
Ticks GetAirportCapacityPeriod()
{
	return 30 * DAY_TICKS * DayLengthFactor();
}

/**
 * Get the default airport type of which an airport uses the layout.
 * @param st Station with the airport.
 * @return The default airport type, or AT_INVALID if unknown.
 */
static uint8_t GetDefaultAirportLayoutType(const Station *st)
{
	const AirportFTAClass *fta = st->airport.GetFTA();
	for (uint8_t type = 0; type < NEW_AIRPORT_OFFSET; type++) {
		if (AirportSpec::GetWithoutOverride(type)->fsm == fta) return type;
	}
	return AT_INVALID;
}

/**
 * Get whether the block usage of a class of aircraft has been measured at an airport.
 * @param st Station with the airport.
 * @param ac Class of aircraft.
 * @return True iff there are enough measurements to use them.
 */
static bool HasMeasuredBlockUsage(const Station *st, AircraftClass ac)
{
	return st->airport.block_usage != nullptr && st->airport.block_usage->landings[ac] >= MIN_MEASURED_LANDINGS;
}

/**
 * Get the time each flight of a class of aircraft occupies a runway of a runway group.
 * @param st Station with the airport.
 * @param group The runway group.
 * @param ac Class of aircraft.
 * @return The occupancy in ticks.
 */
static uint64_t GetRunwayOccupancy(const Station *st, const RunwayGroup &group, AircraftClass ac)
{
	if (HasMeasuredBlockUsage(st, ac)) {
		const AirportBlockUsageStats &stats = *st->airport.block_usage;
		uint64_t usage = 0;
		for (AirportBlock block : group.blocks.IterateSetBits()) {
			uint index = to_underlying(block);
			if (index < AirportBlockUsageStats::NUM_TRACKED_BLOCKS) usage += stats.usage[ac][index];
		}
		return usage / stats.landings[ac];
	}

	/* Interpolate the standard values for the plane speed setting. */
	const auto &ticks = group.standard_ticks[ac];
	int speed = Clamp<int>(_settings_game.vehicle.plane_speed, 1, 4);
	return ticks[0] + ((static_cast<int>(ticks[1]) - static_cast<int>(ticks[0])) * (speed - 1)) / 3;
}

/**
 * Get whether the journey time to each destination of an order list is known, either timetabled or from previous journeys.
 * @param list The order list.
 * @return True iff the total duration of the order list is known.
 */
static bool HasKnownJourneyTimes(const OrderList *list)
{
	for (const Order *order : list->Orders()) {
		if (order->IsType(OT_IMPLICIT) || order->IsType(OT_CONDITIONAL)) continue;
		if (!order->IsTravelTimetabled() && order->GetTravelTime() == 0) return false;
	}
	return true;
}

/**
 * Estimate the capacity of an airport, and compare it to the scheduled aircraft movements.
 * @param st Station with the airport.
 * @return The estimate.
 */
AirportCapacityInfo GetAirportCapacityInfo(const Station *st)
{
	AirportCapacityInfo info;
	if (!st->facilities.Test(StationFacility::Airport)) return info;

	uint8_t layout_type = GetDefaultAirportLayoutType(st);
	if (layout_type == AT_INVALID) return info;
	const std::vector<RunwayGroup> &groups = _airport_runway_groups[layout_type];

	const uint64_t period = GetAirportCapacityPeriod();

	/* Scheduled landings at this airport per period, in thousandths. */
	std::array<uint64_t, AC_END> visits{};

	for (const OrderList *list : OrderList::Iterate()) {
		const Vehicle *first = list->GetFirstSharedVehicle();
		if (first == nullptr || first->type != VehicleType::Aircraft) continue;

		uint stops = 0;
		for (const Order *order : list->Orders()) {
			if (order->IsType(OT_GOTO_STATION) && order->GetDestination() == st->index) stops++;
		}
		if (stops == 0) continue;

		std::array<uint, AC_END> vehicles{};
		for (const Vehicle *v = first; v != nullptr; v = v->NextShared()) {
			if (v->vehstatus.Any({VehState::Stopped, VehState::Crashed})) continue;
			vehicles[v->subtype == AIR_HELICOPTER ? AC_HELICOPTER : AC_AEROPLANE]++;
		}
		const uint num_vehicles = vehicles[AC_AEROPLANE] + vehicles[AC_HELICOPTER];
		if (num_vehicles == 0) continue;

		/* Landings of all vehicles of this order list per period, in thousandths. */
		uint64_t list_visits = UINT64_MAX;
		bool estimated = false;

		Ticks duration = list->GetTimetableTotalDuration();
		if (duration == INVALID_TICKS) {
			duration = HasKnownJourneyTimes(list) ? list->GetTotalDuration() : 0;
			estimated = true;
		}
		if (duration > 0) list_visits = stops * num_vehicles * period * 1000 / duration;

		/* Vehicles can't depart more often than their dispatch schedules allow. */
		if (first->vehicle_flags.Test(VehicleFlag::ScheduledDispatch)) {
			for (const Order *order : list->Orders()) {
				if (!order->IsScheduledDispatchOrder(true)) continue;
				uint schedule_index = order->GetDispatchScheduleIndex();
				if (schedule_index >= list->GetScheduledDispatchScheduleCount()) continue;
				const DispatchSchedule &ds = list->GetDispatchScheduleByIndex(schedule_index);
				if (!ds.IsScheduledDispatchValid()) continue;
				list_visits = std::min<uint64_t>(list_visits, stops * ds.GetScheduledDispatch().size() * period * 1000 / ds.GetScheduledDispatchDuration());
				estimated = false;
			}
		}

		if (list_visits == UINT64_MAX) {
			info.unknown_aircraft += num_vehicles;
			continue;
		}

		info.aircraft += num_vehicles;
		if (estimated) info.estimated_aircraft += num_vehicles;
		for (uint ac = 0; ac < AC_END; ac++) {
			visits[ac] += list_visits * vehicles[ac] / num_vehicles;
		}
	}

	const uint64_t total_visits = visits[AC_AEROPLANE] + visits[AC_HELICOPTER];

	/* Find the runway group which is busy for the largest part of the period. */
	uint64_t max_load = 0;
	uint64_t max_load_count = 1;
	for (const RunwayGroup &group : groups) {
		uint64_t load = 0;
		for (uint ac = 0; ac < AC_END; ac++) {
			load += visits[ac] * GetRunwayOccupancy(st, group, static_cast<AircraftClass>(ac));
		}
		if (load * max_load_count >= max_load * group.count) {
			max_load = load;
			max_load_count = group.count;
		}
	}

	info.valid = true;
	info.scheduled_movements = 2 * total_visits / 100;

	if (max_load > 0) {
		/* The capacity for the current mix of aircraft. */
		info.measured = true;
		for (uint ac = 0; ac < AC_END; ac++) {
			if (visits[ac] > 0 && !HasMeasuredBlockUsage(st, static_cast<AircraftClass>(ac))) info.measured = false;
		}
		info.capacity_movements = 2 * total_visits * 10 * max_load_count * period / max_load;
		info.utilisation = static_cast<uint>((max_load * 100 + max_load_count * period * 500) / (max_load_count * period * 1000));
		info.occupancy_per_flight = static_cast<Ticks>(max_load / total_visits);
		return info;
	}

	/* Nothing is scheduled which uses the runways: give the capacity for the main type of aircraft of this airport. */
	AircraftClass ac = st->airport.GetFTA()->flags.Test(AirportFTAClass::Flag::Airplanes) ? AC_AEROPLANE : AC_HELICOPTER;
	info.measured = HasMeasuredBlockUsage(st, ac);
	for (const RunwayGroup &group : groups) {
		uint64_t occupancy = GetRunwayOccupancy(st, group, ac);
		if (occupancy == 0) continue;
		uint64_t capacity = 2 * group.count * period * 10 / occupancy;
		if (info.capacity_movements == 0 || capacity < info.capacity_movements) {
			info.capacity_movements = capacity;
			info.occupancy_per_flight = static_cast<Ticks>(occupancy);
		}
	}
	return info;
}
