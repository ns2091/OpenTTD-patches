/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file airport_capacity.h Estimation of the capacity of airports, compared to the scheduled aircraft movements. */

#ifndef AIRPORT_CAPACITY_H
#define AIRPORT_CAPACITY_H

#include "date_type.h"

struct Station;

/**
 * Scheduled aircraft movements of an airport, compared to the number of movements the airport can handle.
 * The capacity is the number of movements for which no aircraft has to circle before landing, or wait before
 * taking off, provided that the flights are evenly spaced. It is limited by how long each flight occupies
 * the runways (or landing area and helipads of heliports).
 * All movement counts are per period of #GetAirportCapacityPeriod, in tenths.
 */
struct AirportCapacityInfo {
	bool valid = false;                   ///< Whether the capacity of this airport could be estimated.
	uint64_t scheduled_movements = 0;     ///< Scheduled landings plus take-offs, in tenths.
	uint64_t capacity_movements = 0;      ///< Movements the airport can handle with the current mix of aircraft, in tenths.
	uint utilisation = 0;                 ///< Utilisation of the busiest runway (group), in percent.
	Ticks occupancy_per_flight = 0;       ///< Time each flight occupies the busiest runway (group), for the current mix of aircraft.
	bool measured = false;                ///< Whether the occupancy has been measured at this airport, instead of using standard values.
	uint aircraft = 0;                    ///< Number of aircraft included in the scheduled movements.
	uint estimated_aircraft = 0;          ///< Number of included aircraft without complete timetable, using the duration of previous journeys.
	uint unknown_aircraft = 0;            ///< Number of aircraft serving the airport which could not be included, as their journey times are unknown.
};

Ticks GetAirportCapacityPeriod();
AirportCapacityInfo GetAirportCapacityInfo(const Station *st);

#endif /* AIRPORT_CAPACITY_H */
