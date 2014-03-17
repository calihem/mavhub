/****************************************************************************
** Copyright 2011 Humboldt-Universitaet zu Berlin
**
** This file is part of MAVHUB.
**
** MAVHUB is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** MAVHUB is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with MAVHUB.  If not, see <http://www.gnu.org/licenses/>.
**
*****************************************************************************/
/**
 * \file exec_timing.h
 * \date created at 2011/08/30
 * \author Oswald Berthold
 *
 * \brief Handle timing calculation for execution timing
 *        of controller modules
 *
 * \sa exec_timing.cpp
 */

// Execution timing class

#ifndef _EXEC_TIMING_H_
#define _EXEC_TIMING_H_

#include <inttypes.h> // for uint64_t
#include <sys/time.h> //us
#include <vector>

namespace mavhub {
	class Exec_Timing {
	public:
		/// set up Exec_Timing filter with given order (number of taps) and coefficients
		//Exec_Timing(int order_, std::vector<double> &coeff);
		Exec_Timing(int rate);
		virtual ~Exec_Timing();

		/// calculate next sample
		virtual int calcSleeptime();
		virtual uint64_t updateExecStats();
	private:
		/// dt since last call in microseconds
		uint64_t dt;
		/// current timestamp
		struct timeval tk;
		/// previous timestamp
		struct timeval tkm1;
		/// overall waiting time
		int wait_freq;
		/// spare time (overall waiting time minus time needed for execution)
		int wait_time;
		/// wiating time inverse
		uint64_t frequency;
		/// start time
		uint64_t start;
		/// end time
		uint64_t end;
		/// usec timestamp
		uint64_t usec;
	};
}

#endif
