// Execution timing class

#ifndef _EXEC_TIMING_H_
#define _EXEC_TIMING_H_

#include "utility.h" // for uint64_t

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
