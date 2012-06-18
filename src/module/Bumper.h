// generate system identification signals
// - step signal
// - impulse
// - frequencies

#ifndef _BUMPER_H_
#define _BUMPER_H_

#include "core/logger.h"

namespace mavhub {
	class Bumper {
	public:
		Bumper(double low, double high);
		Bumper(double low, double high, double dur);
		Bumper(double low, double high, double dur, double delay);
		virtual ~Bumper();
		/// calculation function
		virtual double calc(double dt);
		/// start bump test
		virtual void bump(double now);
		/// set thr low
		virtual void set_thr_low(double val);
		/// set thr high
		virtual void set_thr_high(double val);
		/// set bump duration
		virtual void set_bump_dur(double val);
		/// set pre-bump delay
		virtual void set_bump_delay(double val);
	private:
		/// bump low throttle
		double thr_low;
		/// bump high throttle
		double thr_high;
		/// bump segment duration
		double bump_dur;
		/// bump now time
		double bump_now;
		/// bump run flag
		bool bump_run;
		/// bump delay
		double bump_delay;
	};
}

#endif
