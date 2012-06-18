#include "Bumper.h"

using namespace std;

namespace mavhub {
	Bumper::Bumper(double thr_low_, double thr_high_) :
		thr_low(thr_low_),
		thr_high(thr_high_),
		bump_dur(3.0),
		bump_now(0.0),
		bump_run(false),
		bump_delay(3.0)
	{
	}

	Bumper::Bumper(double thr_low_, double thr_high_, double dur_) :
		thr_low(thr_low_),
		thr_high(thr_high_),
		bump_dur(dur_),
		bump_now(0.0),
		bump_run(false),
		bump_delay(3.0)
	{
	}

	Bumper::Bumper(double thr_low_, double thr_high_,
								 double dur_, double delay_) :
		thr_low(thr_low_),
		thr_high(thr_high_),
		bump_dur(dur_),
		bump_now(0.0),
		bump_run(false),
		bump_delay(delay_)
	{
	}

	Bumper::~Bumper() {}

	void Bumper::set_thr_low(double val) {
		thr_low = val;
	}

	void Bumper::set_thr_high(double val) {
		thr_high = val;
	}

	void Bumper::set_bump_dur(double val) {
		bump_dur = val;
	}

	void Bumper::set_bump_delay(double val) {
		bump_delay = val;
	}

	double Bumper::calc(double dt) {
		double thr;
		if(bump_run) {
			bump_now += dt;
			if(bump_now <= bump_delay)
				thr = 0.0;
			else if(bump_now > bump_delay &&
				 bump_now <= (bump_delay + bump_dur)) {
				thr = thr_high;
			}
			else if(bump_now > (bump_delay + bump_dur) &&
							bump_now <= (bump_delay + (2*bump_dur))) {
				thr = thr_low;
			}
			else {
				bump_run = false;
				thr = 0.0;
			}
			Logger::log("Bumper::calc() bumping", bump_now, thr, Logger::LOGLEVEL_INFO);
		}
		else
			thr = 0.0;
		return thr;
	}

	void Bumper::bump(double now) {
		bump_now = 0.0;
		bump_run = true;
	}
}
