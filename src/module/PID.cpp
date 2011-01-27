#include "PID.h"

using namespace std;

namespace mavhub {
	PID::PID(double bias_, double Kc_, double Ti_, double Td_) {
		bias = bias_;
		Kc = Kc_;
		Ti = Ti_;
		Td = Td_;

		err = 0.0;
		dpv = 0.0;
		pv_m1 = 0.0;
		ppart = 0.0;
		ipart = 0.0;
		dpart = 0.0;
		pv_int = 0.0;
		pv_int_m1 = 0.0;
	}

	PID::~PID() {
	}

	double PID::calc(double dt, double meas, double setpoint) {
		// setSp(setpoint);
		sp = setpoint;
		pv = meas;
		return calc(dt);
	}

	double PID::calc(double dt, double meas) {
		pv = meas;
		return calc(dt);
	}

	double PID::calc(double dt) {
		// Logger::log("PID::calc:", pv, sp, Logger::LOGLEVEL_INFO);
		// Logger::log("PID::calc:", bias, Kc, Logger::LOGLEVEL_INFO);
		// Logger::log("PID::calc:", Ti, Td, Logger::LOGLEVEL_INFO);

		err = sp - pv;
		dpv = pv - pv_m1; // XXX: we regulate on dpv rather than on
		                  // XXX: derr, the value is the same and we
		                  // XXX: avoid unwanted large values from setpoint
                      // XXX: changes
		pv_m1 = pv;

		// D-part
		if(dt != 0)
			dpart = Td * dpv / dt; // check beagle
		else
			dpart = 0.f;

		// I-part
		pv_int_m1 = pv_int;
		pv_int += err * dt;
		// limit integral
		if (pv_int > 12000.0)
			pv_int = 12000.0;
		if (pv_int < -12000.0)
			pv_int = -12000.0;

		// divide by zero
		if(Ti != 0)
			ipart = pv_int / Ti;
		else
			ipart = 0.0;
		
		// compute corrective (watch dpart sign)
		return(bias + Kc * (err + ipart - dpart));
	}

	double PID::getIntegral() {
		return pv_int;
	}

	void PID::setIntegral(double integral) {
		pv_int = integral;
	}

	// reset integral to last value
	void PID::setIntegralM1() {
		pv_int = pv_int_m1;
	}
}
