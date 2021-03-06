#ifndef _PID_H_
#define _PID_H_

#include "core/logger.h"

namespace mavhub {
	class PID {
	public:
		PID(double bias, double Kc, double Ti, double Td);
		virtual ~PID();
		virtual double calc(double dt);
		virtual double calc(double dt, double meas);
		virtual double calc(double dt, double meas, double setpoint);
		virtual double calc2(double dt, double meas);
		/// controller integral getter
		virtual double getIntegral();
		/// controller integral setter
		virtual void setIntegral(double integral);
		/// reset integral to last value
		virtual void setIntegralM1();
		// PID setter
		/// set setpoint
		void setSp(double sp_);
		double getSp();
		/// set process variable by measurement
		void setPv(double meas);
		double getPv();
		// state getter
		/// get error
		double getErr();
		/// get dpv
		double getDpv();
		/// get pv_m1
		double getPv_m1();
		/// get ppart
		double getPpart();
		/// get ipart
		double getIpart();
		/// get dpart
		double getDpart();
		/// get pv_int
		double getPv_int();
		/// get pv_int_m1
		double getPv_int_m1();
		/// get pv_int_lim
		double getPv_int_lim();

		/// get controller bias
		double getBias();
		/// get controller gain
		double getKc();
		/// get controller reset time
		double getTi();
		/// get controller derivative time
		double getTd();
		/// set controller bias
		void setBias(double bias);
		/// set controller gain
		void setKc(double kc);
		/// set reset time
		void setTi(double ti);
		/// set controller gain
		void setTd(double td);
		/// set controller gain
		void setPv_int_lim(double lim);

	private:
		double bias; //!< controller bias
		double Kc; //!< controller gain (P-part coefficient)
		double Ti; //!< reset time (I-part coefficient)
		double Td; //!< derivative time (D-part coefficient)

		double sp; //!< setpoint
		double pv; //!< process variable
		double dpv; //!< dpv/dt
		double err; //!< error e
		double pv_m1; //!< dv_{k-1}
		double ppart; //!< P-part
		double ipart; //!< I-part
		double dpart; //!< D-part
		double pv_int; //!< \f$ \int_{t_0}^{t_k} e(t) dt \f$
		double pv_int_m1; //!< \f$ \int_{t_0}^{t_{k-1}} e(t) dt \f$

		double pv_int_lim;
	};

	/// set setpoint
	inline void PID::setSp(double sp_) {
		sp = sp_;
	}
	/// get setpoint
	inline double PID::getSp() {return sp;}
	/// set process variable
	inline void PID::setPv(double meas) {
		pv = meas;
	}
	/// set controller bias
	inline void PID::setBias(double bias_) { bias = bias_; }
	/// set controller gain
	inline void PID::setKc(double kc_) { Kc = kc_; }
	/// set reset time
	inline void PID::setTi(double ti_) { Ti = ti_; }
	/// set controller gain
	inline void PID::setTd(double td_) { Td = td_; }
	/// set integral limit
	inline void PID::setPv_int_lim(double lim_) { pv_int_lim = lim_; }
	
	/// get controller bias
	inline double PID::getBias() {return bias;}
	/// get controller bias
	inline double PID::getKc() {return Kc;}
	/// get controller bias
	inline double PID::getTi() {return Ti;}
	/// get controller bias
	inline double PID::getTd() {return Td;}
	/// get process variable current value
	inline double PID::getPv() {return pv;}
	/// get error
	inline double PID::getErr() {return err;}
	/// get dpv
	inline double PID::getDpv() {return dpv;}
	/// get pv_m1
	inline double PID::getPv_m1() {return pv_m1;}
	/// get ppart
	inline double PID::getPpart() {return ppart;}
	/// get ipart
	inline double PID::getIpart() {return ipart;}
	/// get dpart
	inline double PID::getDpart() {return dpart;}
	/// get pv_int
	inline double PID::getPv_int() {return pv_int;}
	/// get pv_int_m1
	inline double PID::getPv_int_m1() {return pv_int_m1;}
	/// get pv_int_m1
	inline double PID::getPv_int_lim() {return pv_int_lim;}
}

#endif
