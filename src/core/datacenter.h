#ifndef _DATACENTER_H_
#define _DATACENTER_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#include "core/thread.h"

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>
#endif // HAVE_MAVLINK_H

#ifdef HAVE_OPENCV2
#include <opencv/cv.h>
#endif // HAVE_OPENCV2


#define DC_NUMSENS 10 // FIXME: get from config

namespace mavhub {

	class DataCenter {
		public:
			/// set FC legacy extctrl components
			static void set_extctrl_pitch(const double pitch);
			static void set_extctrl_roll(const double roll);
			static void set_extctrl_yaw(const double yaw);
			static const double get_extctrl_pitch();
			static const double get_extctrl_roll();
			static const double get_extctrl_yaw();

			// unified sensor array read/write
			static void set_sensor(const int id, const double val);
			static const double get_sensor(const int id);

#ifdef HAVE_MAVLINK_H
			// raw imu 
			static const mavlink_raw_imu_t get_raw_imu();
			static void set_raw_imu(const mavlink_raw_imu_t &mavlink_raw_imu);
			/**
			 * Raw pressure
			 */
			static void set_raw_pressure(const mavlink_raw_pressure_t &raw_pressure);
			static const mavlink_raw_pressure_t get_raw_pressure();
#ifdef MAVLINK_ENABLED_HUCH
			/// Set ExpCtrl data
			static void set_exp_ctrl(const mavlink_huch_exp_ctrl_rx_t &exp_ctrl_rx_data);
			static const mavlink_huch_exp_ctrl_rx_t get_exp_ctrl();
			/**
			 * HUCH IMU raw adc
			 */
			static void set_huch_imu_raw_adc(const mavlink_huch_imu_raw_adc_t &huch_imu_raw_adc);
			static const mavlink_huch_imu_raw_adc_t get_huch_imu_raw_adc();
			/**
			 * HUCH MK IMU
			 */
			static void set_huch_mk_imu(const mavlink_huch_mk_imu_t &huch_mk_imu);
			static const mavlink_huch_mk_imu_t get_huch_mk_imu();
			// FlightCtrl legacy

			/// get FC legacy data
			static const mavlink_huch_attitude_t get_huch_attitude();
			static const mavlink_huch_fc_altitude_t get_huch_fc_altitude();
			static const mavlink_huch_ranger_t get_huch_ranger();
			static const mavlink_mk_fc_status_t get_mk_fc_status();

			/// set FC legacy data
			static void set_huch_attitude(const mavlink_huch_attitude_t &huch_attitude);
			static void set_huch_fc_altitude(const mavlink_huch_fc_altitude_t &huch_fc_altitude);
			static void set_huch_ranger_at(const mavlink_huch_ranger_t &huch_ranger, int index);
			static void set_mk_fc_status(const mavlink_mk_fc_status_t &mk_fc_status);

			/**
			 * HUCH MAGNETIC KOMPASS
			 */
			static void set_huch_magnetic_kompass(const mavlink_huch_magnetic_kompass_t &huch_magnetic_kompass);
			static const mavlink_huch_magnetic_kompass_t get_huch_magnetic_kompass();
#endif // MAVLINK_ENABLED_HUCH
#endif // HAVE_MAVLINK_H
#ifdef HAVE_OPENCV2
			// fiducal data
			static void set_fiducal_rot_raw(const cv::Mat &rvec);
			static void set_fiducal_trans_raw(const cv::Mat &tvec);
			static cv::Mat get_fiducal_rot_raw();
			static cv::Mat get_fiducal_trans_raw();
#endif // HAVE_OPENCV2

		private:
			static double extctrl_pitch;
			static double extctrl_roll;
			static double extctrl_yaw;
			// unified sensor array
			static double sensors[DC_NUMSENS];

			static pthread_mutex_t sensors_mutex;
			static pthread_mutex_t extctrl_mutex;
#ifdef HAVE_MAVLINK_H
			//data structs
			static mavlink_raw_imu_t raw_imu;
			static mavlink_raw_pressure_t raw_pressure;

			//sync data
			static pthread_mutex_t raw_imu_mutex;
			static pthread_mutex_t raw_pressure_mutex;
#ifdef MAVLINK_ENABLED_HUCH
			static mavlink_huch_imu_raw_adc_t huch_imu_raw_adc;
			static mavlink_huch_mk_imu_t huch_mk_imu;
			static mavlink_huch_attitude_t huch_attitude;
			static mavlink_huch_fc_altitude_t huch_altitude;
			static mavlink_huch_ranger_t huch_ranger;
			static mavlink_mk_fc_status_t mk_fc_status;
			/// ExpCtrl data structure
			static mavlink_huch_exp_ctrl_rx_t exp_ctrl_rx_data;
			/// Magnetic kompass
			static mavlink_huch_magnetic_kompass_t huch_magnetic_kompass;

			static pthread_mutex_t exp_ctrl_mutex;
			static pthread_mutex_t huch_imu_raw_adc_mutex;
			static pthread_mutex_t huch_mk_imu_mutex;
			static pthread_mutex_t huch_magnetic_kompass_mutex;
			// FlightCtrl legacy
			static pthread_mutex_t mk_fc_mutex;
			static pthread_mutex_t huch_ranger_mutex;
#endif // MAVLINK_ENABLED_HUCH
#endif // HAVE_MAVLINK_H
#ifdef HAVE_OPENCV2
			// fiducal data structures
			static cv::Mat fiducal_rot_raw;
			static cv::Mat fiducal_trans_raw;

			static pthread_mutex_t fiducal_raw_mutex;
#endif // HAVE_OPENCV2

			DataCenter();
			DataCenter(const DataCenter &data);
			~DataCenter();
			void operator=(const DataCenter &data);
	};

// ----------------------------------------------------------------------------
// DataCenter
// ----------------------------------------------------------------------------
// extctrl component setters
inline void DataCenter::set_extctrl_pitch(const double pitch) {
	using namespace cpp_pthread;

	Lock ri_lock(extctrl_mutex);
	extctrl_pitch = pitch;
}
inline void DataCenter::set_extctrl_roll(const double roll) {
	using namespace cpp_pthread;

	Lock ri_lock(extctrl_mutex);
	extctrl_roll = roll;
}
inline void DataCenter::set_extctrl_yaw(const double yaw) {
	using namespace cpp_pthread;

	Lock ri_lock(extctrl_mutex);
	extctrl_yaw = yaw;
}

// extctrl component getters
inline const double DataCenter::get_extctrl_pitch() {
	using namespace cpp_pthread;

	Lock ri_lock(extctrl_mutex);
	return extctrl_pitch;
}
inline const double DataCenter::get_extctrl_roll() {
	using namespace cpp_pthread;

	Lock ri_lock(extctrl_mutex);
	return extctrl_roll;
}
inline const double DataCenter::get_extctrl_yaw() {
	using namespace cpp_pthread;

	Lock ri_lock(extctrl_mutex);
	return extctrl_yaw;
}

// unified sensor array read/write
inline void DataCenter::set_sensor(const int id, const double val) {
	using namespace cpp_pthread;

	Lock ri_lock(sensors_mutex);
	if(id >= 0 && id < DC_NUMSENS)
		sensors[id] = val;
	//else
		//Logger::log("DC::set_sensor: invalid index", Logger::LOGLEVEL_DEBUG);
	return;
}
inline const double DataCenter::get_sensor(const int id) {
	using namespace cpp_pthread;

	Lock ri_lock(extctrl_mutex);
	if(id >= 0 && id < DC_NUMSENS)
		return sensors[id];
	else {
		//Logger::log("DC::get_sensor: invalid index", Logger::LOGLEVEL_DEBUG);
		return -1;
	}
}

#ifdef HAVE_MAVLINK_H
	inline const mavlink_raw_imu_t DataCenter::get_raw_imu() {
		using namespace cpp_pthread;

		Lock ri_lock(raw_imu_mutex);
		mavlink_raw_imu_t raw_imu_copy(raw_imu);

		return raw_imu_copy;
	}
	inline void DataCenter::set_raw_imu(const mavlink_raw_imu_t &mavlink_raw_imu) {
		using namespace cpp_pthread;

		Lock ri_lock(raw_imu_mutex);
		raw_imu = mavlink_raw_imu;
	}
	inline const mavlink_raw_pressure_t DataCenter::get_raw_pressure() {
		using namespace cpp_pthread;

		Lock rp_lock(raw_pressure_mutex);
		mavlink_raw_pressure_t raw_pressure_copy(raw_pressure);

		return raw_pressure_copy;
	}
	inline void DataCenter::set_raw_pressure(const mavlink_raw_pressure_t &raw_pressure) {
		using namespace cpp_pthread;

		Lock rp_lock(raw_pressure_mutex);
		DataCenter::raw_pressure = raw_pressure;
	
	}
#ifdef MAVLINK_ENABLED_HUCH
	inline const mavlink_huch_exp_ctrl_rx_t DataCenter::get_exp_ctrl() {
		using namespace cpp_pthread;

		Lock ri_lock(exp_ctrl_mutex);
		mavlink_huch_exp_ctrl_rx_t exp_ctrl_data_copy(exp_ctrl_rx_data);

		return exp_ctrl_data_copy;
	}
	inline void DataCenter::set_exp_ctrl(const mavlink_huch_exp_ctrl_rx_t &exp_ctrl_rx_data) {
		using namespace cpp_pthread;

		Lock ri_lock(exp_ctrl_mutex);
		DataCenter::exp_ctrl_rx_data = exp_ctrl_rx_data;
	}

	inline const mavlink_huch_imu_raw_adc_t DataCenter::get_huch_imu_raw_adc() {
		using namespace cpp_pthread;

		Lock ira_lock(raw_imu_mutex);
		mavlink_huch_imu_raw_adc_t imu_raw_adc_copy(huch_imu_raw_adc);

		return imu_raw_adc_copy;
	}
	inline void DataCenter::set_huch_imu_raw_adc(const mavlink_huch_imu_raw_adc_t &huch_imu_raw_adc) {
		using namespace cpp_pthread;

		Lock ira_lock(huch_imu_raw_adc_mutex);
		DataCenter::huch_imu_raw_adc = huch_imu_raw_adc;
	}

	inline const mavlink_huch_mk_imu_t DataCenter::get_huch_mk_imu() {
		using namespace cpp_pthread;

		Lock mi_lock(huch_mk_imu_mutex);
		mavlink_huch_mk_imu_t mk_imu_copy(huch_mk_imu);

		return mk_imu_copy;
	}
	inline void DataCenter::set_huch_mk_imu(const mavlink_huch_mk_imu_t &huch_mk_imu) {
		using namespace cpp_pthread;

		Lock mi_lock(huch_imu_raw_adc_mutex);
		DataCenter::huch_mk_imu = huch_mk_imu;
	}

	inline const mavlink_huch_magnetic_kompass_t DataCenter::get_huch_magnetic_kompass() {
		using namespace cpp_pthread;

		Lock ira_lock(raw_imu_mutex);
		mavlink_huch_magnetic_kompass_t magnetic_kompass_copy(huch_magnetic_kompass);

		return magnetic_kompass_copy;
	}
	inline void DataCenter::set_huch_magnetic_kompass(const mavlink_huch_magnetic_kompass_t &huch_magnetic_kompass) {
		using namespace cpp_pthread;

		Lock ira_lock(huch_magnetic_kompass_mutex);
		DataCenter::huch_magnetic_kompass = huch_magnetic_kompass;
	}

	// FlightCtrl legacy functions
	// attitude
	inline const mavlink_huch_attitude_t DataCenter::get_huch_attitude() {
		using namespace cpp_pthread;

		Lock ri_lock(mk_fc_mutex);
		mavlink_huch_attitude_t huch_attitude_copy(huch_attitude);

		return huch_attitude_copy;
	}
	inline void DataCenter::set_huch_attitude(const mavlink_huch_attitude_t &huch_attitude_a) {
		using namespace cpp_pthread;

		Lock ri_lock(mk_fc_mutex);
		DataCenter::huch_attitude = huch_attitude_a;
	}
	// altitude
	inline const mavlink_huch_fc_altitude_t DataCenter::get_huch_fc_altitude() {
		using namespace cpp_pthread;

		Lock ri_lock(mk_fc_mutex);
		mavlink_huch_fc_altitude_t huch_fc_altitude_copy(huch_altitude);

		return huch_fc_altitude_copy;
	}
	inline void DataCenter::set_huch_fc_altitude(const mavlink_huch_fc_altitude_t &huch_fc_altitude_a) {
		using namespace cpp_pthread;

		Lock ri_lock(mk_fc_mutex);
		DataCenter::huch_altitude = huch_fc_altitude_a;
	}
	// ranger
	inline const mavlink_huch_ranger_t DataCenter::get_huch_ranger() {
		using namespace cpp_pthread;

		Lock ri_lock(huch_ranger_mutex);
		mavlink_huch_ranger_t huch_ranger_copy(huch_ranger);

		return huch_ranger_copy;
	}
	inline void DataCenter::set_huch_ranger_at(const mavlink_huch_ranger_t &huch_ranger_a, int index) {
		using namespace cpp_pthread;

		Lock ri_lock(huch_ranger_mutex);
		switch(index) {
		case 0:
			DataCenter::huch_ranger.ranger1 = huch_ranger_a.ranger1;
			break;
		case 1:
			DataCenter::huch_ranger.ranger2 = huch_ranger_a.ranger2;
			break;
		case 2:
			DataCenter::huch_ranger.ranger3 = huch_ranger_a.ranger3;
			break;
		default:
			break;
		}
	}
	// status
	inline const mavlink_mk_fc_status_t DataCenter::get_mk_fc_status() {
		using namespace cpp_pthread;

		Lock ri_lock(mk_fc_mutex);
		mavlink_mk_fc_status_t mk_fc_status_copy(mk_fc_status);

		return mk_fc_status_copy;
	}
	inline void DataCenter::set_mk_fc_status(const mavlink_mk_fc_status_t &mk_fc_status_a) {
		using namespace cpp_pthread;

		Lock ri_lock(mk_fc_mutex);
		DataCenter::mk_fc_status = mk_fc_status_a;
	}
#endif // MAVLINK_ENABLED_HUCH
#endif // HAVE_MAVLINK_H

#ifdef HAVE_OPENCV2
// fiducal data
inline void DataCenter::set_fiducal_rot_raw(const cv::Mat &rvec) {
	using namespace cpp_pthread;

	Lock fiducal_lock(fiducal_raw_mutex);
	rvec.copyTo(fiducal_rot_raw);
}
inline void DataCenter::set_fiducal_trans_raw(const cv::Mat &tvec) {
	using namespace cpp_pthread;

	Lock fiducal_lock(fiducal_raw_mutex);
	tvec.copyTo(fiducal_rot_raw);
}
inline cv::Mat DataCenter::get_fiducal_rot_raw() {
	using namespace cpp_pthread;

	cv::Mat rvec;
	Lock fiducal_lock(fiducal_raw_mutex);
	fiducal_rot_raw.copyTo(rvec);
	return rvec;
}
inline cv::Mat DataCenter::get_fiducal_trans_raw(){
	using namespace cpp_pthread;

	cv::Mat tvec;
	Lock fiducal_lock(fiducal_raw_mutex);
	fiducal_rot_raw.copyTo(tvec);
	return tvec;
}
#endif // HAVE_OPENCV2

} // namespace mavhub

#endif
