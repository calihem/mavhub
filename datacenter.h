#ifndef _DATACENTER_H_
#define _DATACENTER_H_

#include "thread.h"
#include <mavlink.h>

namespace mavhub {

	class DataCenter {
		public:
			// raw imu 
			static const mavlink_raw_imu_t get_raw_imu();
			static void set_raw_imu(const mavlink_raw_imu_t &mavlink_raw_imu);
			// bmp085 pressure sensor
			static const mavlink_huch_bmp085_t get_bmp085();
			static void set_bmp085(const mavlink_huch_bmp085_t &bmp085);
			// hmc5843 kompass sensor
			static const mavlink_huch_hmc5843_t get_hmc5843();
			static void set_hmc5843(const mavlink_huch_hmc5843_t &hmc5843);
			// ExpCtrl sensor board
			/// Get ExpCtrl data
			static const mavlink_huch_exp_ctrl_rx_t get_exp_ctrl();
			/// Set ExpCtrl data
			static void set_exp_ctrl(const mavlink_huch_exp_ctrl_rx_t &exp_ctrl_rx_data);
			// FlightCtrl legacy
			/// get FC legacy data
			static const mavlink_huch_attitude_t get_huch_attitude();
			static const mavlink_huch_altitude_t get_huch_altitude();
			static const mavlink_huch_ranger_t get_huch_ranger();
			static const mavlink_mk_fc_status_t get_mk_fc_status();
			/// set FC legacy data
			static void set_huch_attitude(const mavlink_huch_attitude_t &huch_attitude);
			static void set_huch_altitude(const mavlink_huch_altitude_t &huch_altitude);
			static void set_huch_ranger(const mavlink_huch_ranger_t &huch_ranger);
			static void set_mk_fc_status(const mavlink_mk_fc_status_t &mk_fc_status);

		private:
			//data structs
			static mavlink_raw_imu_t raw_imu;
			static mavlink_raw_pressure_t raw_pressure;
// 			static mavlink_rc_channels_raw_t rc_channels_raw;

			//sync data
			static pthread_mutex_t raw_imu_mutex;
			
			// bmp085 pressure sensor
			static pthread_mutex_t bmp085_mutex;
			static mavlink_huch_bmp085_t bmp085_data;
			// hmc5843 pressure sensor
			static pthread_mutex_t hmc5843_mutex;
			static mavlink_huch_hmc5843_t hmc5843_data;
			// ExpCtrl sensor board
			/// ExpCtrl data access lock
			static pthread_mutex_t exp_ctrl_mutex;
			/// ExpCtrl data structure
			static mavlink_huch_exp_ctrl_rx_t exp_ctrl_rx_data;

			// FlightCtrl legacy
			static pthread_mutex_t mk_fc_mutex;
			static mavlink_huch_attitude_t huch_attitude;
			static mavlink_huch_altitude_t huch_altitude;
			static mavlink_huch_ranger_t huch_ranger;
			static mavlink_mk_fc_status_t mk_fc_status;

			
			DataCenter();
			DataCenter(const DataCenter &data);
			~DataCenter();
			void operator=(const DataCenter &data);
	};
	// ----------------------------------------------------------------------------
	// DataCenter
	// ----------------------------------------------------------------------------
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

	// BMP085 functions
	inline const mavlink_huch_bmp085_t DataCenter::get_bmp085() {
		using namespace cpp_pthread;

		Lock ri_lock(bmp085_mutex);
		mavlink_huch_bmp085_t bmp085_data_copy(bmp085_data);

		return bmp085_data_copy;
	}
	inline void DataCenter::set_bmp085(const mavlink_huch_bmp085_t &bmp085_data) {
		using namespace cpp_pthread;

		Lock ri_lock(bmp085_mutex);
		DataCenter::bmp085_data = bmp085_data;
	}

	// HMC5843 functions
	inline const mavlink_huch_hmc5843_t DataCenter::get_hmc5843() {
		using namespace cpp_pthread;

		Lock ri_lock(hmc5843_mutex);
		mavlink_huch_hmc5843_t hmc5843_data_copy(hmc5843_data);

		return hmc5843_data_copy;
	}
	inline void DataCenter::set_hmc5843(const mavlink_huch_hmc5843_t &hmc5843) {
		using namespace cpp_pthread;

		Lock ri_lock(hmc5843_mutex);
		DataCenter::hmc5843_data = hmc5843_data;
	}

	// ExpCtrl functions
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
	inline const mavlink_huch_altitude_t DataCenter::get_huch_altitude() {
		using namespace cpp_pthread;

		Lock ri_lock(mk_fc_mutex);
		mavlink_huch_altitude_t huch_altitude_copy(huch_altitude);

		return huch_altitude_copy;
	}
	inline void DataCenter::set_huch_altitude(const mavlink_huch_altitude_t &huch_altitude_a) {
		using namespace cpp_pthread;

		Lock ri_lock(mk_fc_mutex);
		DataCenter::huch_altitude = huch_altitude_a;
	}
	// ranger
	inline const mavlink_huch_ranger_t DataCenter::get_huch_ranger() {
		using namespace cpp_pthread;

		Lock ri_lock(mk_fc_mutex);
		mavlink_huch_ranger_t huch_ranger_copy(huch_ranger);

		return huch_ranger_copy;
	}
	inline void DataCenter::set_huch_ranger(const mavlink_huch_ranger_t &huch_ranger_a) {
		using namespace cpp_pthread;

		Lock ri_lock(mk_fc_mutex);
		DataCenter::huch_ranger = huch_ranger_a;
	}
	// ranger
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

} // namespace mavhub

#endif
