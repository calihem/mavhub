#ifndef _DATACENTER_H_
#define _DATACENTER_H_

#include "core/thread.h"
#include <mavlink.h>

namespace mavhub {

	class DataCenter {
		public:
			// raw imu 
			static const mavlink_raw_imu_t get_raw_imu();
			static void set_raw_imu(const mavlink_raw_imu_t &mavlink_raw_imu);
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
			/**
			 * Raw pressure
			 */
			static void set_raw_pressure(const mavlink_raw_pressure_t &raw_pressure);
			static const mavlink_raw_pressure_t get_raw_pressure();

			/// Set ExpCtrl data
			static void set_exp_ctrl(const mavlink_huch_exp_ctrl_rx_t &exp_ctrl_rx_data);

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
			/// set FC legacy extctrl components
			static void set_extctrl_nick(const int16_t nick);
			static void set_extctrl_roll(const int16_t roll);
			static void set_extctrl_yaw(const int16_t yaw);
			static const int16_t get_extctrl_nick();
			static const int16_t get_extctrl_roll();
			static const int16_t get_extctrl_yaw();

		private:
			//data structs
			static mavlink_raw_imu_t raw_imu;
			static mavlink_huch_imu_raw_adc_t huch_imu_raw_adc;
			static mavlink_huch_mk_imu_t huch_mk_imu;
			static mavlink_raw_pressure_t raw_pressure;

			//sync data
			static pthread_mutex_t raw_imu_mutex;
			static pthread_mutex_t exp_ctrl_mutex;
			static pthread_mutex_t huch_imu_raw_adc_mutex;
			static pthread_mutex_t huch_mk_imu_mutex;
			static pthread_mutex_t raw_pressure_mutex;

			/// ExpCtrl data structure
			static mavlink_huch_exp_ctrl_rx_t exp_ctrl_rx_data;

			// FlightCtrl legacy
			static pthread_mutex_t mk_fc_mutex;
			static pthread_mutex_t huch_ranger_mutex;
			static mavlink_huch_attitude_t huch_attitude;
			static mavlink_huch_fc_altitude_t huch_altitude;
			static mavlink_huch_ranger_t huch_ranger;
			static mavlink_mk_fc_status_t mk_fc_status;
			static int16_t extctrl_nick;
			static int16_t extctrl_roll;
			static int16_t extctrl_yaw;
			static pthread_mutex_t extctrl_mutex;
			
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

	// extctrl component setters
	inline void DataCenter::set_extctrl_nick(const int16_t nick) {
		using namespace cpp_pthread;

		Lock ri_lock(extctrl_mutex);
		extctrl_nick = nick;
	}
	inline void DataCenter::set_extctrl_roll(const int16_t roll) {
		using namespace cpp_pthread;

		Lock ri_lock(extctrl_mutex);
		extctrl_roll = roll;
	}
	inline void DataCenter::set_extctrl_yaw(const int16_t yaw) {
		using namespace cpp_pthread;

		Lock ri_lock(extctrl_mutex);
		extctrl_yaw = yaw;
	}
	
	// extctrl component getters
	inline const int16_t DataCenter::get_extctrl_nick() {
		using namespace cpp_pthread;

		Lock ri_lock(extctrl_mutex);
		return extctrl_nick;
	}
	inline const int16_t DataCenter::get_extctrl_roll() {
		using namespace cpp_pthread;

		Lock ri_lock(extctrl_mutex);
		return extctrl_roll;
	}
	inline const int16_t DataCenter::get_extctrl_yaw() {
		using namespace cpp_pthread;

		Lock ri_lock(extctrl_mutex);
		return extctrl_yaw;
	}


} // namespace mavhub

#endif
