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

} // namespace mavhub

#endif
