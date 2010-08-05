#ifndef _DATACENTER_H_
#define _DATACENTER_H_

#include <pthread.h>
#include <mavlink.h>

namespace mavhub {

	class DataCenter {
		public:
			static const mavlink_raw_imu_t& get_raw_imu();
			static void set_raw_imu(const mavlink_raw_imu_t &mavlink_raw_imu);

		private:
			//data structs
			static mavlink_raw_imu_t raw_imu;
			static mavlink_raw_pressure_t raw_pressure;
			static mavlink_rc_channels_raw_t rc_channels_raw;

			//sync data
			static pthread_mutex_t raw_imu_mutex;
			
			DataCenter();
			DataCenter(const DataCenter &data);
			~DataCenter();
			void operator=(const DataCenter &data);
	};
	// ----------------------------------------------------------------------------
	// DataCenter
	// ----------------------------------------------------------------------------
	inline const mavlink_raw_imu_t& DataCenter::get_raw_imu() {
		return raw_imu;
	}
	inline void DataCenter::set_raw_imu(const mavlink_raw_imu_t &mavlink_raw_imu) {
		pthread_mutex_lock(&raw_imu_mutex);
		raw_imu = mavlink_raw_imu;
		pthread_mutex_unlock(&raw_imu_mutex);
	}

} // namespace mavhub

#endif