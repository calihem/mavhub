#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "thread.h"

// sensor typ definitions
#define ALTITUDE_SENSOR		0x115
#define KOMPASS_SENSOR		0x116
#define TEMPERATURE_SENSOR	0x121
#define PRESSURE_SENSOR		0x122
#define DISTANCE_SENSOR		0x132

namespace mavhub {

	class Sensor : public cpp_pthread::PThread {
		public:
			Sensor();
			virtual ~Sensor();
			virtual void* get_data_pointer(unsigned int id) throw(const char *) = 0;
			enum statusT {UNINITIALIZED, STOPPED, RUNNING, STRANGE};
			statusT get_status();
			void stop();
			pthread_mutex_t &get_data_mutex();
		protected:
			statusT status;

			pthread_mutex_t data_mutex;

			unsigned short dev_id;
			unsigned short func_id;
			int update_rate;
			int debug;
			int timings;
		private:

	};
	// ----------------------------------------------------------------------------
	// Sensor
	// ----------------------------------------------------------------------------	
} // namespace mavhub

#endif
