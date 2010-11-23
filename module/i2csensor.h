#ifndef _I2CSENSOR_H_
#define _I2CSENSOR_H_

#include <exception>
#include <inttypes.h>

#include "thread.h"

/* output defines */
#define NONE	0x00
#define MAVHUB	0x01
#define DEBUG	0x02
#define TIMINGS	0x04

namespace mavhub {

	class I2cSensor : public cpp_pthread::PThread {
		public:
			I2cSensor();
			virtual ~I2cSensor();
//			virtual void print_CSV();
		protected:
			virtual void run() = 0;
			virtual void publish_data(uint64_t time) = 0;

			static void i2c_set_adr(const int fd, const int adr) throw(std::exception);
			// syncs 
			static pthread_mutex_t i2c_mutex;

		private:
			static int i2c_mutex_count;
	};
	// ----------------------------------------------------------------------------
	// I2cSensors
	// ----------------------------------------------------------------------------	
} // namespace mavhub

#endif
