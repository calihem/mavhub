#ifndef _I2CSENSOR_H_
#define _I2CSENSOR_H_

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
