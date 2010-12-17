#ifndef _I2CSENSOR_H_
#define _I2CSENSOR_H_

#include <string>
#include <map>
#include <inttypes.h>

#include "sensor.h"

//FIXME die müssen hier noch weg
/* output defines */
#define NONE	0x00
#define MAVHUB	0x01
#define DEBUG	0x02
#define TIMINGS	0x04

namespace mavhub {

	class I2cSensor : public Sensor {
		public:
			I2cSensor();
			virtual ~I2cSensor();
		protected:
			static const int i2c_init(const std::string port) throw(const char *);
			static void i2c_start_conversion(const int fd, const int adr) throw(const char *);
			static void i2c_end_conversion(const int fd);
			static void i2c_write_bytes(const int fd, uint8_t *buffer, int size) throw(const char *);
			static void i2c_read_bytes(const int fd, uint8_t *buffer, int size) throw(const char *);
			
			int fd;
			int adr;
			pthread_mutex_t *mutex_ptr;
		private:
			// port name fd mapping			
			static std::map<const std::string , const int> port_map;
			// syncs 
			static std::map<const int, pthread_mutex_t> mutex_map;

	};
	// ----------------------------------------------------------------------------
	// I2cSensors
	// ----------------------------------------------------------------------------	
} // namespace mavhub

#endif
