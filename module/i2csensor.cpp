//#include <linux/i2c.h>
//#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "i2csensor.h"
#include "logger.h"

using namespace std;

namespace mavhub {

//int I2cSensor::i2c_mutex_count = 0;
pthread_mutex_t I2cSensor::i2c_mutex = PTHREAD_MUTEX_INITIALIZER;

I2cSensor::I2cSensor()  {
//	if (!i2c_mutex_count) {
//		pthread_mutex_init( &i2c_mutex, NULL );
//		i2c_mutex_count++;
//	}
}

I2cSensor::~I2cSensor() {}

void I2cSensor::i2c_set_adr(const int fd, const int adr) throw(exception) {
#define I2C_SLAVE 0x0703
	if (ioctl(fd, I2C_SLAVE, adr) < 0) {
		Logger::warn("i2c: Failed to acquire bus access and/or talk to slave.");
		throw exception();
	}	
}


} // namespace mavhub
