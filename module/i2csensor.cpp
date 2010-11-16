#include "i2csensor.h"

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

} // namespace mavhub
