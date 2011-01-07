//#include <linux/i2c.h>
//#include <linux/i2c-dev.h>
#include <fcntl.h> //open
#include <sys/ioctl.h> //ioctl

#include "i2csensor.h"
#include "core/logger.h"

using namespace std;

namespace mavhub {

std::map<const int, pthread_mutex_t> I2cSensor::mutex_map;
std::map<const std::string, const int> I2cSensor::port_map;

I2cSensor::I2cSensor()  {
}

I2cSensor::~I2cSensor() {}

const int I2cSensor::i2c_init(const string port) throw(const char *) {
	std::map<const string, const int>::const_iterator iter;
	int _fd;

	if ((iter = port_map.find(port)) == port_map.end()) {
		if ((_fd = open(port.c_str(), O_RDWR)) < 0) {
			throw "i2c_init(): Failed to open the i2c bus";	
		} else {
			port_map.insert(pair<const string, const int>(port,_fd));
			pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
			mutex_map.insert(pair<const int, pthread_mutex_t>(_fd,mutex));
		}
	} 

	return port_map[port];
}

void I2cSensor::i2c_start_conversion(const int fd, const int adr) throw(const char *) {
	pthread_mutex_lock( &mutex_map[fd] );
	
	#define I2C_SLAVE 0x0703
	if (ioctl(fd, I2C_SLAVE, adr) < 0) {
		throw "i2c: Failed to acquire bus access and/or talk to slave.";
	}	
}

void I2cSensor::i2c_end_conversion(const int fd) {
	pthread_mutex_unlock( &mutex_map[fd] );
}

void I2cSensor::i2c_write_bytes(const int fd, uint8_t *buffer, int size) throw(const char *) {
	if (write(fd, buffer, size) != size) {
		throw "i2c: Failed to write to slave!";
	}
}

void I2cSensor::i2c_read_bytes(const int fd, uint8_t *buffer, int size) throw(const char *) {
	if (read(fd, buffer, size) != size) {
		throw "i2c: Failed to read to slave!";
	}
}

} // namespace mavhub
