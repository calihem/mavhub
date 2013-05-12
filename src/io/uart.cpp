#include "uart.h"

#include <fcntl.h> //open
#include <strings.h> //bzero
#include <unistd.h> //sleep
#include <sstream> // ostream >> int

namespace mavhub {

UART::UART(const std::string&  devicename, const tcflag_t control_modes) :
		IOInterface(devicename, "Serial Port"),
		control_modes(control_modes) {
	open();
}

UART::~UART() {
	close();
}

int UART::open() {
	if( !is_open() ) {
		fd = ::open(_name.c_str(), O_RDWR | O_NOCTTY);

		//save old settings
		tcgetattr(fd, &old_io_cfg);

		//reset new config struct
		bzero(&new_io_cfg, sizeof(new_io_cfg));
		// set control modes
		new_io_cfg.c_cflag = control_modes;

		//apply settings to serial interface
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &new_io_cfg);
                Logger::log("opened", _name, Logger::LOGLEVEL_DEBUG);
	}
	return fd;
}

void UART::close() {
	if( is_open() ) {
		//restore settings of serial interface
		tcsetattr(fd, TCSANOW, &old_io_cfg);
		::close(fd);
		fd = -1;
	}
}

const speed_t UART::baudrate_to_speed(const unsigned int baudrate) {

	switch(baudrate) {
		case 0:
			return B0;
		case 50:
			return B50;
		case 75:
			return B75;
		case 110:
			return B110;
		case 134:
			return B134;
		case 150:
			return B150;
		case 200:
			return B200;
		case 300:
			return B300;
		case 600:
			return B600;
		case 1200:
			return B1200;
		case 1800:
			return B1800;
		case 2400:
			return B2400;
		case 4800:
			return B4800;
		case 9600:
			return B9600;
		case 19200:
			return B19200;
		case 38400:
			return B38400;
		case 57600:
			return B57600;
		case 115200:
			return B115200;
		case 230400:
			return B230400;
		default:
			break;
	}

	return B0;
}

const unsigned int UART::speed_to_baudrate(const speed_t speed) {

	switch(speed) {
		case B0:
			return 0;
		case B50:
			return 50;
		case B75:
			return 75;
		case B110:
			return 110;
		case B134:
			return 134;
		case B150:
			return 150;
		case B200:
			return 200;
		case B300:
			return 300;
		case B600:
			return 600;
		case B1200:
			return 1200;
		case B1800:
			return 1800;
		case B2400:
			return 2400;
		case B4800:
			return 4800;
		case B9600:
			return 9600;
		case B19200:
			return 19200;
		case B38400:
			return 38400;
		case B57600:
			return 57600;
		case B115200:
			return 115200;
		case B230400:
			return 230400;
		default:
			break;
	}

	return 0;
}

} // namespace mavhub
