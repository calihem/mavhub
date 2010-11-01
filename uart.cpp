#include "uart.h"

#include <fcntl.h> //open
#include <strings.h> //bzero
#include <unistd.h> //sleep
#include <sstream> // ostream >> int

namespace mavhub {

const unsigned int baudrate_t::numeric() const {

	switch(baudrate) {
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

std::ostream& operator <<(std::ostream &os, const baudrate_t &baudrate) {
	os << baudrate.numeric();
	return os;
}

std::istream& operator >>(std::istream &is, baudrate_t &baudrate) {
	int tmp;
	is >> tmp;

	switch(tmp) {
		case 0:
			baudrate.baudrate = B0;
			break;
		case 50:
			baudrate.baudrate = B50;
			break;
		case 75:
			baudrate.baudrate = B75;
			break;
		case 110:
			baudrate.baudrate = B110;
			break;
		case 134:
			baudrate.baudrate = B134;
			break;
		case 150:
			baudrate.baudrate = B150;
			break;
		case 200:
			baudrate.baudrate = B200;
			break;
		case 300:
			baudrate.baudrate = B300;
			break;
		case 600:
			baudrate.baudrate = B600;
			break;
		case 1200:
			baudrate.baudrate = B1200;
			break;
		case 1800:
			baudrate.baudrate = B1800;
			break;
		case 2400:
			baudrate.baudrate = B2400;
			break;
		case 4800:
			baudrate.baudrate = B4800;
			break;
		case 9600:
			baudrate.baudrate = B9600;
			break;
		case 19200:
			baudrate.baudrate = B19200;
			break;
		case 38400:
			baudrate.baudrate = B38400;
			break;
		case 57600:
			baudrate.baudrate = B57600;
			break;
		case 115200:
			baudrate.baudrate = B115200;
			break;
		case 230400:
			baudrate.baudrate = B230400;
			break;
		default:
			break;
	}

	 return is;
}

UART::UART(const std::string&  devicename, tcflag_t control_modes) throw(const char*) {
	if( (sifd = open(devicename.c_str(), O_RDWR | O_NOCTTY)) < 0 ) {
		throw "Can't open serial interface device";
		return;
	}

	//save old settings
	tcgetattr(sifd, &old_io_cfg);

	//reset new config struct
	bzero(&new_io_cfg, sizeof(new_io_cfg));
	// set control modes
	new_io_cfg.c_cflag = control_modes;

	//apply settings to serial interface
	tcflush(sifd, TCIFLUSH);
	tcsetattr(sifd, TCSANOW, &new_io_cfg);
}

UART::~UART() {
	//restore settings of serial interface
 	tcsetattr(sifd, TCSANOW, &old_io_cfg);
	//close file
	close(sifd);
}

} // namespace mavhub
