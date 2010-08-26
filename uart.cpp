#include "uart.h"

#include <fcntl.h> //open
#include <strings.h> //bzero
#include <unistd.h> //sleep

namespace mavhub {

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
