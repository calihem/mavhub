#include "uart.h"

#include <fcntl.h> //open
#include <strings.h> //bzero
#include <unistd.h> //sleep

namespace mavhub {

  // std::ostream& operator <<(std::ostream &os, const dev_rate_t &dev_rate) {
  // 	 // Logger::log("dev_rate_t << exec");
  // 	 os << static_cast<int>(dev_rate);
  // 	 return os;
  // }

  // std::istream& operator >>(std::istream &is, dev_rate_t &dev_rate) {
  // 	 unsigned int tmp;
  // 	 is >> tmp;
  // 	 // Logger::log("dev_rate_t >> exec", tmp, Logger::LOGLEVEL_DEBUG);
  // 	 switch(tmp) {
  // 	 case 57600:
  // 		dev_rate = B57600;
  // 		break;
  // 	 case 230400:
  // 		dev_rate = B230400;
  // 		break;
  // 	 default:
  // 		dev_rate = B57600;
  // 		break;
  // 	 }
  // 	 return is;
  // }


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

  dev_rate_t UART::rate_rewrite(dev_rate_t rate) {
	 dev_rate_t dev_rate;
	 switch(rate) {
	 case 57600:
		dev_rate = B57600;
		break;
	 case 230400:
		dev_rate = B230400;
		break;
	 default:
		dev_rate = B57600;
	 }
	 return dev_rate;
  }
} // namespace mavhub
