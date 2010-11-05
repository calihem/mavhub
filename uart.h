#ifndef _UART_H_
#define _UART_H_

#include "logger.h"
#include "utility.h"
#include <termios.h>
#include <unistd.h> //read, write
#include <string>


namespace mavhub {

  typedef unsigned int dev_rate_t;
  /* enum dev_rate_t { */
  /* 	 kb57, */
  /* 	 kb230 */
  /* }; */
  // std::ostream& operator <<(std::ostream &os, const dev_rate_t &dev_rate);
  // std::istream& operator >>(std::istream &is, dev_rate_t &dev_rate);
  

  class UART {
  public:

			UART(const std::string&  devicename, tcflag_t control_modes) throw(const char*);
			~UART();

			static dev_rate_t rate_rewrite(dev_rate_t rate);

			void enable_blocking_mode(bool enabled);
			ssize_t read(void *buf, size_t nbyte) const;
			ssize_t write(const void *buf, size_t nbyte) const;


		protected:

		private:
			/// filedescriptor for serial interface
			int sifd;
			/// config struct of serial interface
			struct termios old_io_cfg, new_io_cfg;

	};
	// ----------------------------------------------------------------------------
	// UART
	// ----------------------------------------------------------------------------
	inline void UART::enable_blocking_mode(bool enabled) {
		if( ::mavhub::enable_blocking_mode(sifd, enabled) != 0 ) {
			Logger::log("setting of blocking mode for uart failed", Logger::LOGLEVEL_ERROR);
		}
	}
	inline ssize_t UART::read(void *buf, size_t nbyte) const {
		return ::read(sifd, buf, nbyte);
	}
	inline ssize_t UART::write(const void *buf, size_t nbyte) const {
		return ::write(sifd, buf, nbyte);
	}


} // namespace mavhub

#endif
