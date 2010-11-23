#ifndef _UART_H_
#define _UART_H_

#include "lib/io.h"
#include "logger.h"
#include "utility.h"

#include <termios.h>
#include <unistd.h> //read, write
#include <string>


namespace mavhub {

class UART : public cpp_io::IOInterface {
	public:
		UART(const std::string&  devicename, tcflag_t control_modes) throw(const char*);
		~UART();

		/**
		 * \brief converts numeric baudrate to speed type needed for termios structure 
		 */
		static const speed_t baudrate_to_speed(const unsigned int baudrate);
		/**
		 * \brief converts speed_t used in termios structure tcflag_t to numeric baudrate
		 */
		static const unsigned int speed_to_baudrate(const speed_t speed);

	protected:

	private:
		/// config struct of serial interface
		struct termios old_io_cfg, new_io_cfg;
};
// ----------------------------------------------------------------------------
// UART
// ----------------------------------------------------------------------------

} // namespace mavhub

#endif
