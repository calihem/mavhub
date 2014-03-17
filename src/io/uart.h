#ifndef _UART_H_
#define _UART_H_

#include "io/io.h"
#include "core/logger.h"

#include <termios.h>
#include <unistd.h> //read, write
#include <string>


namespace mavhub {

class UART : public cpp_io::IOInterface {
	public:
		UART();
		UART(const std::string&  devicename, const tcflag_t control_modes);
		~UART();

		virtual int open();
		virtual void close();
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
		/// control modes
		tcflag_t control_modes;
		/// config struct of serial interface
		struct termios old_io_cfg, new_io_cfg;
};
// ----------------------------------------------------------------------------
// UART
// ----------------------------------------------------------------------------

} // namespace mavhub

#endif
