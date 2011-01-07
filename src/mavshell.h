#ifndef _MAVSHELL_H_
#define _MAVSHELL_H_

#include "core/thread.h"
#include "io/network.h"

#include <inttypes.h> //uint8_t
#include <sstream> //stringstream
#include <vector>

namespace mavhub {

	class MAVShell : public cpp_pthread::PThread {
		public:
			MAVShell(uint16_t port = 32001) throw(const char*);
			~MAVShell();

		protected:
			virtual void run();


		private:
			/// size of buffer
			static const int BUFFERLENGTH = 512;
			MAVShell(const MAVShell &); // intentionally undefined
			MAVShell& operator=(const MAVShell &); // intentionally undefined
			
			TCPServerSocket server_socket;
			TCPSocket *client_socket;
			std::stringstream input_stream;

			void handle_client();
			void send_message(const std::string& msg);
			void tokenize_stream(std::stringstream& stream);
			void execute_cmd(const std::vector<std::string>& argv);
			void send_help();
			void welcome();

	
	};
	// ----------------------------------------------------------------------------
	// MAVShell
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif
