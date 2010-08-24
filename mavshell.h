#ifndef _MAVSHELL_H_
#define _MAVSHELL_H_

#include "thread.h"
#include <inttypes.h> //uint8_t
#include "network.h"

namespace mavhub {

	class MAVShell : public PThread {
		public:
			MAVShell(uint16_t port = 32001) throw(const char*);
			~MAVShell();

		protected:
			virtual void run();


		private:
			/// size of buffer
			static const int BUFFERLENGTH = 512;
			static const char *WELCOMEMESSAGE;
			MAVShell(const MAVShell &); // intentionally undefined
			MAVShell& operator=(const MAVShell &); // intentionally undefined
			
			TCPServerSocket server_socket;
			TCPSocket *client_socket;

			void handle_client();
			void send_message(const std::string& msg);
			void parse_stream(const char *stream, int stream_len);
			void execute_cmd(const char **argv);
			void welcome();

	
	};
	// ----------------------------------------------------------------------------
	// MAVShell
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif
