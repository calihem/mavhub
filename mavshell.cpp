#include "mavshell.h"

#include "logger.h"
#include "utility.h"

#include <iostream> //cout
using namespace std;

namespace mavhub {

const char* MAVShell::WELCOMEMESSAGE = "Welcome on mavhub";

MAVShell::MAVShell(uint16_t port) throw(const char*) :
		server_socket(port, 1),
		client_socket(NULL) {
}

MAVShell::~MAVShell() {
}

void MAVShell::run() {

	try {
		while(1) {
			client_socket = server_socket.accept();
			handle_client();
		}
	}
	catch(const char *message) {
		cout << message << endl;
	}
}

void MAVShell::handle_client() {
	if(!client_socket) return;

	//send welcome message
	welcome();

	int received;
	char rx_buffer[BUFFERLENGTH];

	while( (received = client_socket->receive(rx_buffer, BUFFERLENGTH)) > 0 ) {
		Logger::log("received", received, "bytes", Logger::LOGLEVEL_DEBUG);
		parse_stream(rx_buffer, received);
	}

	client_socket = NULL;
}

void MAVShell::send_message(const std::string& msg) {
	if(!client_socket) return;

	client_socket->send(msg.c_str(), msg.size());
	client_socket->send("\n\r> ", 4);
}

void MAVShell::parse_stream(const char *stream, int stream_len) {
	
}

void MAVShell::execute_cmd(const char **argv) {
	
}

void MAVShell::welcome() {
	std::string welcome_str(WELCOMEMESSAGE);

	send_message(welcome_str);
}

} // namespace mavhub
