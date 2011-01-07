#include "mavshell.h"

#include "core/logger.h"
#include "core/protocolstack.h"
#include "utility.h"
#include "factory.h"

#include <iterator> //istream_iterator
#include <cstdlib> //exit
#include <stdexcept> // out_of_range exception

#include <iostream> //cout
using namespace std;

namespace mavhub {

MAVShell::MAVShell(uint16_t port) throw(const char*) :
		server_socket(port, 1),
		client_socket(NULL),
		input_stream(ios_base::out|ios_base::in) {
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

	string remote_addr;
	uint16_t remote_port = 0;
	try {
		remote_addr = client_socket->foreign_addr();
		remote_port = client_socket->foreign_port();
		Logger::log(remote_addr, remote_port, "connected", Logger::LOGLEVEL_INFO);
	}
	catch(const char *message) {
		Logger::log(message);
	}

	//send welcome message
	welcome();

	int received;
	char rx_buffer[BUFFERLENGTH];

	while( (received = client_socket->receive(rx_buffer, BUFFERLENGTH)) > 0 ) {
		Logger::log("MAVShell received", received, "bytes", Logger::LOGLEVEL_DEBUG);
		// append rx_buffer to input_stream
		input_stream.write(rx_buffer, received);
		// append rx_buffer to input_stream but without '\r'
// 		for(int i=0; i<received; i++) {
// 			if (rx_buffer[i] != '\r')
// 				input_stream.put(rx_buffer[i]);
// 		}

		tokenize_stream(input_stream);
	}

	Logger::log(remote_addr, remote_port, "disconnected", Logger::LOGLEVEL_INFO);
	client_socket = NULL;
}

void MAVShell::send_message(const std::string& msg) {
	if(!client_socket) return;

	client_socket->send(msg.c_str(), msg.size());
	client_socket->send("\n\r> ", 4);
}

void MAVShell::tokenize_stream(std::stringstream& sstream) {
	string cmd_line;
	//get command line (everything up to '\n')
	while( getline(sstream, cmd_line) ) {
		istringstream cmd_stream(cmd_line);

		// copy cmd_line to vector as whitespace separated strings
		istream_iterator<string> begin(cmd_stream);
		istream_iterator<string> end;
		vector<string> argv(begin, end);

		execute_cmd(argv);
	}
	// we have to clear the stringstream, otherwise all operations on the stream will fail instantly
	sstream.clear();
}

void MAVShell::execute_cmd(const std::vector<std::string>& argv) {
	ostringstream send_stream;

	for(unsigned int i=0; i<argv.size(); i++) {
		if(argv.at(i).compare("addmember") == 0) {
			try{
				//get link belonging to ID
				istringstream idstream( argv.at(i+1) );
				int id;
				idstream >> id;
				cpp_io::IOInterface *link = ProtocolStack::instance().link(id);
				if(!link) {
					send_stream << "No link with ID " << id << " available" << endl;
					continue;
				}
				UDPLayer *udp_layer = dynamic_cast<UDPLayer*>(link);
				if(!udp_layer) {
					send_stream << "Link with ID " << id << " is no UDP Port" << endl;
					continue;
				}
				// get port number
				istringstream portstream( argv.at(i+3) );
				uint16_t port;
				portstream >> port;
				// add member to group
				udp_layer->add_groupmember(argv.at(i+2), port);
				i += 3;
			}
			catch(std::out_of_range& e) {
				send_stream << "Not enough arguments given for addmember" << endl;
			}
		} else if( (argv.at(i).compare("close") == 0) 
		|| (argv.at(i).compare("exit") == 0) ) {
			//close connection
			if(!client_socket) return;
			client_socket->disconnect();
			return;
		} else if(argv.at(i).compare("help") == 0) {
			send_help();
		} else if(argv.at(i).compare("ifdown") == 0) {
			try {
				i++;
				istringstream istream( argv.at(i) );
				int id;
				istream >> id;
				if( ProtocolStack::instance().remove_link(id) != 0 ) {
					send_stream << "Removing of link with ID " << id << " failed" << endl;
				} else {
					send_stream << "Removed link with ID " << id << endl;
				}
			}
			catch(std::out_of_range& e) {
				send_stream << "ID argument is missing" << endl;
			}
		} else if(argv.at(i).compare("iflist") == 0) {
			try {
				istringstream istream( argv.at(i+1) );
				int id;
				istream >> id;
				cpp_io::IOInterface *link = ProtocolStack::instance().link(id);
				if(!link) {
					send_stream << "No link with ID " << id << " available" << endl;
					send_stream << ProtocolStack::instance().link_list();
				} else {
					send_stream << *link;
					i++;
				}
			}
			catch(std::out_of_range& e) {
					send_stream << ProtocolStack::instance().link_list();
			}
		} else if(argv.at(i).compare("ifup") == 0) {
			try {
				cpp_io::IOInterface *link = LinkFactory::build(argv.at(i+1), argv.at(i+2));
				istringstream istream( argv.at(i+3) );
				ProtocolStack::packageformat_t format;
				istream >> format;
				int add_failed = ProtocolStack::instance().add_link(link, format);
				if(add_failed) {
					send_stream << "Bringing interface up failed" << endl;
				}
				i += 3;
			}
			catch(std::out_of_range& e) {
				send_stream << "Not enough arguments given for ifup" << endl;
			}
		} else if(argv.at(i).compare("applist") == 0) {
			try {
				istringstream istream( argv.at(i+1) );
				int id;
				istream >> id;
				const AppLayer *app = ProtocolStack::instance().application(id);
				if(!app) {
					send_stream << "No application with ID " << id << " available" << endl;
					send_stream << ProtocolStack::instance().application_list();

				} else {
					send_stream << *app;
					i++;
				}
			}
			catch(std::out_of_range& e) {
					send_stream << ProtocolStack::instance().application_list();
			}
		} else if(argv.at(i).compare("loglevel") == 0) {
			try {
				i++;
				istringstream istream( argv.at(i) );
				Logger::log_level_t loglevel;
				istream >> loglevel;
				Logger::setLogLevel(loglevel);
				send_stream << "Loglevel is now: " << Logger::loglevel() << endl;
			}
			catch(std::out_of_range& e) {
				send_stream << "Loglevel is: " << Logger::loglevel() << endl;
			}
		} else if(argv.at(i).compare("print") == 0) {
			send_stream << ProtocolStack::instance();
		} else if( (argv.at(i).compare("quit") == 0)
		|| (argv.at(i).compare("shutdown") == 0) ) {
			// exit whole process - the easy way
			exit(0);
		} else {
			send_stream << "\"" << argv.at(i) << "\" is no valid argument" << endl;
		}
	}

	send_message(send_stream.str());
}

void MAVShell::send_help() {
	if(!client_socket) return;

	ostringstream help_stream;
	help_stream << "addmember linkID IP port" << endl
		<< "\t" << "add group member to broadcast group of UDPLayer" << endl
		<< "applist [appID]" << endl
		<< "\t" << "list all applications (or those with appID) registered with protocol stack" << endl
		<< "close | exit" << endl
		<< "\t" << "close connection to mavhub" << endl
		<< "help" << endl
		<< "\t" << "print usage summary" << endl
		<< "ifdown linkID" << endl
		<< "\t" << "remove device with id" << endl
		<< "iflist [linkID]" << endl
		<< "\t" << "list all devices or device with linkID" << endl
		<< "ifup type device protocol" << endl
		<< "\t" << "add device with given protocol" << endl
		<< "loglevel [0...6]" << endl
		<< "\t" << "get/set loglevel" << endl
		<< "print" << endl
		<< "\t" << "print overview" << endl
		<< "quit | shutdown" << endl
		<< "\t" << "stop mavhub" << endl
		;

	client_socket->send(help_stream.str().c_str(), help_stream.str().size() );
}

void MAVShell::welcome() {
	std::string welcome_str("Welcome on mavhub");

	send_message(welcome_str);
}

} // namespace mavhub
