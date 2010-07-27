#include <iostream> //cout
#include <cstring> //strcmp
#include <cstdlib> //exit

#include "logger.h"
#include "protocolstack.h"
#include "protocollayer.h"

using namespace std;
using namespace mavhub;

uint8_t system_id = 42;
int udp_port = UDPLayer::DefaultPort;
int tcp_port = 30001;

void parse_argv(int argc, char **argv);
void print_help();

int main(int argc, char **argv) {
	Logger::setLogLevel(Logger::LOGLEVEL_ALL);
	parse_argv(argc, argv);

	//create media layers
	UARTLayer *uart;
	UDPLayer *udp;
	try{
		uart = new UARTLayer("/dev/ttyS0");
	}
	catch(const char *message) {
		Logger::error(message);
		exit(-1);
	}
	try{
		udp = new UDPLayer(udp_port);
	}
	catch(const char *message) {
		Logger::error(message);
		exit(-1);
	}

	//configure stack
	ProtocolStack stack(system_id);
	stack.addInterface(udp , ProtocolStack::MAVLINKPACKAGE );
	stack.addInterface(uart , ProtocolStack::MKPACKAGE );

	//activate stack
	pthread_t stack_thread = stack.start();

	PThread::join(stack_thread);
}

void parse_argv(int argc, char **argv) {
	for(int i=1; i<argc; i++) {
		if( !strcmp(argv[i], "-p") || !strcmp(argv[i], "--port") ) {
			//FIXME: check i++ < argc
			udp_port = atoi(argv[i++]);
		} else 	if( !strcmp(argv[i], "-h") || !strcmp(argv[i], "--help") ) {
			print_help();
			exit(0);
		} else {
			cout << argv[i] << " is no valid argument" << endl;
		}
	}
}

void print_help() {
	cout << "NAME" << endl;
	cout << "\t" << "MAVHUB - Micro Air Vehicle HUB" << endl << endl;
	cout << "SYNOPSIS" << endl;
	cout << "\t" << "yarc [-h]" << endl << endl;
	cout << "OPTIONS" << endl;
	cout << "\t-h or --help" << endl;
	cout << "\t\t" << "print usage summary" << endl;
}