#include <iostream> //cout
#include <cstring> //strcmp
#include <cstdlib> //exit

#include "logger.h"
#include "protocolstack.h"
#include "protocollayer.h"
#include "coreapp.h"
#include "datacenter.h"
#include "mavshell.h"

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
// 		uart = new UARTLayer("/dev/ttyUSB0");
	}
	catch(const char *message) {
		Logger::error(message);
		exit(-1);
	}
	try{
		udp = new UDPLayer(udp_port);
// 		udp->add_groupmember("127.0.0.1", 32001);
		udp->add_groupmember("127.0.0.1", 14550);
	}
	catch(const char *message) {
		Logger::error(message);
		exit(-1);
	}

	//create apps
// 	CoreApp *core_app = new CoreApp();

	//configure stack
	ProtocolStack::instance().setSystemID(system_id);
// 	ProtocolStack stack(system_id);
// 	stack.addInterface(udp , ProtocolStack::MAVLINKPACKAGE );
// 	stack.addInterface(udp , ProtocolStack::MKPACKAGE );
// 	stack.addInterface(uart , ProtocolStack::MKPACKAGE );
// 	ProtocolStack::instance().addInterface(uart , ProtocolStack::MAVLINKPACKAGE );
// 	stack.addApplication(core_app);

	//activate stack
// 	pthread_t stack_thread = ProtocolStack::instance().start();
	
	//start mav shell
	MAVShell *mav_shell = NULL;
	pthread_t shell_thread;
	
	try {
		mav_shell = new MAVShell(32001);
		shell_thread = mav_shell->start();
	}
	catch(const char* message) {
		cout << message << endl;
	}
	PThread::join(shell_thread);
	

// 	PThread::join(stack_thread);
}

void parse_argv(int argc, char **argv) {
	for(int i=1; i<argc; i++) {
		if( !strcmp(argv[i], "-p") || !strcmp(argv[i], "--port") ) {
			if(++i < argc) {
				udp_port = atoi(argv[i]);
			} else {
				cout << "ERROR: port argument missing" << endl;
				exit(-1);
			}
		} else 	if( !strcmp(argv[i], "-h") || !strcmp(argv[i], "--help") ) {
			print_help();
			exit(0);
		} else {
			cout << "ERROR: " << argv[i] << " is no valid argument" << endl;
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