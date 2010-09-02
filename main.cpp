#include "main.h"

#include <iostream> //cout
#include <cstring> //strcmp
#include <cstdlib> //exit

#include "logger.h"
#include "protocolstack.h"
#include "protocollayer.h"
#include "coremod.h"
#include "datacenter.h"
#include "mavshell.h"
#include "factory.h"

using namespace std;
using namespace mavhub;

uint8_t mavhub::system_id = 42;
int udp_port = UDPLayer::DefaultPort;
int tcp_port = 30001;

void parse_argv(int argc, char **argv);
void print_help();

int main(int argc, char **argv) {
	Logger::setLogLevel(Logger::LOGLEVEL_ALL);
	parse_argv(argc, argv);

	//create media layers
	MediaLayer *uart = LinkFactory::build(LinkFactory::SerialLink, "/dev/ttyS0");
// 	MediaLayer *uart = LinkFactory::build(LinkFactory::SerialLink, "/dev/ttyUSB0");
	MediaLayer *udp = LinkFactory::build(LinkFactory::UDPLink, udp_port);
	if(udp) {
		UDPLayer *udp_layer = dynamic_cast<UDPLayer*>(udp);
		if(udp_layer) {
			udp_layer->add_groupmember("127.0.0.1", 32001);
			udp_layer->add_groupmember("127.0.0.1", 14550);
		}
	}
 
	//create modules
// 	CoreModule *core_app = new CoreModule();

	//configure stack
	ProtocolStack::instance().system_id(system_id);
	ProtocolStack::instance().add_link(udp , ProtocolStack::MAVLINKPACKAGE );
// 	stack.add_link(udp , ProtocolStack::MKPACKAGE );
// 	stack.add_link(uart , ProtocolStack::MKPACKAGE );
	ProtocolStack::instance().add_link(uart , ProtocolStack::MAVLINKPACKAGE );
// 	stack.add_application(core_app);

	//activate stack
	pthread_t stack_thread = ProtocolStack::instance().start();
	
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

	//join threads
	PThread::join(shell_thread);
	PThread::join(stack_thread);
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
