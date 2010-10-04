#include "main.h"

#include <iostream> //cout
#include <cstring> //strcmp
#include <cstdlib> //exit

#include "logger.h"
#include "lib/setting.h"
#include "protocolstack.h"
#include "protocollayer.h"
#include "coremod.h"
#include "datacenter.h"
#include "mavshell.h"
#include "factory.h"

using namespace std;
using namespace mavhub;
using namespace cpp_pthread;
using namespace cpp_io;

uint8_t mavhub::system_id = 42;
int tcp_port = 30001;
string cfg_filename;

void parse_argv(int argc, char **argv);
void print_help();

int main(int argc, char **argv) {
	Logger::setLogLevel(Logger::LOGLEVEL_WARN);

	parse_argv(argc, argv);

	if( !cfg_filename.empty() ) {
		//open config file
		Setting settings(cfg_filename);
		//read and set loglevel
		Logger::log_level_t loglevel;
		if( settings.value("loglevel", loglevel) )
			Logger::log("Loglevel is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
		else
			Logger::setLogLevel(loglevel);
		if( settings.value("system_id", system_id) )
			Logger::log("System ID is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
		if( settings.value("tcp_port", tcp_port) )
			Logger::log("TCP port is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
	}

	//create media layers
	MediaLayer *uart = LinkFactory::build(LinkFactory::SerialLink, "/dev/ttyS1");
// 	MediaLayer *uart = LinkFactory::build(LinkFactory::SerialLink, "/dev/ttyUSB0");
	MediaLayer *udp = LinkFactory::build(LinkFactory::UDPLink, UDPLayer::DefaultPort);
	if(udp) {
		UDPLayer *udp_layer = dynamic_cast<UDPLayer*>(udp);
		if(udp_layer) {
			udp_layer->add_groupmember("127.0.0.1", 32001);
			udp_layer->add_groupmember("127.0.0.1", 14550);
			udp_layer->add_groupmember("127.0.0.1", 5000);
			udp_layer->add_groupmember("192.168.1.10", 5000);
		}
	}
 
	//create modules
// 	CoreModule *core_app = new CoreModule();

	//configure stack
	ProtocolStack::instance().system_id(system_id);
// 	ProtocolStack::instance().add_link(udp , ProtocolStack::MAVLINKPACKAGE );
	ProtocolStack::instance().add_link(udp , ProtocolStack::MKPACKAGE );
// 	stack.add_link(udp , ProtocolStack::MKPACKAGE );
// 	stack.add_link(uart , ProtocolStack::MKPACKAGE );
// 	ProtocolStack::instance().add_link(uart , ProtocolStack::MAVLINKPACKAGE );
	ProtocolStack::instance().add_link(uart , ProtocolStack::MKPACKAGE );
// 	stack.add_application(core_app);

	//activate stack
	pthread_t stack_thread = ProtocolStack::instance().start();
	
	//start mav shell
	try {
		MAVShell *mav_shell = new MAVShell(32001);
		pthread_t shell_thread = mav_shell->start();
		PThread::join(shell_thread);
	}
	catch(const char* message) {
		cout << message << endl;
	}

	//join threads
	PThread::join(stack_thread);
}

void parse_argv(int argc, char **argv) {
	for(int i=1; i<argc; i++) {
		if( !strcmp(argv[i], "-c") || !strcmp(argv[i], "--config") ) {
			if(++i < argc) {
				cfg_filename = argv[i];
			} else {
				cout << "ERROR: file argument missing" << endl;
				exit(-1);
			}
		} else 	if( !strcmp(argv[i], "-h") || !strcmp(argv[i], "--help") ) {
			print_help();
			exit(0);
		} else if( !strcmp(argv[i], "-p") || !strcmp(argv[i], "--port") ) {
			if(++i < argc) {
				tcp_port = atoi(argv[i]);
			} else {
				cout << "ERROR: port argument missing" << endl;
				exit(-1);
			}
		} else {
			cout << "ERROR: " << argv[i] << " is no valid argument" << endl;
		}
	}
}

void print_help() {
	cout << "NAME" << endl;
	cout << "\t" << "MAVHUB - Micro Air Vehicle HUB" << endl << endl;
	cout << "SYNOPSIS" << endl;
	cout << "\t" << "mavhub [-h]" << endl << endl;
	cout << "OPTIONS" << endl;
	cout << "\t-c <file> or --config <file>" << endl;
	cout << "\t\t" << "open config file <file>" << endl;
	cout << "\t-h or --help" << endl;
	cout << "\t\t" << "print usage summary" << endl;
	cout << "\t-p <port> or --port <port>" << endl;
	cout << "\t\t" << "set port of management console" << endl;
}
