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
string cfg_filename("mavhub.conf");

void parse_argv(int argc, char **argv);
void print_help();

int main(int argc, char **argv) {
	Logger::setLogLevel(Logger::LOGLEVEL_WARN);

	parse_argv(argc, argv);

	//open config file
	Setting settings(cfg_filename, std::ios_base::in);
	//read loglevel
	Logger::log_level_t loglevel;
	if( settings.value("loglevel", loglevel) )
		Logger::log("Loglevel is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
	else
		Logger::setLogLevel(loglevel);
	//read system ID
	if( settings.value("system_id", system_id) )
		Logger::log("System ID is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
	//read TCP port of MAVShell
	if( settings.value("tcp_port", tcp_port) )
		Logger::log("TCP port is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);

	//basic stack configuration
	ProtocolStack::instance().system_id(system_id);

	/*
	* create media layers
	*/
	if( settings.begin_group("serial_link") == 0) { //serial link group available
		string dev_name;
		if( settings.value("name", dev_name) ) {
			Logger::log("Device name is missing in config file:", cfg_filename, "for serial link", Logger::LOGLEVEL_WARN);
		} else {
			baudrate_t baudrate(B57600);
			if( settings.value("baudrate", baudrate) ) {
				Logger::log("Baudrate is missing for device:", dev_name, Logger::LOGLEVEL_WARN);
			}

			MediaLayer *uart = LinkFactory::build(LinkFactory::SerialLink, dev_name, baudrate);

			ProtocolStack::packageformat_t package_format;
			if( settings.value("protocol", package_format) ) {
				Logger::log("Protocol is missing in config file:", cfg_filename, "for serial link", Logger::LOGLEVEL_WARN);
				package_format = ProtocolStack::MAVLINKPACKAGE;
			}

			ProtocolStack::instance().add_link(uart, package_format);
		}
		settings.end_group();
	}

	if( settings.begin_group("UDP_link") == 0) { //UDP link group available
		//read udp port
		uint16_t udp_port;
		if( settings.value("port", udp_port) ) {
			Logger::log("Port is missing in config file:", cfg_filename, "for UDP link", Logger::LOGLEVEL_WARN);
			udp_port = UDPLayer::DefaultPort;
		}
		MediaLayer *udp = LinkFactory::build(LinkFactory::UDPLink, udp_port);
		if(udp) {
			UDPLayer *udp_layer = dynamic_cast<UDPLayer*>(udp);
			if(udp_layer) {
				// read udp group members
				std::list<string_addr_pair_t> groupmember_list;
				settings.value("members", groupmember_list);
				try{
					udp_layer->add_groupmembers(groupmember_list);
				}
				catch(const char *message) {
					Logger::log(message, Logger::LOGLEVEL_DEBUG);
				}
			}
		}
		//read package format
		ProtocolStack::packageformat_t package_format;
		if( settings.value("protocol", package_format) ) {
			Logger::log("Protocol is missing in config file:", cfg_filename, "for UDP link", Logger::LOGLEVEL_WARN);
			package_format = ProtocolStack::MAVLINKPACKAGE;
		}
		ProtocolStack::instance().add_link(udp, package_format);

		settings.end_group();
	}
 
	//create modules
// 	CoreModule *core_app = new CoreModule();

// 	ProtocolStack::instance().add_application(core_app);

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
