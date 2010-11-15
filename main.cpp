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
#include "module/fc_mpkg.h"

// sensors
#include "module/senbmp085.h"
#include "module/testcore.h"

using namespace std;
using namespace mavhub;
using namespace cpp_pthread;
using namespace cpp_io;

uint8_t mavhub::system_id(42);
int tcp_port(32000);
string cfg_filename("mavhub.conf");
list<I2cSensor*> i2c_sensors;

void read_settings(Setting &settings);
void add_links(const list<string> link_list, Setting &settings);
void parse_argv(int argc, char **argv);
void print_help();

int main(int argc, char **argv) {
	Logger::setLogLevel(Logger::LOGLEVEL_WARN);

	parse_argv(argc, argv);

	//open config file and read settings
	Setting settings(cfg_filename, std::ios_base::in);
	read_settings(settings);
 
	//create test application
	TestCore *test_app = new TestCore();
 	ProtocolStack::instance().add_application(test_app);

	// start modules
	for (list<I2cSensor*>::iterator iter = i2c_sensors.begin(); iter != i2c_sensors.end(); ++iter) {
		(*iter)->start();
	}

	FC_Mpkg *fc_mpkg_app = new FC_Mpkg();
	// fc_mpkg_mod->start();
	ProtocolStack::instance().add_application(fc_mpkg_app);

	//activate stack
	pthread_t stack_thread = ProtocolStack::instance().start();
	
	//start mav shell
	try {
		MAVShell *mav_shell = new MAVShell(tcp_port);
		pthread_t shell_thread = mav_shell->start();
		PThread::join(shell_thread);
	}
	catch(const char* message) {
		cout << message << endl;
	}

	//join threads
	PThread::join(stack_thread);
}

void read_settings(Setting &settings) {
	//read loglevel
	Logger::log_level_t loglevel;
	if( settings.value("loglevel", loglevel) )
		Logger::log("Loglevel is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
	else
		Logger::setLogLevel(loglevel);

	//read system ID
	if( settings.value("system_id", system_id) )
		Logger::log("System ID is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
	ProtocolStack::instance().system_id(system_id);

	//read TCP port of MAVShell
	if( settings.value("tcp_port", tcp_port) )
		Logger::log("TCP port is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);

	//read interfaces (links)
	list<string> link_list;
	if( settings.value("interfaces", link_list) ) {
		Logger::log("List of links is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
	} else {
		add_links(link_list, settings);
	}
 
	if( settings.begin_group("sensors") == 0) { //sensor group available
		string i2c_config_file;
		if ( settings.value("i2c_config_file", i2c_config_file) ) { 
			Logger::log("i2c config file is missing in config file:", cfg_filename, Logger::LOGLEVEL_WARN);
		} else {
			SensorFactory::build(i2c_sensors, i2c_config_file);
		}
		settings.end_group();
	}
}

void add_links(const list<string> link_list, Setting &settings) {
	for(list<string>::const_iterator link_iter = link_list.begin(); link_iter != link_list.end(); ++link_iter) {
		if( settings.begin_group(*link_iter) == 0) { //link group available
			LinkFactory::link_type_t link_type(LinkFactory::UnsupportedLink);
			if( settings.value("type", link_type) ) {
				Logger::log("Link type is missing for", *link_iter, Logger::LOGLEVEL_WARN);
			} else {
				switch(link_type) {
					case LinkFactory::SerialLink: {
						string dev_name;
						if( settings.value("name", dev_name) ) {
							Logger::log("Device name is missing in config file:", cfg_filename, "for serial link", Logger::LOGLEVEL_WARN);
						} else {
							unsigned int baudrate(57600);
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
						break;
					}
					case LinkFactory::UDPLink: {
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
						break;
					}
					default:
						break;
				}
			}
			settings.end_group();
		} else {
			Logger::log(*link_iter, "link group entry is missing in config file", cfg_filename, Logger::LOGLEVEL_WARN);
		}
	}
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
