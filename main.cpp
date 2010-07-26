#include <iostream> //cout
#include <cstring> //strcmp
#include <cstdlib> //exit

#include "logger.h"

using namespace std;
using namespace mavhub;

// int port = UDPSocket::DefaultPort;
int udp_port = 30000;
int tcp_port = 30001;

void parse_argv(int argc, char **argv);
void print_help();

int main(int argc, char **argv) {
	Logger::setLogLevel(Logger::LOGLEVEL_ALL);
	parse_argv(argc, argv);

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