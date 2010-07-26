#include "protocolstack.h"

#include "logger.h"
#include <mavlink.h>

#include <iostream> //cout
using namespace std;

namespace mavhub {

ProtocolStack::ProtocolStack(uint8_t system_id) : system_id(system_id) {
}

ProtocolStack::~ProtocolStack() {
}

void ProtocolStack::run() {
	Logger::debug("entering ProtocolStack::run()");

	uint8_t c;
	int received;
	int channel;
	mavlink_message_t msg;
	mavlink_status_t status;
	interface_list_t::iterator iter;

	//TODO: number of channels must be < 16
	
	while(1) {
		channel = 0;
		//iterate through interfaces
		for(iter = interface_list.begin(); iter != interface_list.end(); ++iter ) {
			received = (*iter)->read(&c, 1);
			if(received<0) {
				if(errno != EAGAIN) {
					Logger::log("reading of interface failed with", strerror(errno), Logger::LOGLEVEL_ERROR);
					//FIXME: sleep is not the solution
					sleep(1);
					break;
				} else {//no data available
					break;
				}
			}
			if( mavlink_parse_char(channel, c, &msg, &status) ) {
				Logger::info("got mavlink");
			}
			channel++;
		}
	}
}

void ProtocolStack::addInterface(MediaLayer *interface) {
	interface_list.push_back(interface);
}

} // namespace mavhub
