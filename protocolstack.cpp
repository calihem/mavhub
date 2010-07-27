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
	interface_packet_list_t::iterator iter;

	
	while(1) {
		channel = 0;
		//iterate through interfaces
		for(iter = interface_list.begin(); iter != interface_list.end(); ++iter ) {
			received = (iter->first)->read(&c, 1);
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
			//received data, try to parse it
			switch(iter->second) {
				case MAVLINKPACKAGE:
					if( mavlink_parse_char(channel, c, &msg, &status) ) {
						Logger::info("got mavlink");
					}
					break;
				case MKPACKAGE:
					break;
				default:
					Logger::log("unsupported package format on channel", channel, Logger::LOGLEVEL_DEBUG);
					break;
			}
			channel++;
		}
	}
}

void ProtocolStack::addInterface(MediaLayer *interface, const packageformat_t format) {
	if(interface_list.size() == MAVLINK_COMM_NB_HIGH) {
		Logger::log("reached maximum number of interfaces", Logger::LOGLEVEL_WARN);
		return;
	}

	interface_list.push_back( make_pair(interface, format) );
}

} // namespace mavhub
