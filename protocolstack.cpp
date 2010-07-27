#include "protocolstack.h"

#include "logger.h"
#include <mavlink.h>
#include "mkpackage.h"
#include <algorithm> //find

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
	interface_packet_list_t::iterator iface_iter;
	buffer_list_t::iterator buf_iter;

	while(1) {
		channel = 0;
		buf_iter = rx_buffer_list.begin();

		//iface_iterate through interfaces
		for(iface_iter = interface_list.begin(); iface_iter != interface_list.end(); ++iface_iter ) {
			received = (iface_iter->first)->read(&c, 1);
			if(received<0) {
				if(errno != EAGAIN) {
					Logger::log("reading of interface failed with", strerror(errno), Logger::LOGLEVEL_ERROR);
					//FIXME: sleep is not the solution
					sleep(1);
					channel++;
					continue;
				} else {//no data available
					Logger::log("no data available on channel", channel, Logger::LOGLEVEL_DEBUG);
					channel++;
					continue;
				}
			}
			//received data, try to parse it
			switch(iface_iter->second) {
				case MAVLINKPACKAGE:
					if( mavlink_parse_char(channel, c, &msg, &status) ) {
						Logger::info("got mavlink");
					}
					break;
				case MKPACKAGE:
					if( buf_iter->empty() ) {//synchronize to start sign
						if(c != '#') {
							buf_iter++;
							break;
						}
						buf_iter->push_back(c);
					} else {//append data
						buf_iter->push_back(c);
						if(c != '\r') {// no stop sign
							buf_iter++;
							break;
						}
						//found stop sign, try to parse
						try{
							MKPackage mk_package( &(*buf_iter)[0], buf_iter->size() );
						}
						catch(const char *message) {
							//no valid MKPackage, remove data from rxBuffer up to next start sign
							vector<uint8_t>::iterator start_iter;
							start_iter = std::find(buf_iter->begin()+1, buf_iter->end(), '#');
							if( start_iter != buf_iter->end() ) {//found another start sign
								buf_iter->erase(buf_iter->begin(), start_iter-1);
							} else { //no further start sign
								buf_iter->clear();
							}
							buf_iter++;
							break;
						}
						Logger::debug("got mkpackage");
					}
					buf_iter++;
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

	if(format == MKPACKAGE) {
		rx_buffer_list.push_back( vector<uint8_t>() );
	}
}

} // namespace mavhub
