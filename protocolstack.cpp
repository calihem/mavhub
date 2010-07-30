#include "protocolstack.h"

#include "logger.h"
#include "utility.h"
#include <mavlink.h>
#include "mkpackage.h"
#include <algorithm> //find
#include <sys/time.h> //gettimeofday

#include <iostream> //cout
using namespace std;

namespace mavhub {

ProtocolStack::ProtocolStack(uint8_t system_id) : system_id(system_id) {
}

ProtocolStack::~ProtocolStack() {
}

void ProtocolStack::run() {
	Logger::debug("entering ProtocolStack::run()");

	vector<uint8_t> rx_buffer(BUFFERLENGTH);
	int received;
	int channel;
	mavlink_message_t msg;
	mavlink_status_t status;
	interface_packet_list_t::iterator iface_iter;
	buffer_list_t::iterator buf_iter;
	std::vector<uint8_t>::iterator stop_iter;
	timeval t1, t2, diff;

	while(1) {
		gettimeofday(&t1, 0);
		
		channel = 0;
		buf_iter = rx_buffer_list.begin();

		//iterate through interfaces
		//TODO: use select on fd_set
		for(iface_iter = interface_list.begin(); iface_iter != interface_list.end(); ++iface_iter ) {
			received = (iface_iter->first)->read( &rx_buffer[0], rx_buffer.size() );
			if(received<0) {
				if(errno != EAGAIN) {
					Logger::log("reading of interface failed with", strerror(errno), Logger::LOGLEVEL_ERROR);
					channel++;
					continue;
				} else {//no data available
					channel++;
					continue;
				}
			}
			//received data, try to parse it
			Logger::log(received, "bytes received on channel", channel, Logger::LOGLEVEL_DEBUG);

			switch(iface_iter->second) {
				case MAVLINKPACKAGE:
					for(int i=0; i<received; i++) {
						if( mavlink_parse_char(channel, rx_buffer[i], &msg, &status) ) {
							Logger::info("got mavlink");
						}
					}
					break;
				case MKPACKAGE:
					if( buf_iter->empty() ) { //synchronize to start sign
						std::vector<uint8_t>::iterator start_iter;
						start_iter = std::find(rx_buffer.begin(), rx_buffer.end(), '#');
						if( start_iter != rx_buffer.end() ) {//found #
							buf_iter->insert( buf_iter->end(), start_iter, rx_buffer.end() );
						} else {//throw data away
							buf_iter++;
							break;
						}
					} else {//append data
						buf_iter->insert( buf_iter->end(), rx_buffer.begin(), rx_buffer.end() );
					}

					//look for stop sign
					stop_iter = std::find(buf_iter->begin()+3, buf_iter->end(), '\r');

					while( stop_iter != buf_iter->end() ) {//found \r
						MKPackage *mk_package;
						int data_length;
						try{
							//TODO: use static parse method instead of constructor
							data_length = stop_iter-buf_iter->begin()+1;
							mk_package = new MKPackage(&(*buf_iter)[0], data_length);
						}
						catch(const char *message) {
							Logger::log(message, Logger::LOGLEVEL_ERROR);
							//no valid MKPackage, remove data up to next start sign
							std::vector<uint8_t>::iterator start_iter;
							start_iter = std::find(stop_iter+1, buf_iter->end(), '#');
							if( start_iter != buf_iter->end() ) {//found new start sign
								buf_iter->erase(buf_iter->begin(), start_iter-1);
							} else {//no start sign found
								buf_iter->clear();
								break;
							}
							//memory occupied by mk_package is deleted due to exception
							//no need of delete mk_package

							stop_iter = std::find(buf_iter->begin()+3, buf_iter->end(), '\r');
							continue;
						}
						Logger::debug("got mkpackage");
						delete mk_package;

						//remove data of mk_package from buffer
						buf_iter->erase(buf_iter->begin(), buf_iter->begin()+data_length);
						//ensure that buffer starts with start sign
						if( buf_iter->at(0) != '#' ) {
							//remove data from buffer up to next start sign
							std::vector<uint8_t>::iterator start_iter;
							start_iter = std::find(buf_iter->begin()+1, buf_iter->end(), '#');
							if( start_iter != buf_iter->end() ) {//found new start sign
								buf_iter->erase(buf_iter->begin(), start_iter-1);
							} else {//no start sign found
								buf_iter->clear();
							}
						}
						stop_iter = std::find(buf_iter->begin()+3, buf_iter->end(), '\r');
					}
					buf_iter++;
					break;
				default:
					Logger::log("unsupported package format on channel", channel, Logger::LOGLEVEL_DEBUG);
					break;
			}
			channel++;
		}
		gettimeofday(&t2, 0);
		timediff(diff, t1, t2);
		if(diff.tv_usec < POLLINTERVAL) {
			usleep(POLLINTERVAL-diff.tv_usec);
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
