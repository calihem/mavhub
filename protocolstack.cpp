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

ProtocolStack& ProtocolStack::instance() {
	// instantiated on first use and will be guaranteed destroyed
	static ProtocolStack instance;

	return instance;
}

ProtocolStack::ProtocolStack(uint8_t system_id) :
		sys_id(system_id) {
	pthread_mutex_init(&link_mutex, NULL);
	pthread_mutex_init(&tx_mutex, NULL);
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

	//activate apps
	std::list<AppLayer*>::iterator app_iter;
	for(app_iter = app_list.begin(); app_iter != app_list.end(); ++app_iter) {
		(*app_iter)->start();
	}

	while(1) {
		gettimeofday(&t1, 0);
		
		channel = 0;
		buf_iter = rx_buffer_list.begin();

		pthread_mutex_lock(&link_mutex);

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
							Logger::log("received mavlink packet on channel", channel, Logger::LOGLEVEL_DEBUG);
							//broadcast msg on every channel
							retransmit(msg, iface_iter->first);
							//send msg to applications
							transmit_to_apps(msg);
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
		
		pthread_mutex_unlock(&link_mutex);

		gettimeofday(&t2, 0);
		timediff(diff, t1, t2);
		if(diff.tv_usec < POLLINTERVAL) {
			usleep(POLLINTERVAL-diff.tv_usec);
		}
	}
}

void ProtocolStack::send(const mavlink_message_t &msg) const {
	pthread_mutex_lock(&tx_mutex);

	uint16_t len = mavlink_msg_to_send_buffer(tx_buffer, &msg);

	pthread_mutex_lock(&link_mutex);

	interface_packet_list_t::const_iterator iface_iter;
	for(iface_iter = interface_list.begin(); iface_iter != interface_list.end(); ++iface_iter ) {
		iface_iter->first->write(tx_buffer, len);
	}

	pthread_mutex_unlock(&link_mutex);
	pthread_mutex_unlock(&tx_mutex);

}

void ProtocolStack::retransmit(const mavlink_message_t &msg, const MediaLayer *src_iface) const {
	pthread_mutex_lock(&tx_mutex);

	uint16_t len = mavlink_msg_to_send_buffer(tx_buffer, &msg);

	// here locking of link_mutex is not neccesary

	interface_packet_list_t::const_iterator iface_iter;
	for(iface_iter = interface_list.begin(); iface_iter != interface_list.end(); ++iface_iter ) {
		if(iface_iter->first != src_iface) {
			iface_iter->first->write(tx_buffer, len);
		}
	}

	pthread_mutex_unlock(&tx_mutex);
}

int ProtocolStack::add_link(MediaLayer *interface, const packageformat_t format) {
	if(!interface) return -1;

	pthread_mutex_lock(&link_mutex);
	
	if(interface_list.size() == MAVLINK_COMM_NB_HIGH) {
		Logger::log("reached maximum number of interfaces", Logger::LOGLEVEL_WARN);
		return -2;
	}

	interface_list.push_back( make_pair(interface, format) );

	if(format == MKPACKAGE) {
		rx_buffer_list.push_back( vector<uint8_t>() );
	}
	
	pthread_mutex_unlock(&link_mutex);

	return 0;
}

MediaLayer* ProtocolStack::link(unsigned int link_id) {
	if(interface_list.size() >= link_id+1) { //ID is in range
		interface_packet_list_t::iterator iface_iter = interface_list.begin();
		// seek iterator to right position
		for(unsigned int i=0; i<link_id; i++) iface_iter++;
		return iface_iter->first;
	}
	return NULL;
}

int ProtocolStack::remove_link(unsigned int link_id) {
	int rc = -1;

	pthread_mutex_lock(&link_mutex);
	if(interface_list.size() >= link_id+1) { //ID is in range
		interface_packet_list_t::iterator iface_iter = interface_list.begin();
		// seek iterator to right position
		for(unsigned int i=0; i<link_id; i++) iface_iter++;
		if(iface_iter->second == MKPACKAGE) {
			//TODO: remove rx_buffer
		}
		interface_list.erase(iface_iter);
		rc = 0;
	}

	pthread_mutex_unlock(&link_mutex);

	return rc;
}


void ProtocolStack::add_application(AppLayer *app) {
	app->set_owner(this);
	app_list.push_back(app);
}

std::ostream& operator <<(std::ostream &os, const ProtocolStack &proto_stack) {
	pthread_mutex_lock(&proto_stack.link_mutex);

	// print headline
	os << std::setw(3) << "ID" 
		<< std::setw(15) << "Type"
		<< std::setw(15) << "Device" 
		<< std::setw(10) << "Protocol" 
		<< endl;
	
	ProtocolStack::interface_packet_list_t::const_iterator iface_iter;
	int id = 0;
	for(iface_iter = proto_stack.interface_list.begin(); iface_iter != proto_stack.interface_list.end(); ++iface_iter ) {
		os << std::setw(3) << id
			<< std::setw(15) << iface_iter->first->name()
			<< std::setw(15) << iface_iter->first->system_name()
			<< std::setw(10) << iface_iter->second
			<< endl;
		id++;
	}

	pthread_mutex_unlock(&proto_stack.link_mutex);

	return os;
}

} // namespace mavhub
