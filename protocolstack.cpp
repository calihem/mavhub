#include "protocolstack.h"

#include "logger.h"
#include "utility.h"
#include <mavlink.h>
#include "mkpackage.h"
#include <algorithm> //find
#include <sys/time.h> //gettimeofday

#include <iostream> //cout
using namespace std;
using namespace cpp_pthread;

namespace mavhub {

ProtocolStack& ProtocolStack::instance() {
	// instantiated on first use and will be guaranteed destroyed
	static ProtocolStack instance;

	return instance;
}

ProtocolStack::ProtocolStack(uint8_t system_id) :
		sys_id(system_id),
		highest_fd(-1) {
	pthread_mutex_init(&link_mutex, NULL);
	pthread_mutex_init(&tx_mutex, NULL);
}

ProtocolStack::~ProtocolStack() {
}

void ProtocolStack::run() {
	fd_set read_fds;
	int fds_ready;
	timeval timeout;

	Logger::debug("entering ProtocolStack::run()");

	//activate apps
	std::list<AppLayer*>::iterator app_iter;
	for(app_iter = app_list.begin(); app_iter != app_list.end(); ++app_iter) {
		(*app_iter)->start();
	}

	while(1) {
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;

		links_to_file_set(read_fds);
		fds_ready = select(highest_fd+1, &read_fds, NULL, NULL, &timeout);
		Logger::log(fds_ready, "link(s) ready to read", Logger::LOGLEVEL_DEBUG);
		if(fds_ready < 0) {
			Logger::log("select failed with", strerror(errno), Logger::LOGLEVEL_ERROR);
			continue;
		}
		if(fds_ready == 0) continue;

		read(read_fds);
	}
}

void ProtocolStack::read(const fd_set& fds) {
	interface_packet_list_t::iterator iface_iter;
	vector<uint8_t> rx_buffer(BUFFERLENGTH);
	int received;
	int channel = 0;
	mavlink_message_t msg;
	static mavlink_status_t status;
	buffer_list_t::iterator buf_iter = rx_buffer_list.begin();

	{ //begin of link mutex scope
	Lock lm_lock(link_mutex);
	//iterate through interfaces
	for(iface_iter = interface_list.begin(); iface_iter != interface_list.end(); ++iface_iter ) {
		if( !FD_ISSET( (iface_iter->first)->handle(), &fds) ) {
			channel++;
			continue;
		}

		received = (iface_iter->first)->read( &rx_buffer[0], rx_buffer.size() );
		if(received < 0) {
			if(errno != EAGAIN) {
				Logger::log("reading of interface failed with", strerror(errno), Logger::LOGLEVEL_ERROR);
			}
			channel++;
			continue;
		}
		//catch strange 0 byte
		if(received == 0) {
			channel++;
			continue;
		}

		//received data, try to parse it
		Logger::log(received, "bytes received on channel", channel, Logger::LOGLEVEL_DEBUG);

		switch(iface_iter->second) {
			case MAVLINKPACKAGE:
				for(int i=0; i<received; i++) {
					if( mavlink_parse_char(channel, rx_buffer[i], &msg, &status) ) {
						Logger::log("received mavlink packet on channel", channel, Logger::LOGLEVEL_DEBUG);
						//broadcast msg on every mavlink channel
						retransmit(msg, iface_iter->first);
						//send msg to applications
						transmit_to_apps(msg);
					}
				}
				break;
			case MKPACKAGE: {
				if( buf_iter->empty() ) { //synchronize to start sign
					std::vector<uint8_t>::iterator start_iter;
					start_iter = std::find(rx_buffer.begin(), rx_buffer.begin()+received, '#');
					if( start_iter != rx_buffer.end() ) {//found #
						Logger::log("found MK start sign", Logger::LOGLEVEL_DEBUG);
						buf_iter->insert( buf_iter->end(), start_iter, rx_buffer.begin()+received );
					} else {//throw data away
						Logger::log("throw data away", Logger::LOGLEVEL_DEBUG);
						buf_iter++;
						break;
					}
				} else {//append data
					Logger::log("append MK data", Logger::LOGLEVEL_DEBUG);
					buf_iter->insert( buf_iter->end(), rx_buffer.begin(), rx_buffer.begin()+received );
				}

				//look for stop sign
				std::vector<uint8_t>::iterator stop_iter = std::find(buf_iter->begin()+3, buf_iter->end(), '\r');

				while( stop_iter != buf_iter->end() ) {//found \r
					Logger::log("found MK stop sign", Logger::LOGLEVEL_DEBUG);
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
					Logger::log("received MK packet on channel", channel, Logger::LOGLEVEL_DEBUG);
					//broadcast msg on every mkpackage channel
					retransmit(*mk_package, iface_iter->first);

					//convert package to mavling before sending to apps
					if( !mk2mavlink(*mk_package, msg) ) {
						transmit_to_apps(msg);
					}
					delete mk_package;

					//remove data of mk_package from buffer
					buf_iter->erase(buf_iter->begin(), buf_iter->begin()+data_length);
					Logger::log(data_length, "bytes cleared out of mk buffer on channel ", channel, Logger::LOGLEVEL_DEBUG);
					//ensure that buffer starts with start sign
					if( buf_iter->size() > 0 && buf_iter->at(0) != '#' ) {
						Logger::log("synchronize mk stream", Logger::LOGLEVEL_WARN);
						//remove data from buffer up to next start sign
						std::vector<uint8_t>::iterator start_iter;
						start_iter = std::find(buf_iter->begin()+1, buf_iter->end(), '#');
						if( start_iter != buf_iter->end() ) {//found new start sign
							buf_iter->erase(buf_iter->begin(), start_iter-1);
						} else {//no start sign found
							buf_iter->clear();
						}
					}
					//look for next stop sign
					stop_iter = std::find(buf_iter->begin()+3, buf_iter->end(), '\r');
				}
				buf_iter++;
				break;
			}
			default:
				Logger::log("unsupported package format on channel", channel, Logger::LOGLEVEL_DEBUG);
				break;
		}
		channel++;
	}

	}// end of link mutex scope

}

void ProtocolStack::links_to_file_set(fd_set& fds) const {
	FD_ZERO(&fds);
	// add file descriptors to set
	interface_packet_list_t::const_iterator iface_iter;
	Lock lm_lock(link_mutex);
	for(iface_iter = interface_list.begin(); iface_iter != interface_list.end(); ++iface_iter ) {
		FD_SET( (iface_iter->first)->handle() , &fds);
	}
}

void ProtocolStack::send(const mavlink_message_t &msg, const AppLayer *app) const {
	retransmit_to_apps(msg, app);
	
	Lock tx_lock(tx_mutex);
	uint16_t len = mavlink_msg_to_send_buffer(tx_buffer, &msg);

	Lock lm_lock(link_mutex);

	interface_packet_list_t::const_iterator iface_iter;
	for(iface_iter = interface_list.begin(); iface_iter != interface_list.end(); ++iface_iter ) {
		if(iface_iter->second == MAVLINKPACKAGE)
			iface_iter->first->write(tx_buffer, len);
	}
}

void ProtocolStack::send(const MKPackage &msg, const AppLayer *app) const {
	//FIXME: send to apps
	
	Lock lm_lock(link_mutex);

	interface_packet_list_t::const_iterator iface_iter;
	for(iface_iter = interface_list.begin(); iface_iter != interface_list.end(); ++iface_iter ) {
		if(iface_iter->second == MKPACKAGE)
			iface_iter->first->write(msg.rawData(), msg.rawSize());
	}
}

void ProtocolStack::retransmit(const mavlink_message_t &msg, const cpp_io::IOInterface *src_iface) const {
	Lock tx_lock(tx_mutex);

	uint16_t len = mavlink_msg_to_send_buffer(tx_buffer, &msg);

	// here locking of link_mutex is not neccesary

	interface_packet_list_t::const_iterator iface_iter;
	for(iface_iter = interface_list.begin(); iface_iter != interface_list.end(); ++iface_iter ) {
		if(iface_iter->second == MAVLINKPACKAGE
		&& iface_iter->first != src_iface) {
			iface_iter->first->write(tx_buffer, len);
		}
	}
}

void ProtocolStack::retransmit(const MKPackage &msg, const cpp_io::IOInterface *src_iface) const {
	// locking of link_mutex is done by run()
	
	interface_packet_list_t::const_iterator iface_iter;
	for(iface_iter = interface_list.begin(); iface_iter != interface_list.end(); ++iface_iter ) {
		if(iface_iter->second == MKPACKAGE
		&& iface_iter->first != src_iface) {
			Logger::log("send mk package on ", iface_iter->first->name(), Logger::LOGLEVEL_DEBUG);
			iface_iter->first->write(msg.rawData(), msg.rawSize());
			Logger::log("sent mk package on ", iface_iter->first->name(), Logger::LOGLEVEL_DEBUG);
		}
	}
}

int ProtocolStack::mk2mavlink(const MKPackage &mk_msg, mavlink_message_t &mav_msg) {

	switch(mk_msg.command()) {
		case 'D': // normal debug
			mavlink_msg_mk_debugout_pack(mk_msg.address(), 42, &mav_msg, (const int8_t*)(&mk_msg.data()[0]));
			return 0;
		default: break;
	}

	return -1;
}

int ProtocolStack::add_link(cpp_io::IOInterface *interface, const packageformat_t format) {
	if(!interface) return -1;

	Lock lm_lock(link_mutex);
	
	if(interface_list.size() == MAVLINK_COMM_NB_HIGH) {
		Logger::log("reached maximum number of interfaces", Logger::LOGLEVEL_WARN);
		return -2;
	}

	interface->enable_blocking_mode(false);
	interface_list.push_back( make_pair(interface, format) );

	if(format == MKPACKAGE) {
		rx_buffer_list.push_back( vector<uint8_t>() );
	}

	if(interface->handle() > highest_fd)
		highest_fd = interface->handle();

	return 0;
}

cpp_io::IOInterface* ProtocolStack::link(unsigned int link_id) {
	if(interface_list.size() >= link_id+1) { //ID is in range
		interface_packet_list_t::iterator iface_iter = interface_list.begin();
		// seek iterator to right position
		for(unsigned int i=0; i<link_id; i++) iface_iter++;
		return iface_iter->first;
	}
	return NULL;
}

int ProtocolStack::remove_link(unsigned int link_id) {
	Lock lm_lock(link_mutex);

	if(interface_list.size() >= link_id+1) { //ID is in range
		interface_packet_list_t::iterator iface_iter = interface_list.begin();
		// seek iterator to right position
		for(unsigned int i=0; i<link_id; i++) iface_iter++;
		if(iface_iter->second == MKPACKAGE) {
			//TODO: remove rx_buffer
		}
		interface_list.erase(iface_iter);
		//TODO: get new highest_fd
		return 0;
	}

	return -1;
}


void ProtocolStack::add_application(AppLayer *app) {
	if(!app) return;

	app->set_owner(this);
	app_list.push_back(app);
}

std::ostream& operator <<(std::ostream &os, const ProtocolStack &proto_stack) {
	Lock lm_lock(proto_stack.link_mutex);

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
			<< std::setw(15) << iface_iter->first->description()
			<< std::setw(15) << iface_iter->first->name()
			<< std::setw(10) << iface_iter->second
			<< endl;
		id++;
	}

	return os;
}

} // namespace mavhub
