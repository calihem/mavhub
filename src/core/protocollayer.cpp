#include "protocollayer.h"
#include "protocolstack.h"

#include <sstream>

namespace mavhub {

AppLayer::AppLayer(const std::string& name, const Logger::log_level_t loglevel) :
	_owner(0),
	_loglevel(loglevel),
	_name(name) {}

std::ostream& operator <<(std::ostream &os, const AppLayer &app) {
	app.print(os);
	return os;
}

void AppLayer::print(std::ostream &os) const {
	os << name() << ":" << std::endl
		<< "* loglevel: " << _loglevel << std::endl;
}

void AppLayer::send(const mavlink_message_t &msg) const {
	if(_owner) _owner->send(msg, this);
}

void AppLayer::send(const MKPackage &msg) const {
	if(_owner) _owner->send(msg, this);
}

// ----------------------------------------------------------------------------
// UDPLayer
// ----------------------------------------------------------------------------
UDPLayer::UDPLayer(int port) throw(const char*) :
		UDPSocket(port) {

	enable_blocking_mode(false);
}

UDPLayer::~UDPLayer(){ }

void UDPLayer::add_groupmember(const std::string& addr, uint16_t port) throw(const char*) {
	in_addr num_addr;

	if( inet_aton(addr.c_str(), &num_addr) == 0) {
		throw "Assignment of IP Address failed";
	}

	groupmember_list.push_back( std::make_pair(num_addr, port) );
}

void UDPLayer::add_groupmembers(const std::list<string_addr_pair_t>& member_list) throw(const char*) {
	if(member_list.empty()) return;

	std::list<string_addr_pair_t>::const_iterator it;
	for(it=member_list.begin(); it!=member_list.end(); ++it) {
		try {
			add_groupmember(it->first, it->second);
		}
		catch(const char *message) {
			throw message;
		}
	}

}

ssize_t UDPLayer::write(const void *buf, size_t nbyte) const {
	std::list<num_addr_pair_t>::const_iterator gmember_iter;
	int rc = 0;

	for(gmember_iter = groupmember_list.begin(); gmember_iter != groupmember_list.end(); ++gmember_iter ) {
		try{
			rc = send_to(buf, nbyte, gmember_iter->first, gmember_iter->second);
		}
		catch(const char *message) {
			Logger::log(message, Logger::LOGLEVEL_ERROR);
		}
	}
	
	return rc;
}

void UDPLayer::print(std::ostream &os) const {
	IOInterface::print(os);

	std::list<num_addr_pair_t>::const_iterator gmember_iter;
	os << "* Group members:" << std::endl;
	for(gmember_iter = groupmember_list.begin(); gmember_iter != groupmember_list.end(); ++gmember_iter) {
		os << "\t" << inet_ntoa(gmember_iter->first) << " " << gmember_iter->second << std::endl;
	}
}

} // namespace mavhub
