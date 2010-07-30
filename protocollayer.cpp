#include "protocollayer.h"

#include "logger.h"

namespace mavhub {

AppLayer::AppLayer() : owner(0) {}

UARTLayer::UARTLayer(const char* devicename, tcflag_t control_modes) throw(const char*) :
		UART(devicename, control_modes) {
	enable_blocking_mode(false);
}

UARTLayer::~UARTLayer(){ }

UDPLayer::UDPLayer(int port) throw(const char*) : UDPSocket(port) {
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

int UDPLayer::write(const uint8_t *buffer, int buf_len) const {
	std::list<num_addr_pair_t>::const_iterator gmember_iter;
	int rc;

	for(gmember_iter = groupmember_list.begin(); gmember_iter != groupmember_list.end(); ++gmember_iter ) {
		try{
			rc = send_to(buffer, buf_len, gmember_iter->first, gmember_iter->second);
		}
		catch(const char *message) {
			Logger::log(message, Logger::LOGLEVEL_ERROR);
		}
	}
	
	return rc;
}

} // namespace mavhub
