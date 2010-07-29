#include "protocollayer.h"

#include "logger.h"

namespace mavhub {

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

} // namespace mavhub
