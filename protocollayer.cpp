#include "protocollayer.h"

#include "logger.h"

namespace mavhub {

UARTLayer::UARTLayer(const char* devicename, tcflag_t control_modes) throw(const char*) :
		UART(devicename, control_modes) {
}

UARTLayer::~UARTLayer(){ }

UDPLayer::UDPLayer(int port) throw(const char*) : UDPSocket(port) {
	enable_blocking_mode(false);
}

UDPLayer::~UDPLayer(){ }

} // namespace mavhub
