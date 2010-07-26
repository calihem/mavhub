#include "protocollayer.h"

#include "logger.h"

namespace mavhub {

UARTLayer::UARTLayer(const char* devicename, tcflag_t control_modes) throw(const char*) :
		UART(devicename, control_modes) {
}

UARTLayer::~UARTLayer(){ }

} // namespace mavhub
