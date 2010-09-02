#include "coremod.h"

#include "logger.h"
#include "utility.h"
#include "protocolstack.h"
#include <mavlink.h>

#include <iostream> //cout
using namespace std;

namespace mavhub {
CoreModule::CoreModule() {
}

CoreModule::~CoreModule() {}

void CoreModule::handle_input(const mavlink_message_t &msg) {
	Logger::log("CoreModule got mavlink_message", Logger::LOGLEVEL_INFO);
}

void CoreModule::run() {
	if(!owner) {
		Logger::log("Owner of CoreModule not set", Logger::LOGLEVEL_WARN);
		return;
	}

	int system_type = MAV_FIXED_WING;
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, MAV_AUTOPILOT_GENERIC);

	while(1) {
		owner->send(msg);
		sleep(5);
	}
}


} // namespace mavhub
