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
	//Logger::log("CoreModule got mavlink_message, msgid:", (int)msg.msgid, (int)msg.len, Logger::LOGLEVEL_INFO);
}

void CoreModule::run() {
	if(!owner) {
		Logger::log("Owner of CoreModule not set", Logger::LOGLEVEL_WARN);
		return;
	}

	int system_type = MAV_QUADROTOR;
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(owner->system_id(), 23, &msg, system_type, MAV_AUTOPILOT_GENERIC);

	Logger::log("CoreModule started", Logger::LOGLEVEL_INFO);

	while(1) {
		//Logger::log("coremod: system_id", static_cast<int>(owner->system_id()), Logger::LOGLEVEL_INFO);
		owner->send(msg);
		sleep(1);
	}
}


} // namespace mavhub
