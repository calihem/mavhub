#include "coreapp.h"

#include "logger.h"
#include "utility.h"
#include "protocolstack.h"
#include <mavlink.h>

#include <iostream> //cout
using namespace std;

namespace mavhub {
CoreApp::CoreApp() {
}

CoreApp::~CoreApp() {}

void CoreApp::handle_input(const mavlink_message_t &msg) {
	Logger::log("CoreApp got mavlink_message", Logger::LOGLEVEL_INFO);
}

void CoreApp::run() {
	if(!owner) {
		Logger::log("Owner of CoreApp not set", Logger::LOGLEVEL_WARN);
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
