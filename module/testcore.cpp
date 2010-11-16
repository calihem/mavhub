#include "testcore.h"

#include "logger.h"
#include "utility.h"
#include "protocolstack.h"
#include <mavlink.h>

#include <iostream> //cout

#include "datacenter.h"
using namespace std;

namespace mavhub {
TestCore::TestCore() {
}

TestCore::~TestCore() {}

void TestCore::handle_input(const mavlink_message_t &msg) {
	Logger::log("TestCore got mavlink_message", Logger::LOGLEVEL_INFO);
}

void TestCore::run() {
	mavlink_huch_bmp085_t bmp085_data_core;
	if(!owner) {
		Logger::log("Owner of TestCore not set", Logger::LOGLEVEL_WARN);
		return;
	}

	Logger::debug("TestCore: running");
	int system_type = MAV_FIXED_WING;
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, MAV_AUTOPILOT_GENERIC);

	while(1) {
		owner->send(msg);
		sleep(1);
		bmp085_data_core = DataCenter::get_bmp085();
		ostringstream send_stream;
		send_stream << "bmp085;" << bmp085_data_core.temperature << ";" << bmp085_data_core.pressure << ";" << bmp085_data_core.height;
		Logger::debug(send_stream.str());
	}
}


} // namespace mavhub
