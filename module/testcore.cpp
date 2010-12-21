#include "testcore.h"

#include "logger.h"
#include "utility.h"
#include "protocolstack.h"
#include <mavlink.h>

#include <iostream> //cout

#include "sensormanager.h"
using namespace std;

namespace mavhub {
TestCore::TestCore() {
	app_id = 0;
	app_name = "testcore";
}

TestCore::~TestCore() {}

void TestCore::handle_input(const mavlink_message_t &msg) {
	Logger::log("TestCore got mavlink_message", Logger::LOGLEVEL_INFO);
}

void TestCore::run() {
	mavlink_huch_altitude_t altitude;
	mavlink_huch_magnetic_kompass_t kompass;
	int count = 0;
	if(!owner) {
		Logger::log("Owner of TestCore not set", Logger::LOGLEVEL_WARN);
		return;
	}

	Logger::debug("TestCore: running");
	int system_type = MAV_FIXED_WING;
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, MAV_AUTOPILOT_GENERIC);

	while(1) {
		send(msg);
		sleep(1);
		try {
			SensorManager::instance().get_data(altitude, 0x01150085);

			ostringstream send_stream00;
			send_stream00 << "altitude: " << altitude.altitude;		
			Logger::debug(send_stream00.str());
		}
		catch (const char *message) {
			std::string s(message);
			s = "in test core: " + s;
			Logger::warn(s);
		}
		try {
			SensorManager::instance().get_data(kompass, 0x01165843);

			ostringstream send_stream01;
			send_stream01 << "kompass: " << kompass.data_x << ";" << kompass.data_y << ";" << kompass.data_z;
			Logger::debug(send_stream01.str());
		}
		catch (const char *message) {
			std::string s(message);
			s = "in test core: " + s;
			Logger::warn(s);
		}

		switch(count++) {
			case 5:
				SensorManager::instance().stop_sensor(0x01150085);
				break;
			case 7:
				SensorManager::instance().stop_sensor(0x01165843);
				break;
			case 10:
				SensorManager::instance().start_sensor(0x01165843);
				break;
			case 11:
				SensorManager::instance().start_sensor(0x01150085);
				break;
			case 13:
				SensorManager::instance().restart_sensor(0x01150085);
				SensorManager::instance().remove_sensor(0x01165843);
				break;
			case 15:
				SensorManager::instance().remove_sensor(0x01150085);
				break;
		}
	}
}


} // namespace mavhub
