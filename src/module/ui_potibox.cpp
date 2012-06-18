#include "ui_potibox.h"

#ifdef HAVE_MAVLINK_H

#include "core/logger.h"
#include "utility.h"
#include "protocol/protocolstack.h"

#include <iostream> //cout

#include "sensor/sensormanager.h"
using namespace std;

namespace mavhub {
	UI_Potibox::UI_Potibox(const map<string, string> args) :
		AppInterface("ui_potibox"),
		ModuleBase(args, "ui_potibox") {

		assign_variable_from_args(component_id);

		read_conf(args);

	}

	UI_Potibox::~UI_Potibox() {}

	void UI_Potibox::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;
		Logger::log(name(), "read_conf", Logger::LOGLEVEL_INFO);


		iter = args.find("ch3_gain");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ch3_gain"];
		}

		Logger::log(name(), "read_conf: ch3_gain", params["ch3_gain"], Logger::LOGLEVEL_INFO);
		
	}

	void UI_Potibox::handle_input(const mavlink_message_t &msg) {
		int i;
		static float param_val;
		// static int8_t param_id[15];
		static float dir = 1.;
		static float gain = 0.01;
		string param_id_alt_sp("alt_sp");
		string param_id_lat_ctl_dir("ctl_direction");
		string param_id_lat_ctl_axis("ctl_axis");
		// Logger::log("UI_Potibox got mavlink_message", Logger::LOGLEVEL_INFO);
		switch(msg.msgid) {
		case MAVLINK_MSG_ID_HUCH_POTIBOX:
			// Logger::log("UI_Potibox got mavlink_message", Logger::LOGLEVEL_INFO);
			// FIXME: use configurable mapping
			mavlink_msg_huch_potibox_get_a(&msg, a_new);
			mavlink_msg_huch_potibox_get_d(&msg, d_new);
			// Logger::log(name(), "a[0]", a[0], Logger::LOGLEVEL_DEBUG);
			// anlog inputs
			for(i = 0; i < 6; i++) {
				if(abs(a[i] - a_new[i]) > 3) { // value changed
 					// assign
					a[i] = a_new[i];

					// send
					if(i == 0) {

						param_request_set(system_id(),
															38,
															string("alt_sp"),
															(float)(5. * a[0]/1024.));

						// Logger::log(name(), "sent param_set \"alt_sp\"", (int)param_set.target_system, param_set.param_value, Logger::LOGLEVEL_DEBUG);
					}

					else if(i == 1) {

						param_request_set(system_id(),
															39,
															string("ctl_axis"),
															(float)(a[1] > 512));

						// Logger::log(name(), "sent param_set", (int)param_set.target_system, Logger::LOGLEVEL_DEBUG);
					}

					else if(i == 2) {

						dir = (float)((a[2] < 512) * 2 - 1);
						param_request_set(system_id(),
															39,
															string("ctl_direction"),
															gain * dir);

						// Logger::log(name(), "sent param_set: dir", (int)param_set.target_system, dir, Logger::LOGLEVEL_DEBUG);
					}

					else if(i == 3) {

						gain = (float)(a[3] * params["ch3_gain"]);
						dir = (float)((a[2] < 512) * 2 - 1);

						param_request_set(system_id(),
															39,
															string("ctl_direction"),
															gain * dir);

						// Logger::log(name(), "sent param_set: gain", gain, Logger::LOGLEVEL_DEBUG);
					}

					else if(i == 4) {

						gain = (float)(a[4]) / 200.;
						// dir = (float)((a[2] < 512) * 2 - 1);

						param_request_set(system_id(),
															28,
															string("pitch_Kc"),
															gain);
						// param_request_set(system_id(),
						// 									28,
						// 									string("roll_Kc"),
						// 									0.0);
						param_request_set(system_id(),
															28,
															string("roll_Kc"),
															-gain);

						// Logger::log(name(), "sent param_set: gain", gain, Logger::LOGLEVEL_DEBUG);
					}

					else if(i == 5) {

						gain = ((float)(a[5]) / 100.) - 5.12;
						// dir = (float)((a[2] < 512) * 2 - 1);

						param_request_set(system_id(),
															28,
															string("pitch_sp"),
															gain);

						// Logger::log(name(), "sent param_set: gain", gain, Logger::LOGLEVEL_DEBUG);
					}

					Logger::log(name(), "value changed, a-", i, a[i], a_new[i], param_val, Logger::LOGLEVEL_DEBUG);
				}
			}
			
			// digital inputs
			for(i = 0; i < 4; i++) {
				if(d[i] != d_new[i]) {
					// first button
					if(i == 0 && d[i] < d_new[i]) {

 						send_action(system_id(), 39, ACTION_BUMP_ROLL);
						// Logger::log(name(), "sent action", (int)action.target, (int)action.target_component, Logger::LOGLEVEL_DEBUG);

					}

					else if(i == 1 && d[i] < d_new[i]) {

						send_action(system_id(), 26, ACTION_TOGGLE_AC);
						send_action(system_id(), 41, ACTION_TOGGLE_AC);
						// Logger::log(name(), "sent action", (int)action.target, (int)action.target_component, Logger::LOGLEVEL_DEBUG);

					}

					else if(i == 2 && d[i] < d_new[i]) {

						// send_action(system_id(), 33, ACTION_TOGGLE_LC);
						send_action(system_id(), 41, ACTION_TOGGLE_LC);
						// Logger::log(name(), "sent action", (int)action.target, (int)action.target_component, Logger::LOGLEVEL_DEBUG);

					}

					d[i] = d_new[i];
					Logger::log(name(), "value changed, d-", i, d[i], d_new[i], action.action, Logger::LOGLEVEL_DEBUG);
				}
			}
			break;
		default:
			break;
		}
	}

	void UI_Potibox::run() {
#ifdef MAVLINK_ENABLED_HUCH
		// mavlink_huch_altitude_t altitude;
		// mavlink_huch_magnetic_kompass_t kompass;
		// int count = 0;
		if(!owner()) {
			Logger::log("Owner of UI_Potibox not set", Logger::LOGLEVEL_WARN);
			return;
		}

		Logger::debug("UI_Potibox: running");
		mavlink_message_t msg;
		mavlink_msg_heartbeat_pack(100,
			200,
			&msg,
			MAV_TYPE_FIXED_WING,
			MAV_AUTOPILOT_GENERIC,
			MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,	//base mode
			0,	//custom mode
			MAV_STATE_ACTIVE);	//system status

		while( !interrupted() ) {
			// send(msg);
			sleep(1);

			// try {
			// 	SensorManager::instance().get_data(altitude, 0x01150085);

			// 	ostringstream send_stream00;
			// 	send_stream00 << "altitude: " << altitude.altitude;		
			// 	Logger::debug(send_stream00.str());
			// }
			// catch (const char *message) {
			// 	std::string s(message);
			// 	s = "in test core: " + s;
			// 	Logger::warn(s);
			// }

			// try {
			// 	SensorManager::instance().get_data(kompass, 0x01165843);

			// 	ostringstream send_stream01;
			// 	send_stream01 << "kompass: " << kompass.data_x << ";" << kompass.data_y << ";" << kompass.data_z;
			// 	Logger::debug(send_stream01.str());
			// }
			// catch (const char *message) {
			// 	std::string s(message);
			// 	s = "in test core: " + s;
			// 	Logger::warn(s);
			// }

			// switch(count++) {
			// 	case 5:
			// 		SensorManager::instance().stop_sensor(0x01150085);
			// 		break;
			// 	case 7:
			// 		SensorManager::instance().stop_sensor(0x01165843);
			// 		break;
			// 	case 10:
			// 		SensorManager::instance().start_sensor(0x01165843);
			// 		break;
			// 	case 11:
			// 		SensorManager::instance().start_sensor(0x01150085);
			// 		break;
			// 	case 13:
			// 		SensorManager::instance().restart_sensor(0x01150085);
			// 		SensorManager::instance().remove_sensor(0x01165843);
			// 		break;
			// 	case 15:
			// 		SensorManager::instance().remove_sensor(0x01150085);
			// 		break;
			// }
		}
#endif // MAVLINK_ENABLED_HUCH
	}

} // namespace mavhub

#endif // HAVE_MAVLINK_H
