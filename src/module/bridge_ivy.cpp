#include "bridge_ivy.h"

#ifdef HAVE_IVY_IVY_H

#include "core/logger.h"
#include "utility.h"
#include "protocol/protocolstack.h"
#include <mavlink.h>

#include <iostream> //cout
#include <stdlib.h>
using namespace std;

TimerId tid;

// /* callback associated to "Hello" messages */
// void cb_Hello (IvyClientPtr app, void *data, int argc, char **argv)
// {                                  
// 	const char* arg = (argc < 1) ? "" : argv[0];
// 	IvySendMsg ("Bonjour%s", arg);
// }
// /* callback associated to "Bye" messages */
// void cb_Bye (IvyClientPtr app, void *data, int argc, char **argv)
// {
// 	IvyStop ();
// }

// Setting message
void cb_DlSetting (IvyClientPtr app, void *data, int argc, char **argv)
{                                  
	mavlink_message_t msg;
	mavlink_param_set_t param_set;
	mavhub::Bridge_Ivy* bi = (mavhub::Bridge_Ivy*)data;

	if(argc == 4) {
		param_set.target_system = 43; //atoi(argv[1]);
		param_set.target_component = 27; // FIXME: HUCH_COMPID_SIM_CRRCSIM
		switch(atoi(argv[2])) {
		case 38: // ctl_mode
			//param_set.param_id = (int8_t*)strdup("ctl_mode");
			strncpy((char*)param_set.param_id, "ctl_mode", 9);
			param_set.param_value = atof(argv[3]);
			// FIXME
			mavlink_msg_param_set_encode(43, 27, &msg, &param_set);
			break;
		case 39:
			mavlink_action_t action;
			action.target = 43;
			action.target_component = 27;
			action.action = 0;
			printf("bump activate action request\n");
			mavlink_msg_action_encode(43, 27, &msg, &action);
			break;
		case 40:
			strncpy((char*)param_set.param_id, "bump_thr_low", 13);
			param_set.param_value = atof(argv[3]);
			// FIXME
			mavlink_msg_param_set_encode(43, 27, &msg, &param_set);
			break;			
		case 41:
			strncpy((char*)param_set.param_id, "bump_thr_high", 14);
			param_set.param_value = atof(argv[3]);
			// FIXME
			mavlink_msg_param_set_encode(43, 27, &msg, &param_set);
			break;			
		case 42: // ac pid bias
			strncpy((char*)param_set.param_id, "ac_pid_bias", 12);
			param_set.param_value = atof(argv[3]);
			// FIXME
			mavlink_msg_param_set_encode(43, 27, &msg, &param_set);
			break;			
		case 43: // ac pid Kc
			strncpy((char*)param_set.param_id, "ac_pid_Kc", 10);
			param_set.param_value = atof(argv[3]);
			// FIXME
			mavlink_msg_param_set_encode(43, 27, &msg, &param_set);
			break;			
		case 44: // ac pid Ti
			strncpy((char*)param_set.param_id, "ac_pid_Ki", 10);
			param_set.param_value = atof(argv[3]);
			// FIXME
			mavlink_msg_param_set_encode(43, 27, &msg, &param_set);
			break;			
		case 45: // ac pid Td
			strncpy((char*)param_set.param_id, "ac_pid_Kd", 10);
			param_set.param_value = atof(argv[3]);
			// FIXME
			mavlink_msg_param_set_encode(43, 27, &msg, &param_set);
			break;			
		case 46: // ac pid Td
			strncpy((char*)param_set.param_id, "ac_sp", 6);
			param_set.param_value = atof(argv[3]);
			// FIXME
			mavlink_msg_param_set_encode(43, 27, &msg, &param_set);
			break;			
		default:
			//param_set.param_id = "none";
			param_set.param_value = 0.0;
			break;
		}
 		// printf("bridge_ivy/DlSettingCallback: %d, %d, %s, %f\n",
		// 			 param_set.target_system,
		// 			 param_set.target_component,
		// 			 param_set.param_id,
		// 			 param_set.param_value
		// 			 );
		//const char* arg = (argc < 1) ? "" : argv[0];
		//IvySendMsg ("Bonjour%s", arg);
		bi->send_mavlink_msg(&msg);

 		printf("bridge_ivy/DlSettingCallback: %s, %s, %s, %s\n",
					 argv[0], argv[1], argv[2], argv[3]);
	}
	// mavlink_msg_param_value_pack(atoi(argv[1]), 27, &msg, (const int8_t*) "ctl_mode", atoi(argv[3]), 1, 0);
	// AppLayer<mavlink_message_t>::send(msg);
}

void (*pt2Function)(void*) = NULL;                        // C
//void (mavhub::Bridge_Ivy::*pt2Member)(void*) = NULL;                // C++


//void afterSelect_cb(ivy_cb_t* arg) {
void afterSelect_cb(void* arg) {
	//IvyHookPtr* a;
	//std::cout << arg->a ", " << arg->b << std::endl;
	//printf("%d, %f\n", arg->a, arg->b);
	printf("afterSelect_cb: %s\n", (char *)arg);
	//return a;
	//return 0;
}

// Timer callbacks
void handle_timer (TimerId id, void *data, unsigned long delta) {
	//if (wakeup(3) && send_loop()) decode_and_send_to_ivy();
	//if (wakeup(3))
	//do_something();
	//printf("handle_timer\n");
	//IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", 155);
}

void cb_DlValue (TimerId id, void *data, unsigned long delta) {
	mavhub::Bridge_Ivy* bi = (mavhub::Bridge_Ivy*)data;
	//if (wakeup(3) && send_loop()) decode_and_send_to_ivy();
	//if (wakeup(3))
	//do_something();
	//printf("handle_timer\n");
	if(!bi->doDlValue_())
		return;
	else {
		// send param_list_request
		// mavlink_param_request_list_t param_list
		mavlink_message_t msg;
		mavlink_msg_param_request_list_pack(
																				bi->system_id(), 1, &msg,
																				43, 27);
		bi->send_mavlink_msg(&msg);
		bi->_doDlValue(false);
	}
}

namespace mavhub {
	Bridge_Ivy::Bridge_Ivy(const map<string, string> args) : AppInterface("bridge_ivy"), AppLayer("bridge_ivy") {
		//, phi(0.0), theta(0.0), psi(0.0) 
		// initialize mod parameters from conf
		read_conf(args);
		/* initializations */
		char* bus = 0;
		bus = getenv ("IVYBUS");
		std::cout << bus << std::endl;
		IvyInit("MAVHUB", "READY", 0, 0, 0, 0);
		IvyStart(bus);

		//IvyBindMsg (cb_DlSetting, 0, "^(dl DL_SETTING) ([0-9]*) ([0-9]*) ([0-9]*.[0-9]*)");
		IvyBindMsg (cb_DlSetting, this, "^(dl DL_SETTING) ([0-9]*) ([0-9]*) ([0-9]*.[0-9]*)");

		doDlValue = true;
	}

	Bridge_Ivy::~Bridge_Ivy() {}

	void Bridge_Ivy::_doDlValue(bool val) {
		doDlValue = val;
	}

	bool Bridge_Ivy::doDlValue_() {
		return doDlValue;
	}

	void Bridge_Ivy::send_mavlink_msg(mavlink_message_t* msg) {
		printf("callbackcallback: %i\n", msg->sysid);
		AppLayer<mavlink_message_t>::send(*msg);
	}

	void Bridge_Ivy::handle_input(const mavlink_message_t &msg) {
		mavlink_huch_sensor_array_t sa;
		switch(msg.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:
			//Logger::log("Bridge_Ivy got ml heartbeat: (msgid, sysid)", (int)msg.msgid, (int)msg.sysid, Logger::LOGLEVEL_INFO);
			//IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", msg.sysid);
			IvySendMsg("%d ALIVE 50,162,250,192,221,27,111,57,63,249,122,206,139,11,197,244", msg.sysid);
			break;
		case MAVLINK_MSG_ID_ATTITUDE:
			// phi = mavlink_msg_attitude_get_roll(&msg);
			// theta = mavlink_msg_attitude_get_pitch(&msg);
			// psi = mavlink_msg_attitude_get_yaw(&msg);
			// //Logger::log("Bridge_Ivy got ml attitude", phi, theta, Logger::LOGLEVEL_INFO);
			//IvySendMsg("%d ATTITUDE %f %f %f", 155, phi, theta, psi);
			//IvySendMsg("%d ATTITUDE %f %f %f", msg.sysid, 0., 0., 0.);
			break;
		case MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY:
			mavlink_msg_huch_sensor_array_decode(&msg, &sa);
			//IvySendMsg("%d PRESSURE 0.0 0.0 %f %f", msg.sysid, sa.data[0], sa.data[1]);
			IvySendMsg("%d HUCH_SENSOR_RAW %f %f %f %f %f %f", msg.sysid,
								 sa.data[0], sa.data[1], sa.data[2], sa.data[3],
								 sa.data[4], sa.data[5]
								 );
			break;
		case MAVLINK_MSG_ID_PARAM_VALUE:
			Logger::log("Bridge_Ivy got ml param value: (msgid, sysid)", (int)msg.msgid, (int)msg.sysid, Logger::LOGLEVEL_INFO);
			static int8_t param_id[15];
			float param_value;
			int setting_id;
			mavlink_msg_param_value_get_param_id(&msg, param_id);
			param_value = mavlink_msg_param_value_get_param_value(&msg);
			Logger::log("Bridge_Ivy got ml param value: (param_id, param_value)", (char*)param_id, param_value, Logger::LOGLEVEL_INFO);
			if(!strcmp((char*)param_id, (const char*)"ctl_mode")) {
				setting_id = 38;
			}
			else if(!strcmp((char*)param_id, (const char*)"ctl_bump")) {
				setting_id = 39;
			}
			else if(!strcmp((char*)param_id, (const char*)"bump_thr_low")) {
				setting_id = 40;
			}
			else if(!strcmp((char*)param_id, (const char*)"bump_thr_high")) {
				setting_id = 41;
			}
			else if(!strcmp((char*)param_id, (const char*)"ac_pid_bias")) {
				setting_id = 42;
			}
			else if(!strcmp((char*)param_id, (const char*)"ac_pid_Kc")) {
				setting_id = 43;
			}
			else if(!strcmp((char*)param_id, (const char*)"ac_pid_Ki")) {
				setting_id = 44;
			}
			else if(!strcmp((char*)param_id, (const char*)"ac_pid_Kd")) {
				setting_id = 45;
			}
			else
				setting_id = -1;
			if(setting_id >= 0) {
			IvySendMsg("%d DL_VALUE %d %f",
								 system_id(),
								 setting_id,
								 param_value);
			}
			// 					 , 38, 2.0);
			// IvySendMsg("%d DL_VALUE %d %f", 42, 39, 0.0);
			break;

		default:
			break;
		}
		// AppLayer<mavlink_message_t>::send(msg);
	}

	void Bridge_Ivy::run() {
		if(!owner()) {
			Logger::log("Owner of Bridge_Ivy not set", Logger::LOGLEVEL_WARN);
			return;
		}

		//int system_type = MAV_QUADROTOR;
		// mavlink_message_t msg_heartbeat;
		// mavlink_msg_heartbeat_pack(system_id(), component_id, &msg_heartbeat, system_type, MAV_AUTOPILOT_HUCH);

		Logger::log("Bridge_Ivy started", Logger::LOGLEVEL_INFO);

		// // how to add before/after select hook
		// pt2Function = &afterSelect_cb;
		// // IvySetBeforeSelectHook(pt2Function, (void*)0);
		// IvySetAfterSelectHook(pt2Function, (void*)0);

		// with local member function, defunct
		// pt2Member = &mavhub::Bridge_Ivy::afterSelect_cb;
		// IvySetBeforeSelectHook(pt2Member, (void*)0);


		int delay = 1000;
		// tid = TimerRepeatAfter (0, delay, handle_timer, 0);
		tid = TimerRepeatAfter (0, delay, cb_DlValue, this);

		// libivy mainloop
		IvyMainLoop();

		// local mainloop
		// while(1) {
		// 	Logger::log("bridge_ivy: system_id", static_cast<int>(system_id()), Logger::LOGLEVEL_INFO);
		// 	//AppLayer<mavlink_message_t>::send(msg_heartbeat);
		// 	//send(msg_heartbeat);
		// 	//IvySendMsg("%d MAVHUB_ALIVE %f", 155, 1.0);
		// 	sleep(1);
		// }
	}

	void Bridge_Ivy::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;
		Logger::log("Bridge_Ivy::read_conf", Logger::LOGLEVEL_INFO);

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}
		Logger::log("Bridge_Ivy::read_conf: component_id", component_id, Logger::LOGLEVEL_INFO);
	}

	// void Bridge_Ivy::handle_timer (TimerId id, void *data, unsigned long delta) {
	// 	//if (wakeup(3) && send_loop()) decode_and_send_to_ivy();
	// 	//if (wakeup(3))
	// 	//do_something();
	// 	printf("handle_timer\n");
	// 	IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", 155);
	// }

	// void Bridge_Ivy::afterSelect_cb(void* arg) {
	// 	//IvyHookPtr* a;
	// 	//std::cout << arg->a ", " << arg->b << std::endl;
	// 	//printf("%d, %f\n", arg->a, arg->b);
	// 	printf("Bridge_Ivy::afterSelect_cb: 1,2,3\n");
	// 	//return a;
	// 	//return 0;
	// }

} // namespace mavhub

#endif // HAVE_IVY_IVY_H
