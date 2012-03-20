// control yaw component

#include "ctrl_yaw.h"

#ifdef HAVE_MAVLINK_H

#include "core/datacenter.h"
#include "protocol/protocolstack.h"

#include <cstdlib>
#include <sstream>

#include <math.h>

#define RAND_MAX_TO_M1 1/(double)RAND_MAX

using namespace std;

namespace mavhub {
	Ctrl_Yaw::Ctrl_Yaw(const map<string, string> args) :
		AppInterface("ctrl_yaw"),
		AppLayer<mavlink_message_t>("ctrl_yaw") {
		read_conf(args);
		param_request_list = 0;
		// pid_yaw = new PID(0, params["yaw_Kc"], params["yaw_Ti"],
		// 									params["yaw_Td"]);
	}

	Ctrl_Yaw::~Ctrl_Yaw() {
	}

  void Ctrl_Yaw::handle_input(const mavlink_message_t &msg) {
		static int8_t param_id[15];
		//Logger::log("Ctrl_Yaw got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_DEBUG);
		switch(msg.msgid) {

#ifdef MAVLINK_ENABLED_HUCH

		case MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION:
			// Logger::log("Ctrl_Yaw: got visual_navigation msg", Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_visual_navigation_decode(&msg, (mavlink_huch_visual_navigation_t *)&huch_visual_navigation);
			break;

		case MAVLINK_MSG_ID_HUCH_VISUAL_FLOW:
			//Logger::log("Ctrl_Yaw: got visual_flow msg", Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_visual_flow_decode(&msg, (mavlink_huch_visual_flow_t *)&huch_visual_flow);
			//Logger::log("psi_est:", huch_visual_flow.psi_estimate, Logger::LOGLEVEL_INFO);
			break;

		// case MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY:
		// 	// FIXME: implement this instead of datacenter
		// 	break;

#endif // MAVLINK_ENABLED_HUCH	

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Ctrl_Yaw::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
				param_request_list = 1;
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			// FIXME: make this a function
			if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
				Logger::log("Ctrl_Yaw::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Ctrl_Yaw::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log("Ctrl_Yaw::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);

					typedef map<string, double>::const_iterator ci;
					for(ci p = params.begin(); p!=params.end(); ++p) {
						// Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
						if(!strcmp(p->first.data(), (const char *)param_id)) {
							params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
							Logger::log("x Ctrl_Yaw::handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_INFO);
						}
					}

					// // update PID controllers
					// pid_pitch->setKc(params["yaw_Kc"]);
					// pid_pitch->setBias(params["yaw_bias"]);

				}
			}
			break;
		default:
			break;
		}		
	}

  void Ctrl_Yaw::run() {
		// generic
		static mavlink_message_t msg;
		//static mavlink_debug_t dbg;
		// timing
		// uint64_t dt = 0;
		// double dtf = 0.0;
		struct timeval tk, tkm1; // timevals
		int update_rate = 10; // 100 Hz
		int wait_freq = update_rate? 1000000 / update_rate: 0;
		int wait_time = wait_freq;
		uint64_t frequency = wait_time;
		uint64_t start = get_time_us();
		uint64_t usec;

		// body variables
		int yaw_meas;
		int sp;
		int compass_res, compass_res_half;
		double yaw, yaw1;
		// vector<int16_t> v(3);

		gettimeofday(&tk, NULL);
		gettimeofday(&tkm1, NULL);

		yaw = yaw1 = 0;
		sp = 0;
		yaw_meas = 0;
		compass_res = 256;
		compass_res_half = 128;

		Logger::log("Ctrl_Yaw started:", name(), Logger::LOGLEVEL_INFO);
		while(true) {
			/* wait time */
			usec = get_time_us();
			uint64_t end = usec;
			wait_time = wait_freq - (end - start);
			wait_time = (wait_time < 0)? 0: wait_time;
		
			/* wait */
			usleep(wait_time);
			//usleep(10);

			/* calculate frequency */
			end = get_time_us();
			frequency = (15 * frequency + end - start) / 16;
			start = end;

			// Logger::log("Ctrl_Yaw slept for", wait_time, component_id, Logger::LOGLEVEL_INFO);

			gettimeofday(&tk, NULL);
			//timediff(tdiff, tkm1, tk);
			// dt = (tk.tv_sec - tkm1.tv_sec) * 1000000 + (tk.tv_usec - tkm1.tv_usec);
			tkm1 = tk; // save current time
			// dt fractional in seconds
			// dtf = (double)dt * 1e-6;

			// param handling
			if(param_request_list) {
				Logger::log("Ctrl_Yaw::run: param request", Logger::LOGLEVEL_INFO);
				param_request_list = 0;

				typedef map<string, double>::const_iterator ci;
				for(ci p = params.begin(); p!=params.end(); ++p) {
					// Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_value_pack(system_id(), component_id, &msg, (const int8_t*) p->first.data(), p->second, 1, 0);
					send(msg);
				}

				// mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"prm_test_pitch", prm_test_pitch, 1, 0);
				// send(msg);
				// mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"prm_yaw_P", prm_yaw_P, 1, 0);
				// send(msg);
			}

			// // test huch_visual_navigation
			// huch_visual_navigation.psi_estimate = 1.234;
			// mavlink_msg_huch_visual_navigation_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &huch_visual_navigation);
			// send(msg);

#ifdef MAVLINK_ENABLED_HUCH

			// get magnetic 2D compass measurement
			yaw_meas = DataCenter::get_sensor(6);
			// calculate controller output
			yaw = (((sp - yaw_meas) + compass_res_half) % compass_res) - compass_res_half;
			// apply gain
			yaw *= params["yaw_Kc"];
			// //comp = (/255.0) * 6.28;
			// yaw = (int16_t)(prm_yaw_P * (0.0 + (comp/255.0) * 6.28));
			// Logger::log("Ctrl_Yaw (psi_est, yaw)", DataCenter::get_sensor(6), yaw, Logger::LOGLEVEL_INFO);

			// FIXME: optical compass
			// yaw1 = (int16_t)(params["yaw_Kc"] * (0.0 - huch_visual_navigation.psi_vc));
			// pid_yaw->setSp(0.0);
			// yaw = pid_yaw->calc(dtf, (0.0 - huch_visual_navigation.visual_compass));
			// Logger::log("Ctrl_Yaw (psi_est, yaw1)", huch_visual_navigation.psi_vc, yaw1, Logger::LOGLEVEL_INFO);
			//yaw = 0;

#endif // MAVLINK_ENABLED_HUCH	

			mavlink_msg_debug_pack( system_id(), component_id, &msg, 108, yaw);
			AppLayer<mavlink_message_t>::send(msg);
	
			// write output to shared store
			DataCenter::set_extctrl_yaw(yaw*-1.0);
			// Logger::log("Ctrl_Yaw (n,r,y)", v, Logger::LOGLEVEL_INFO);

		}
	}

	void Ctrl_Yaw::default_conf() {
		params["yaw_Kc"] = 100.0;
		params["yaw_Ti"] = 0.0;
		params["yaw_Td"] = 0.0;
	}

	void Ctrl_Yaw::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}
		// controller params yaw
		iter = args.find("yaw_Kc");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["yaw_Kc"];
		}
		iter = args.find("yaw_Ti");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["yaw_Ti"];
		}
		iter = args.find("yaw_Td");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["yaw_Td"];
		}

		Logger::log("ctrl_yaw::read_conf: component_id", component_id, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_yaw::read_conf: yaw_Kc", params["yaw_Kc"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_yaw::read_conf: yaw_Ti", params["yaw_Ti"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_yaw::read_conf: yaw_Td", params["yaw_Td"], Logger::LOGLEVEL_DEBUG);
	}
}

#endif // HAVE_MAVLINK_H

