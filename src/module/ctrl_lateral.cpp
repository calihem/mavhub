// control lateral components: pitch, roll, yaw

#include "ctrl_lateral.h"

#ifdef HAVE_MAVLINK_H

#include "core/datacenter.h"
#include "protocol/protocolstack.h"
#include "lib/hub/time.h"

#include <cstdlib>
#include <sstream>

#include <math.h>

#define RAND_MAX_TO_M1 1/(double)RAND_MAX

using namespace std;
using namespace hub;

namespace mavhub {
	Ctrl_Lateral::Ctrl_Lateral(const map<string, string> args) :
		AppInterface("ctrl_lateral"),
		ModuleBase(args, "ctrl_lateral") {
		read_conf(args);
		param_request_list = 0;
		prm_test_pitch = 0;
		prm_yaw_P = 100.0;
		pid_yaw = new PID(0, params["yaw_Kc"], params["yaw_Ti"],
											params["yaw_Td"]);
		pid_pitch = new PID(params["pitch_bias"], params["pitch_Kc"],
											 params["pitch_Ti"], params["pitch_Td"]);
		pid_roll = new PID(params["roll_bias"], params["roll_Kc"],
											 params["roll_Ti"], params["roll_Td"]);

		// init sensor_array structure
		for(int i = 0; i < 16; i++)
			sensor_array.data[i] = 0.;

		// init execution timer
		Logger::log("Ctrl_Lateral ctl_update_rate", (int)params["ctl_update_rate"], Logger::LOGLEVEL_DEBUG);
		exec_tmr = new Exec_Timing((int)params["ctl_update_rate"]);
	}

	Ctrl_Lateral::~Ctrl_Lateral() {
	}

  void Ctrl_Lateral::handle_input(const mavlink_message_t &msg) {
		static char param_id[16];
		int rc2;
		int rc5;
		//Logger::log("Ctrl_Lateral got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_DEBUG);
		switch(msg.msgid) {

#ifdef MAVLINK_ENABLED_HUCH

		case MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION:
			//Logger::log("Ctrl_Lateral: got visual_navigation msg", Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_visual_navigation_decode(&msg, (mavlink_huch_visual_navigation_t *)&huch_visual_navigation);
			//Logger::log("psi_est:", huch_visual_navigation.psi_estimate, Logger::LOGLEVEL_INFO);
			break;

		case MAVLINK_MSG_ID_HUCH_VISUAL_FLOW:
			// Logger::log("Ctrl_Lateral: got visual_flow msg", Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_visual_flow_decode(&msg, (mavlink_huch_visual_flow_t *)&huch_visual_flow);
			//Logger::log("psi_est:", huch_visual_flow.psi_estimate, Logger::LOGLEVEL_INFO);
			break;

		case MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY:
			// Logger::log("Ctrl_Lateral: got visual_flow msg", Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_sensor_array_decode(&msg, (mavlink_huch_sensor_array_t *)&sensor_array);
			//Logger::log("psi_est:", huch_visual_flow.psi_estimate, Logger::LOGLEVEL_INFO);
			break;

		case MAVLINK_MSG_ID_HUCH_ACTION:
			Logger::log(name(), "handle_input: action request", (int)mavlink_msg_huch_action_get_target(&msg), system_id(), Logger::LOGLEVEL_DEBUG);
			if( (mavlink_msg_huch_action_get_target(&msg) == system_id()) ) {
				// 			&& (mavlink_msg_action_get_target_component(&msg) == component_id) ) {
				uint8_t action_id = mavlink_msg_huch_action_get_action(&msg);
				// if(action_id == MAV_ACTION_GET_IMAGE) {
				// 	Lock sync_lock(sync_mutex);
				// 	// new image with ACK
				// 	take_new_image = 3;
				// }
				switch(action_id) {
				case ACTION_TOGGLE_LC:
					params["reset_i"] = 1.;
					Logger::log(name(), "action done: reset_i", params["reset_i"], Logger::LOGLEVEL_DEBUG);
				default:
					break;
				}
			}
			break;

#endif // MAVLINK_ENABLED_HUCH	

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                  Logger::log("Ctrl_Lateral::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
                  if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
                    param_request_list = 1;
                  }
                  break;
		case MAVLINK_MSG_ID_PARAM_SET:
                  if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
                    Logger::log("Ctrl_Lateral::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
                    if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
                      Logger::log("Ctrl_Lateral::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
                      mavlink_msg_param_set_get_param_id(&msg, param_id);
                      Logger::log("Ctrl_Lateral::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);

                      typedef map<string, double>::const_iterator ci;
                      for(ci p = params.begin(); p!=params.end(); ++p) {
                        // Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
                        if(!strcmp(p->first.data(), (const char *)param_id)) {
                          params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
                          Logger::log("x Ctrl_Lateral::handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_INFO);
                        }
                      }

                      // update PID controllers
                      pid_pitch->setKc(params["pitch_Kc"]);
                      pid_roll->setKc(params["roll_Kc"]);
                      pid_pitch->setTi(params["pitch_Ti"]);
                      pid_roll->setTi(params["roll_Ti"]);
                      pid_pitch->setTd(params["pitch_Td"]);
                      pid_roll->setTd(params["roll_Td"]);
                      pid_pitch->setBias(params["pitch_bias"]);
                      pid_roll->setBias(params["roll_bias"]);
                      pid_pitch->setSp(params["pitch_sp"]);
                      pid_roll->setSp(params["roll_sp"]);

                      // if(!strcmp("prm_test_pitch", (const char *)param_id)) {
                      // 	prm_test_pitch = (int)mavlink_msg_param_set_get_param_value(&msg);
                      // 	Logger::log("Ctrl_Lateral::handle_input: PARAM_SET request for prm_test_pitch", prm_test_pitch, Logger::LOGLEVEL_INFO);
                      // }	else if(!strcmp("prm_yaw_P", (const char *)param_id)) {
                      // 	prm_yaw_P = (double)mavlink_msg_param_set_get_param_value(&msg);
                      // 	Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for prm_yaw_P", prm_yaw_P, Logger::LOGLEVEL_INFO);
                      // }
                    }
                  }
                  break;

		case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
			rc2 = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
			rc5 = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
			if (rc2 > 1700 && rc5 > 1700) 
				params["reset_i"] = 2.0;
			break;

		default:
			break;
		}		
	}

  void Ctrl_Lateral::run() {
		// generic
		static mavlink_message_t msg;
		//static mavlink_debug_t dbg;
		// timing
		uint64_t dt = 0;
		double dtf = 0.0;
		// struct timeval tk, tkm1; // timevals
		// int ctl_update_rate = (int)params["ctl_update_rate"]; // 100; // 100 Hz
		// int wait_freq = ctl_update_rate ? 1000000 / ctl_update_rate: 0;
		// int wait_time = wait_freq;
		// uint64_t frequency = wait_time;
		// uint64_t start = get_time_us();
		// uint64_t usec;
		int sleeptime;

		// body variables
		double tmp;
		//int16_t pitch, roll, yaw, yaw1;
		double pitch, roll, yaw, yaw1;
		vector<int16_t> v(3);
		int pitch_test_dur, pitch_test_delay;
		int my_cnt;
		// position
		double x = 0.0, y = 0.0;

		// gettimeofday(&tk, NULL);
		// gettimeofday(&tkm1, NULL);

		pitch = roll = yaw = yaw1 = 0;
		pitch_test_dur = 0;
		pitch_test_delay = 10;
		my_cnt = 0;

		pid_pitch->setSp(0.0);
		pid_roll->setSp(0.0);

		Logger::log("Ctrl_Lateral started:", name(), Logger::LOGLEVEL_INFO);
		while(true) {

			// run method exec timing stuff
			sleeptime = exec_tmr->calcSleeptime();
			//Logger::log("plat_link_mk.cpp: sleeptime: ", sleeptime, Logger::LOGLEVEL_INFO);
			usleep(sleeptime);
			dt = exec_tmr->updateExecStats();

			// /* wait time */
			// usec = get_time_us();
			// uint64_t end = usec;
			// wait_time = wait_freq - (end - start);
			// wait_time = (wait_time < 0)? 0: wait_time;
		
			// /* wait */
			// usleep(wait_time);
			// //usleep(10);

			// /* calculate frequency */
			// end = get_time_us();
			// frequency = (15 * frequency + end - start) / 16;
			// start = end;

			// // Logger::log("Ctrl_Lateral slept for", wait_time, component_id, Logger::LOGLEVEL_INFO);

			// gettimeofday(&tk, NULL);
			// //timediff(tdiff, tkm1, tk);
			// dt = (tk.tv_sec - tkm1.tv_sec) * 1000000 + (tk.tv_usec - tkm1.tv_usec);
			// tkm1 = tk; // save current time
			// // dt fractional in seconds
			// dtf = (double)dt * 1e-6;

			// dt fractional in seconds
			dtf = (double)dt * 1e-6;

			// param handling
			if(param_request_list) {
				Logger::log("Ctrl_Lateral::run: param request", Logger::LOGLEVEL_INFO);
				param_request_list = 0;

				typedef map<string, double>::const_iterator ci;
				for(ci p = params.begin(); p!=params.end(); ++p) {
					// Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_value_pack(system_id(), component_id, &msg, (const char*) p->first.data(), p->second, MAVLINK_TYPE_FLOAT, 1, 0);
					send(msg);
				}

				// mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"prm_test_pitch", prm_test_pitch, 1, 0);
				// send(msg);
				// mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"prm_yaw_P", prm_yaw_P, 1, 0);
				// send(msg);
			}

			if(params["reset_i"] > 0.0) {
				params["reset_i"] = 0.0;
				pid_pitch->setIntegral(0.0);
				pid_roll->setIntegral(0.0);
			}

			// // test huch_visual_navigation
			// huch_visual_navigation.psi_estimate = 1.234;
			// mavlink_msg_huch_visual_navigation_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &huch_visual_navigation);
			// send(msg);

			if(prm_test_pitch > 0) {
				pitch_test_delay--;
			}

			if(pitch_test_delay == 0) {
				pitch_test_dur = 2;
				prm_test_pitch = 0;
				pitch_test_delay = 10;
			}

			// body
			if (0) {
				tmp = rand() * RAND_MAX_TO_M1;
				tmp -= 0.5;
				tmp *= 1000;
				pitch = v[0] = (int16_t)tmp;
				tmp = rand() * RAND_MAX_TO_M1;
				tmp -= 0.5;
				tmp *= 1000;
				roll = v[1] = (int16_t)tmp;
				tmp = rand() * RAND_MAX_TO_M1;
				tmp *= 1000;
				yaw = v[2] = (int16_t)tmp;
			}
			//if(pitch_test_dur > 0) {
			if(0) {
				if(my_cnt % 40 == 0 || my_cnt % 40 == 1) {
					pitch = 200;
					pitch_test_dur--;
				} else if (my_cnt % 40 == 10 || my_cnt % 40 == 11) {
					pitch = -200;
					pitch_test_dur--;
				} else if (my_cnt % 40 == 20 || my_cnt % 40 == 21) {
					roll = 200;
				} else if (my_cnt % 40 == 30 || my_cnt % 40 == 31) {
					roll = -200;
				}
				else {
					pitch = 0;
					roll = 0;
				}
			} else {
				pitch = 0;
				roll = 0;
			}
			//roll = 0;
			// magnetic 2D compass
			// int comp = DataCenter::get_sensor(6) - 128;
			// comp = comp % 256;
			// //comp = (/255.0) * 6.28;
			// yaw = (int16_t)(prm_yaw_P * (0.0 + (comp/255.0) * 6.28));
			// Logger::log("Ctrl_Lateral (psi_est, yaw)", DataCenter::get_sensor(6), yaw, Logger::LOGLEVEL_INFO);
#ifdef MAVLINK_ENABLED_HUCH
			// vn.ego_beta
			// x = cosf(huch_visual_navigation.ego_beta);
			// y = sinf(huch_visual_navigation.ego_beta);
			x = huch_visual_flow.u_i;
			y = huch_visual_flow.v_i;
			// vn.distance

			// FIXME: optical compass
			// yaw1 = (int16_t)(params["yaw_Kc"] * (0.0 - huch_visual_navigation.psi_vc));
			pid_yaw->setSp(0.0);
			yaw = pid_yaw->calc(dtf, (0.0 - huch_visual_navigation.visual_compass));
			//Logger::log("Ctrl_Lateral (psi_est, yaw)", huch_visual_navigation.psi_vc, yaw, Logger::LOGLEVEL_INFO);
			// Logger::log("Ctrl_Lateral (psi_est, yaw1)", huch_visual_navigation.psi_vc, yaw1, Logger::LOGLEVEL_INFO);
			//yaw = 0;
			// pitch
			// pid_pitch->setSp(0.0);
			// pitch = pid_pitch->calc(dtf, y * huch_visual_navigation.ego_speed);
			pitch = pid_pitch->calc(dtf, x);
			// limit
			if(pitch > params["pitch_limit"])
				pitch = params["pitch_limit"];
			if(pitch < -params["pitch_limit"])
				pitch = -params["pitch_limit"];

			// // debug pitch PID
			// send_debug(&msg, &dbg, 8, pid_pitch->getErr());
			// send_debug(&msg, &dbg, 9, pid_pitch->getIpart());
			// send_debug(&msg, &dbg, 10, pid_pitch->getDpart());
			// send_debug(&msg, &dbg, 11, dtf);

			send_debug(&msg, &dbg, 1, params["pitch_Kc"]);

			// roll
			// pid_roll->setSp(0.0);
			// roll = pid_roll->calc(dtf, x * huch_visual_navigation.ego_speed);
			roll = pid_roll->calc(dtf, y);
			if(roll > params["roll_limit"])
				roll = params["roll_limit"];
			if(roll < -params["roll_limit"])
				roll = -params["roll_limit"];


			// if module v_oflow_odca is active, add its opinion as well
			if(sensor_array.data[0] != 0.) {
				Logger::log(name(), sensor_array.data[8], sensor_array.data[9], Logger::LOGLEVEL_INFO);
				pitch -= sensor_array.data[8] * params["pitch_obst"];
				roll  += sensor_array.data[9] * params["roll_obst"];
			}

#endif // MAVLINK_ENABLED_HUCH	

			// mavlink_msg_debug_pack(system_id(), component_id, &msg, 100, x);
			// AppLayer<mavlink_message_t>::send(msg);
			// mavlink_msg_debug_pack(system_id(), component_id, &msg, 101, y);
			// AppLayer<mavlink_message_t>::send(msg);
			// mavlink_msg_debug_pack(system_id(), component_id, &msg, 105, pitch);
			// AppLayer<mavlink_message_t>::send(msg);
			// mavlink_msg_debug_pack(system_id(), component_id, &msg, 106, roll);
			// AppLayer<mavlink_message_t>::send(msg);
			// mavlink_msg_debug_pack( system_id(), component_id, &msg, 107, atan2f(-pitch, -roll) );
			// AppLayer<mavlink_message_t>::send(msg);
	
			//send_debug(&msg, &dbg, 100, x, component_id);
			//send_debug(&msg, &dbg, 101, y, component_id);
			//send_debug(&msg, &dbg, 105, pitch, component_id);
			//send_debug(&msg, &dbg, 106, roll, component_id);
			//send_debug(&msg, &dbg, 107, atan2f(-pitch, -roll), component_id);

			DataCenter::set_extctrl_pitch(pitch);
			DataCenter::set_extctrl_roll(roll);

			chan.usec = get_time_us();
			chan.index = CHAN_PITCH;    
			chan.value = pitch;
			mavlink_msg_huch_generic_channel_encode(system_id(),
																							component_id,
																							&msg,
																							&chan);
			AppLayer<mavlink_message_t>::send(msg);

			chan.usec = get_time_us();
			chan.index = CHAN_ROLL;
			chan.value = roll;
			mavlink_msg_huch_generic_channel_encode(system_id(),
																							component_id,
																							&msg,
																							&chan);
			AppLayer<mavlink_message_t>::send(msg);

			// DataCenter::set_extctrl_yaw(yaw*-1.0);
			// Logger::log("Ctrl_Lateral (n,r,y)", v, Logger::LOGLEVEL_INFO);
			// Logger::log("Ctrl_Lateral (x,y)", x, y, Logger::LOGLEVEL_INFO);
			// Logger::log("Ctrl_Lateral (n,r,y)", v, Logger::LOGLEVEL_INFO);

			my_cnt++;
		}
	}

	void Ctrl_Lateral::default_conf() {
		params["yaw_Kc"] = 100.0;
		params["yaw_Ti"] = 0.0;
		params["yaw_Td"] = 0.0;
		params["pitch_Kc"] = 1.0;
		params["pitch_Ti"] = 0.0;
		params["pitch_Td"] = 0.0;
		params["pitch_sp"] = 0.0;
		params["roll_Kc"] = 1.0;
		params["roll_Ti"] = 0.0;
		params["roll_Td"] = 0.0;
		params["roll_sp"] = 0.0;
		params["roll_obst"] = 20.0;
		params["pitch_obst"] = 20.0;
	}

	void Ctrl_Lateral::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}

		// update rate
		iter = args.find("ctl_update_rate");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_update_rate"];
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

		// controller params pitch
		iter = args.find("pitch_bias");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["pitch_bias"];
		}
		iter = args.find("pitch_Kc");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["pitch_Kc"];
		}
		iter = args.find("pitch_Ti");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["pitch_Ti"];
		}
		iter = args.find("pitch_Td");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["pitch_Td"];
		}
		iter = args.find("pitch_limit");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["pitch_limit"];
		}
		iter = args.find("pitch_sp");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["pitch_sp"];
		}

		// controller params roll
		iter = args.find("roll_bias");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["roll_bias"];
		}
		iter = args.find("roll_Kc");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["roll_Kc"];
		}
		iter = args.find("roll_Ti");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["roll_Ti"];
		}
		iter = args.find("roll_Td");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["roll_Td"];
		}
		iter = args.find("roll_limit");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["roll_limit"];
		}
		iter = args.find("roll_sp");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["roll_sp"];
		}

		iter = args.find("reset_i");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["reset_i"];
		}
		else {
			params["reset_i"] = 0.0;
		}

		Logger::log("ctrl_lateral::read_conf: component_id", component_id, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: yaw_Kc", params["yaw_Kc"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: yaw_Ti", params["yaw_Ti"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: yaw_Td", params["yaw_Td"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: pitch_bias", params["pitch_bias"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: pitch_Kc", params["pitch_Kc"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: pitch_Ti", params["pitch_Ti"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: pitch_Td", params["pitch_Td"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: pitch_limit", params["pitch_limit"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: roll_bias", params["roll_bias"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: roll_Kc", params["roll_Kc"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: roll_Ti", params["roll_Ti"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: roll_Td", params["roll_Td"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: roll_limit", params["roll_limit"], Logger::LOGLEVEL_DEBUG);
	}
}

#endif // HAVE_MAVLINK_H

