// control lateral components: nick, roll, yaw

#include "ctrl_lateral.h"

#include <mavlink.h>
#include "core/datacenter.h"
#include "core/protocolstack.h"

#include <cstdlib>
#include <sstream>

#include <math.h>

#define RAND_MAX_TO_M1 1/(double)RAND_MAX

using namespace std;

namespace mavhub {
	Ctrl_Lateral::Ctrl_Lateral(const map<string, string> args) : AppLayer("ctrl_lateral") {
		read_conf(args);
		param_request_list = 0;
		prm_test_nick = 0;
		prm_yaw_P = 100.0;
		pid_yaw = new PID(0, params["yaw_Kc"], params["yaw_Ti"],
											params["yaw_Td"]);
		pid_nick = new PID(0, params["nick_Kc"], params["nick_Ti"],
											params["nick_Td"]);
		pid_roll = new PID(0, params["roll_Kc"], params["roll_Ti"],
											params["roll_Td"]);
	}

	Ctrl_Lateral::~Ctrl_Lateral() {
	}

  void Ctrl_Lateral::handle_input(const mavlink_message_t &msg) {
		static int8_t param_id[15];
		//Logger::log("Ctrl_Lateral got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_DEBUG);
		switch(msg.msgid) {
		case MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION:
			//Logger::log("Ctrl_Lateral: got visual_navigation msg", Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_visual_navigation_decode(&msg, (mavlink_huch_visual_navigation_t *)&huch_visual_navigation);
			//Logger::log("psi_est:", huch_visual_navigation.psi_estimate, Logger::LOGLEVEL_INFO);
			break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Ctrl_Lateral::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == owner()->system_id()) {
				param_request_list = 1;
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			if(mavlink_msg_param_set_get_target_system(&msg) == owner()->system_id()) {
				Logger::log("Ctrl_Lateral::handle_input: PARAM_SET for this system", (int)owner()->system_id(), Logger::LOGLEVEL_INFO);
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

					// if(!strcmp("prm_test_nick", (const char *)param_id)) {
					// 	prm_test_nick = (int)mavlink_msg_param_set_get_param_value(&msg);
					// 	Logger::log("Ctrl_Lateral::handle_input: PARAM_SET request for prm_test_nick", prm_test_nick, Logger::LOGLEVEL_INFO);
					// }	else if(!strcmp("prm_yaw_P", (const char *)param_id)) {
					// 	prm_yaw_P = (double)mavlink_msg_param_set_get_param_value(&msg);
					// 	Logger::log("Ctrl_Hover::handle_input: PARAM_SET request for prm_yaw_P", prm_yaw_P, Logger::LOGLEVEL_INFO);
					// }
				}
			}
			break;
		default:
			break;
		}		
	}

  void Ctrl_Lateral::run() {
		// generic
		static mavlink_message_t msg;
		static mavlink_debug_t dbg;
		// timing
		uint64_t dt = 0;
		double dtf = 0.0;
		struct timeval tk, tkm1; // timevals
		int update_rate = 10; // 100 Hz
		int wait_freq = update_rate? 1000000 / update_rate: 0;
		int wait_time = wait_freq;
		uint64_t frequency = wait_time;
		uint64_t start = get_time_us();
		uint64_t usec;

		// body variables
		double tmp;
		//int16_t nick, roll, yaw, yaw1;
		double nick, roll, yaw, yaw1;
		vector<int16_t> v(3);
		int nick_test_dur, nick_test_delay;
		int my_cnt;
		// position
		double x = 0.0, y = 0.0;

		gettimeofday(&tk, NULL);
		gettimeofday(&tkm1, NULL);

		nick = roll = yaw = yaw1 = 0;
		nick_test_dur = 0;
		nick_test_delay = 10;
		my_cnt = 0;

		Logger::log("Ctrl_Lateral started:", name(), Logger::LOGLEVEL_INFO);
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

			// Logger::log("Ctrl_Lateral slept for", wait_time, component_id, Logger::LOGLEVEL_INFO);

			gettimeofday(&tk, NULL);
			//timediff(tdiff, tkm1, tk);
			dt = (tk.tv_sec - tkm1.tv_sec) * 1000000 + (tk.tv_usec - tkm1.tv_usec);
			tkm1 = tk; // save current time
			// dt fractional in seconds
			dtf = (double)dt * 1e-6;

			// param handling
			if(param_request_list) {
				Logger::log("Ctrl_Lateral::run: param request", Logger::LOGLEVEL_INFO);
				param_request_list = 0;

				typedef map<string, double>::const_iterator ci;
				for(ci p = params.begin(); p!=params.end(); ++p) {
					// Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (const int8_t*) p->first.data(), p->second, 1, 0);
					send(msg);
				}

				// mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"prm_test_nick", prm_test_nick, 1, 0);
				// send(msg);
				// mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"prm_yaw_P", prm_yaw_P, 1, 0);
				// send(msg);
			}

			// // test huch_visual_navigation
			// huch_visual_navigation.psi_estimate = 1.234;
			// mavlink_msg_huch_visual_navigation_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &huch_visual_navigation);
			// send(msg);

			if(prm_test_nick > 0) {
				nick_test_delay--;
			}

			if(nick_test_delay == 0) {
				nick_test_dur = 2;
				prm_test_nick = 0;
				nick_test_delay = 10;
			}

			// body
			if (0) {
				tmp = rand() * RAND_MAX_TO_M1;
				tmp -= 0.5;
				tmp *= 1000;
				nick = v[0] = (int16_t)tmp;
				tmp = rand() * RAND_MAX_TO_M1;
				tmp -= 0.5;
				tmp *= 1000;
				roll = v[1] = (int16_t)tmp;
				tmp = rand() * RAND_MAX_TO_M1;
				tmp *= 1000;
				yaw = v[2] = (int16_t)tmp;
			}
			//if(nick_test_dur > 0) {
			if(0) {
				if(my_cnt % 40 == 0 || my_cnt % 40 == 1) {
					nick = 200;
					nick_test_dur--;
				} else if (my_cnt % 40 == 10 || my_cnt % 40 == 11) {
					nick = -200;
					nick_test_dur--;
				} else if (my_cnt % 40 == 20 || my_cnt % 40 == 21) {
					roll = 200;
				} else if (my_cnt % 40 == 30 || my_cnt % 40 == 31) {
					roll = -200;
				}
				else {
					nick = 0;
					roll = 0;
				}
			} else {
				nick = 0;
				roll = 0;
			}
			//roll = 0;
			// magnetic 2D compass
			// int comp = DataCenter::get_sensor(6) - 128;
			// comp = comp % 256;
			// //comp = (/255.0) * 6.28;
			// yaw = (int16_t)(prm_yaw_P * (0.0 + (comp/255.0) * 6.28));
			// Logger::log("Ctrl_Lateral (psi_est, yaw)", DataCenter::get_sensor(6), yaw, Logger::LOGLEVEL_INFO);

			// vn.ego_beta
			x = cosf(huch_visual_navigation.ego_beta);
			y = sinf(huch_visual_navigation.ego_beta);
			// vn.distance

			// FIXME: optical compass
			// yaw1 = (int16_t)(params["yaw_Kc"] * (0.0 - huch_visual_navigation.psi_vc));
			pid_yaw->setSp(0.0);
			yaw = pid_yaw->calc(dtf, (0.0 - huch_visual_navigation.visual_compass));
			//Logger::log("Ctrl_Lateral (psi_est, yaw)", huch_visual_navigation.psi_vc, yaw, Logger::LOGLEVEL_INFO);
			// Logger::log("Ctrl_Lateral (psi_est, yaw1)", huch_visual_navigation.psi_vc, yaw1, Logger::LOGLEVEL_INFO);
			//yaw = 0;
			// nick
			pid_nick->setSp(0.0);
			nick = pid_nick->calc(dtf, y * huch_visual_navigation.ego_speed);
			// limit
			if(nick > 100.0)
				nick = 100.0;
			if(nick < -100.0)
				nick = -100.0;
			// roll
			pid_roll->setSp(0.0);
			roll = pid_roll->calc(dtf, x * huch_visual_navigation.ego_speed);
			if(roll > 100.0)
				roll = 100.0;
			if(roll < -100.0)
				roll = -100.0;

			send_debug(&msg, &dbg, 100, x, component_id);
			send_debug(&msg, &dbg, 101, y, component_id);
			send_debug(&msg, &dbg, 105, nick, component_id);
			send_debug(&msg, &dbg, 106, roll, component_id);
			send_debug(&msg, &dbg, 107, atan2f(-nick, -roll), component_id);

			DataCenter::set_extctrl_nick(nick);
			DataCenter::set_extctrl_roll(roll);
			DataCenter::set_extctrl_yaw(yaw*-1.0);
			// Logger::log("Ctrl_Lateral (n,r,y)", v, Logger::LOGLEVEL_INFO);

			my_cnt++;
		}
	}

	void Ctrl_Lateral::default_conf() {
		params["yaw_Kc"] = 100.0;
		params["yaw_Ti"] = 0.0;
		params["yaw_Td"] = 0.0;
		params["nick_Kc"] = 1.0;
		params["nick_Ti"] = 0.0;
		params["nick_Td"] = 0.0;
		params["roll_Kc"] = 1.0;
		params["roll_Ti"] = 0.0;
		params["roll_Td"] = 0.0;
	}

	void Ctrl_Lateral::read_conf(const map<string, string> args) {
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

		// controller params nick
		iter = args.find("nick_Kc");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["nick_Kc"];
		}
		iter = args.find("nick_Ti");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["nick_Ti"];
		}
		iter = args.find("nick_Td");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["nick_Td"];
		}

		// controller params roll
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

		Logger::log("ctrl_lateral::read_conf: component_id", component_id, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: yaw_Kc", params["yaw_Kc"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: yaw_Ti", params["yaw_Ti"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: yaw_Td", params["yaw_Td"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: nick_Kc", params["nick_Kc"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: nick_Ti", params["nick_Ti"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: nick_Td", params["nick_Td"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: roll_Kc", params["roll_Kc"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: roll_Ti", params["roll_Ti"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lateral::read_conf: roll_Td", params["roll_Td"], Logger::LOGLEVEL_DEBUG);
	}
}
