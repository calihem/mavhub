// control zrate components: nick, roll, yaw

#include "ctrl_zrate.h"

#include <mavlink.h>
#include "core/datacenter.h"
#include "core/protocolstack.h"

#include <cstdlib>
#include <sstream>

#define RAND_MAX_TO_M1 1/(double)RAND_MAX

using namespace std;

namespace mavhub {
	Ctrl_Zrate::Ctrl_Zrate(const map<string, string> args) : AppLayer("ctrl_zrate") {
		read_conf(args);
		param_request_list = 0;
	}

	Ctrl_Zrate::~Ctrl_Zrate() {
	}

  void Ctrl_Zrate::handle_input(const mavlink_message_t &msg) {
		static int8_t param_id[15];
		//Logger::log("Ctrl_Zrate got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_DEBUG);
		switch(msg.msgid) {
		case MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION:
			//Logger::log("Ctrl_Zrate: got visual_navigation msg", Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_visual_navigation_decode(&msg, (mavlink_huch_visual_navigation_t *)&huch_visual_navigation);
			//Logger::log("psi_est:", huch_visual_navigation.psi_estimate, Logger::LOGLEVEL_INFO);
			break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Ctrl_Zrate::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == owner()->system_id()) {
				param_request_list = 1;
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			if(mavlink_msg_param_set_get_target_system(&msg) == owner()->system_id()) {
				Logger::log("Ctrl_Zrate::handle_input: PARAM_SET for this system", (int)owner()->system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Ctrl_Zrate::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log("Ctrl_Zrate::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);

					typedef map<string, double>::const_iterator ci;
					for(ci p = params.begin(); p!=params.end(); ++p) {
						// Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
						if(!strcmp(p->first.data(), (const char *)param_id)) {
							params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
							Logger::log("x Ctrl_Zrate::handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_INFO);
						}
					}

				}
			}
			break;
		default:
			break;
		}		
	}

  void Ctrl_Zrate::run() {
		// generic
		static mavlink_message_t msg;
		// timing
		uint64_t dt = 0;
		struct timeval tk, tkm1; // timevals
		int update_rate = 10; // 100 Hz
		int wait_freq = update_rate? 1000000 / update_rate: 0;
		int wait_time = wait_freq;
		uint64_t frequency = wait_time;
		uint64_t start = get_time_us();
		uint64_t usec;

		// body variables
		double tmp;
		int16_t nick, roll, yaw;
		vector<int16_t> v(3);
		int nick_test_dur, nick_test_delay;
		int my_cnt;

		gettimeofday(&tk, NULL);
		gettimeofday(&tkm1, NULL);

		nick = roll = yaw = 0;
		nick_test_dur = 0;
		nick_test_delay = 10;
		my_cnt = 0;

		Logger::log("Ctrl_Zrate started:", name(), Logger::LOGLEVEL_INFO);
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

			// Logger::log("Ctrl_Zrate slept for", wait_time, component_id, Logger::LOGLEVEL_INFO);

			gettimeofday(&tk, NULL);
			//timediff(tdiff, tkm1, tk);
			dt = (tk.tv_sec - tkm1.tv_sec) * 1000000 + (tk.tv_usec - tkm1.tv_usec);
			tkm1 = tk; // save current time

			// param handling
			if(param_request_list) {
				Logger::log("Ctrl_Zrate::run: param request", Logger::LOGLEVEL_INFO);
				param_request_list = 0;

				typedef map<string, double>::const_iterator ci;
				for(ci p = params.begin(); p!=params.end(); ++p) {
					// Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (const int8_t*) p->first.data(), p->second, 1, 0);
					send(msg);
				}
			}

			// // test huch_visual_navigation
			// huch_visual_navigation.psi_estimate = 1.234;
			// mavlink_msg_huch_visual_navigation_encode(owner()->system_id(), static_cast<uint8_t>(component_id), &msg, &huch_visual_navigation);
			// send(msg);


			// FIXME: optical compass
			// yaw = (int16_t)(prm_yaw_P * (0.0 - huch_visual_navigation.psi_vc));
			// Logger::log("Ctrl_Zrate (psi_est, yaw)", huch_visual_navigation.psi_vc, yaw, Logger::LOGLEVEL_INFO);
			//yaw = 0;

			// DataCenter::set_extctrl_nick(nick);
			// DataCenter::set_extctrl_roll(roll);
			// DataCenter::set_extctrl_yaw(yaw);
			// Logger::log("Ctrl_Zrate (n,r,y)", v, Logger::LOGLEVEL_INFO);

			my_cnt++;
		}
	}

	void Ctrl_Zrate::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}

		Logger::log("ctrl_zrate::read_conf: component_id", component_id, Logger::LOGLEVEL_DEBUG);
	}
}
