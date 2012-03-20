// control zrate components: pitch, roll, yaw

#include "ctrl_zrate.h"

#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MKLINK_H

#include "core/datacenter.h"
#include "protocol/protocolstack.h"
#include "qk_helper.h"
#include "protocol/mkpackage.h"

#include <cstdlib>
#include <sstream>

#define RAND_MAX_TO_M1 1/(double)RAND_MAX

using namespace std;

namespace mavhub {
	Ctrl_Zrate::Ctrl_Zrate(const map<string, string> args) : 
		AppInterface("ctrl_zrate"),
		AppLayer<mavlink_message_t>("ctrl_zrate"),
		AppLayer<mk_message_t>("ctrl_zrate") {
		read_conf(args);
		param_request_list = 0;
		pid_zrate = new PID((int)params["ctl_bias"],
												params["ctl_Kc"], params["ctl_Ti"],
												params["ctl_Td"]);
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
		case MAVLINK_MSG_ID_MANUAL_CONTROL:
			// Logger::log("Ctrl_Hover got huch attitude", Logger::LOGLEVEL_INFO);
			//Logger::log("Ctrl_Hover got huch_altitude [seq]:", (int)msg.seq, Logger::LOGLEVEL_INFO);
			mavlink_msg_manual_control_decode(&msg, &manual_control);
			//Logger::log("Ctrl_Zrate", (int)manual_control.target, manual_control.thrust, Logger::LOGLEVEL_INFO);
			break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Ctrl_Zrate::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
				param_request_list = 1;
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
				Logger::log("Ctrl_Zrate::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
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

  void Ctrl_Zrate::handle_input(const mk_message_t &msg) {

  }

  void Ctrl_Zrate::run() {
		// generic
		static mavlink_message_t msg;
		static mavlink_debug_t dbg;
		// timing
		uint64_t dt = 0;
		struct timeval tk, tkm1; // timevals
		int update_rate = 15; // 100 Hz
		int wait_freq = update_rate? 1000000 / update_rate: 0;
		int wait_time = wait_freq;
		uint64_t frequency = wait_time;
		uint64_t start = get_time_us();
		uint64_t usec;

		// heartbeat
		int system_type = MAV_QUADROTOR;
		mavlink_message_t msg_hb;
		mavlink_msg_heartbeat_pack(system_id(), component_id, &msg_hb, system_type, MAV_AUTOPILOT_HUCH);
		// "check in"
		//send(msg_hb);

		// body variables
		//double tmp;
		int16_t pitch, roll, yaw;
		vector<int16_t> v(3);
		int pitch_test_dur, pitch_test_delay;
		int my_cnt;

		// extctrl
		uint8_t flags = 0;
		double gas;
		extern_control_t extctrl;
		flags |= (APFLAG_GENERAL_ON | APFLAG_KEEP_VALUES | APFLAG_FULL_CTRL );
		extctrl.remote_buttons = 0;	/* for lcd menu */
		extctrl.nick = 0; //pitch;
		extctrl.roll = 0; //roll;
		extctrl.yaw = 0; //yaw;
		extctrl.gas = 0; //gas;	/* MotorGas = min(ExternControl.Gas, StickGas) */

		//extctrl.height = 0; //height;
		/* for autopilot */
		extctrl.AP_flags = flags;
		extctrl.frame = 'E';	/* get ack from flightctrl */
		extctrl.config = 0;	/* activate external control via serial iface in FlightCtrl */


		gettimeofday(&tk, NULL);
		gettimeofday(&tkm1, NULL);

		pitch = roll = yaw = 0;
		pitch_test_dur = 0;
		pitch_test_delay = 10;
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
					mavlink_msg_param_value_pack(system_id(), component_id, &msg, (const int8_t*) p->first.data(), p->second, 1, 0);
					AppLayer<mavlink_message_t>::send(msg);
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

			zrate_sp = 0.0; // (double)manual_control.thrust - 127;
			zrate_av = huch_visual_navigation.alt_velocity;
			zrate_err = zrate_sp - zrate_av;

			// plain
			// gas = params["ctl_bias"] + params["ctl_Kc"] * zrate_err;

			pid_zrate->setSp(zrate_sp);
			gas = pid_zrate->calc((double)dt * 1e-6, -zrate_av);

			if(gas > (manual_control.thrust * 4)) { // 4 <- stick_gain
				pid_zrate->setIntegralM1();
				gas = manual_control.thrust * 4;
			}

			// Logger::log("Ctrl_Zrate gas", params["ctl_bias"], params["ctl_P"], Logger::LOGLEVEL_INFO);
			// Logger::log("Ctrl_Zrate gas", zrate_err, gas, Logger::LOGLEVEL_INFO);

			// reset integral
			if(manual_control.thrust < params["ctl_mingas"]) // reset below threshold
				pid_zrate->setIntegral(0.0);

			// limit gas
			if(gas < params["ctl_mingas"])
				gas = params["ctl_mingas"];
			if(gas > params["ctl_maxgas"]) {
				gas = params["ctl_maxgas"];
				pid_zrate->setIntegralM1();
			}

			extctrl.gas = (int16_t)gas;
			// extctrl.pitch = 0; //(int16_t)DataCenter::get_extctrl_pitch();
			// extctrl.roll = 0; //(int16_t)DataCenter::get_extctrl_roll();
			// extctrl.yaw = 0; //(int16_t)DataCenter::get_extctrl_yaw();
			extctrl.nick = (int16_t)DataCenter::get_extctrl_pitch();
			extctrl.roll = (int16_t)DataCenter::get_extctrl_roll();
			extctrl.yaw = (int16_t)DataCenter::get_extctrl_yaw();

			mavlink_msg_debug_pack( system_id(), component_id, &msg, 103, DataCenter::get_extctrl_pitch() );
			AppLayer<mavlink_message_t>::send(msg);
			mavlink_msg_debug_pack( system_id(), component_id, &msg, 104, DataCenter::get_extctrl_roll() );
			AppLayer<mavlink_message_t>::send(msg);
			//send_debug(&msg, &dbg, 103, DataCenter::get_extctrl_pitch(), component_id);
			//send_debug(&msg, &dbg, 104, DataCenter::get_extctrl_roll(), component_id);

			if(params["output_enable"] > 0) {
				mk_message_t msg_extctrl;
				mklink_msg_pack(&msg_extctrl, MK_FC_ADDRESS, MK_MSG_TYPE_SET_EXT_CTRL, &extctrl, sizeof(extctrl));
				AppLayer<mk_message_t>::send(msg_extctrl);
			}

			// DataCenter::set_extctrl_pitch(pitch);
			// DataCenter::set_extctrl_roll(roll);
			// DataCenter::set_extctrl_yaw(yaw);
			// Logger::log("Ctrl_Zrate (n,r,y)", v, Logger::LOGLEVEL_INFO);

			dbg.ind = 0;
			dbg.value = dt * 1e-6; //wait_time;
			mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			AppLayer<mavlink_message_t>::send(msg);

			dbg.ind = 1;
			dbg.value = zrate_sp;
			mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			AppLayer<mavlink_message_t>::send(msg);

			dbg.ind = 2;
			dbg.value = zrate_av;
			mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			AppLayer<mavlink_message_t>::send(msg);

			dbg.ind = 3;
			dbg.value = manual_control.thrust; // zrate_err;
			mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			AppLayer<mavlink_message_t>::send(msg);

			dbg.ind = 4;
			dbg.value = gas; //extctrl.gas;
			mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			AppLayer<mavlink_message_t>::send(msg);

			if(my_cnt % 50 == 0 && params["en_heartbeat"] > 0.0)
				AppLayer<mavlink_message_t>::send(msg_hb);

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

		iter = args.find("en_heartbeat");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["en_heartbeat"];
		}

		iter = args.find("ctl_bias");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_bias"];
		}

		iter = args.find("ctl_Kc");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_Kc"];
		}

		iter = args.find("ctl_Ti");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_Ti"];
		}

		iter = args.find("ctl_Td");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_Td"];
		}

		iter = args.find("ctl_mingas");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_mingas"];
		}

		iter = args.find("ctl_maxgas");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_maxgas"];
		}

		// output enable
		iter = args.find("output_enable");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["output_enable"];
		}

		Logger::log("ctrl_zrate::read_conf: component_id", component_id, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_zrate::read_conf: en_heartbeat", params["en_heartbeat"], Logger::LOGLEVEL_DEBUG);
	}
}

#endif // HAVE_MKLINK_H
#endif // HAVE_MAVLINK_H

