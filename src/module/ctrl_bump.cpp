// talk to FC with mkpackage
#include "ctrl_bump.h"

#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MKLINK_H

#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //us

#include <sstream>

#include "core/logger.h"
#include "utility.h"
#include "protocol/protocolstack.h"
#include "protocol/mkpackage.h"
#include "core/datacenter.h"

using namespace std;

namespace mavhub {
	// Ctrl_Bump::Ctrl_Bump(int component_id_, int numchan_, const list<pair<int, int> > chanmap_, const map<string, string> args) {
  Ctrl_Bump::Ctrl_Bump(const map<string, string> args) :
	AppInterface("ctrl_bump"),
	AppLayer<mavlink_message_t>("ctrl_bump"),
	AppLayer<mk_message_t>("ctrl_bump") {
		read_conf(args);
		// component_id = component_id_;
		param_count = 1;
		gdt_enable = 0;
		gdt_t0 = 0;
		gdt_delay = 3000000;
		gdt_gas = 200.0;
  }

  Ctrl_Bump::~Ctrl_Bump() {
	}

  void Ctrl_Bump::handle_input(const mavlink_message_t &msg) {
		// static vector<int> v(16);
		static int8_t param_id[15];

		switch(msg.msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Ctrl_Bump::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
				param_list();
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			Logger::log("Ctrl_Bump::handle_input: PARAM_SET", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
				Logger::log("Ctrl_Bump::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Ctrl_Bump::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log("Ctrl_Bump::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);
					if(!strcmp("gdt_enable", (const char *)param_id)) {
						gdt_enable = (int)mavlink_msg_param_set_get_param_value(&msg);
						gdt_t0 = get_time_us();
						Logger::log("Ctrl_Bump::handle_input: PARAM_SET request for gdt_enable", gdt_enable, Logger::LOGLEVEL_INFO);
					}	else if(!strcmp("gdt_delay", (const char *)param_id)) {
						gdt_delay = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Bump::handle_input: PARAM_SET request for gdt_delay", gdt_delay, Logger::LOGLEVEL_INFO);
					}	else if(!strcmp("gdt_gas", (const char *)param_id)) {
						gdt_gas = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Bump::handle_input: PARAM_SET request for gdt_gas", gdt_gas, Logger::LOGLEVEL_INFO);
					}	else if(!strcmp("output_enable", (const char *)param_id)) {
						output_enable = (double)mavlink_msg_param_set_get_param_value(&msg);
						Logger::log("Ctrl_Bump::handle_input: PARAM_SET request for output_enable", output_enable, Logger::LOGLEVEL_INFO);
					}	
				}
			}
			break;
		default:
			break;
		}
  }

  void Ctrl_Bump::handle_input(const mk_message_t &msg) { }

	void Ctrl_Bump::param_list() {
		static mavlink_message_t msg;
		// return params list
		Logger::log("Ctrl_Bump::param_list", Logger::LOGLEVEL_INFO);
		mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"gdt_enable", gdt_enable, 1, 0);
		AppLayer<mavlink_message_t>::send(msg);
		mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"gdt_delay", gdt_delay, 1, 0);
		AppLayer<mavlink_message_t>::send(msg);
		mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"gdt_gas", gdt_gas, 1, 0);
		AppLayer<mavlink_message_t>::send(msg);
		mavlink_msg_param_value_pack(system_id(), component_id, &msg, (int8_t *)"output_enable", output_enable, 1, 0);
		AppLayer<mavlink_message_t>::send(msg);
	}

	double Ctrl_Bump::gdt_eval(uint64_t dt) {
		double gas = 0.0;
		uint64_t t0;
		t0 = get_time_us();
		if(t0 > (gdt_t0 + gdt_delay)
			 && t0 < (gdt_t0 + (2 * gdt_delay))) {
			gas = gdt_gas;
		}
		else if(t0 > (gdt_t0 + (2 * gdt_delay))
						&& t0 < (gdt_t0 + (3 * gdt_delay))) {
			gas = gdt_gas - 15;
		}
		else if (t0 > (gdt_t0 + (3 * gdt_delay))) {
			gdt_enable = 0;
			gas = 0.0;
		}
		return gas;
	}

  void Ctrl_Bump::run() {
		// int buf[1];
		uint8_t flags = 0;
		uint64_t dt = 0;
		struct timeval tk, tkm1; // timevals
		ostringstream o;
		static mavlink_message_t msg;
		static mavlink_debug_t dbg;
		// strapdown matrix
		int run_cnt = 0;
		double gas;

		// more timing
		int update_rate = 100; // 100 Hz
		int wait_freq = update_rate? 1000000 / update_rate: 0;
		int wait_time = wait_freq;

		uint64_t frequency = wait_time;
		uint64_t start = get_time_us();
		//uint64_t time_output = start + 1000000;
		uint64_t usec;
		
		gettimeofday(&tk, NULL);
		gettimeofday(&tkm1, NULL);
		
		// rel
		//flags |= (APFLAG_GENERAL_ON | APFLAG_KEEP_VALUES | APFLAG_HEIGHT_CTRL1 );
		// abs
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

		Logger::log("Ctrl_Bump started", Logger::LOGLEVEL_INFO);

		// MKPackage msg_setneutral(1, 'c');
		// send(msg_setneutral);

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

			// Logger::log("Ctrl_Bump slept for", wait_time, Logger::LOGLEVEL_INFO);

			gettimeofday(&tk, NULL);
			//timediff(tdiff, tkm1, tk);
			dt = (tk.tv_sec - tkm1.tv_sec) * 1000000 + (tk.tv_usec - tkm1.tv_usec);
			tkm1 = tk; // save current time

			if(gdt_enable)
				// XXX: limit
				gas = gdt_eval(dt * 1e-6);
			else
				gas = 0.0;

			// 4. run controller
			// gas = 123.0; //

			// enforce more limits
			// if(!ctl_sticksp) {
			// 	if(gas > (manual_control.thrust * 4)) { // 4 <- stick_gain
			// 		pid_alt->setIntegralM1();
			// 		gas = manual_control.thrust * 4;
			// 	}
			// }

			// if(manual_control.thrust < 5) // reset below threshold
			// 	pid_alt->setIntegral(0.0);

			extctrl.gas = (int16_t)gas;
			//extctrl.gas = 255 * (double)rand()/RAND_MAX;

			// gas out
			dbg.ind = BUMP_GAS;
			dbg.value = gas;
			mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			AppLayer<mavlink_message_t>::send(msg);

			// gdt enable
			dbg.ind = BUMP_ENABLE;
			dbg.value = gdt_enable;
			mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			AppLayer<mavlink_message_t>::send(msg);

			// dt
			dbg.ind = BUMP_DT_S;
			dbg.value = dt * 1e-6;
			mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &dbg);
			AppLayer<mavlink_message_t>::send(msg);

			//Logger::log("Ctrl_Bump: ctl out", extctrl.gas, Logger::LOGLEVEL_INFO);
			if(output_enable > 0) {
				mk_message_t msg_extctrl;
				mklink_msg_pack(&msg_extctrl, MK_FC_ADDRESS, MK_MSG_TYPE_SET_EXT_CTRL, &extctrl, sizeof(extctrl));
				AppLayer<mk_message_t>::send(msg_extctrl);
			}
			
			// stats
			run_cnt += 1;
			// XXX: usleep call takes ~5000 us?
			//usleep(10000);
		}
		return;
  }

	void Ctrl_Bump::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;
		Logger::log("Ctrl_Bump::read_conf", Logger::LOGLEVEL_INFO);

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}

		// iter = args.find("ctl_bias");
		// if( iter != args.end() ) {
		// 	istringstream s(iter->second);
		// 	s >> ctl_bias;
		// }

		// iter = args.find("ctl_Kc");
		// if( iter != args.end() ) {
		// 	istringstream s(iter->second);
		// 	s >> ctl_Kc;
		// }

		// iter = args.find("ctl_Ti");
		// if( iter != args.end() ) {
		// 	istringstream s(iter->second);
		// 	s >> ctl_Ti;
		// }

		// iter = args.find("ctl_Td");
		// if( iter != args.end() ) {
		// 	istringstream s(iter->second);
		// 	s >> ctl_Td;
		// }

		// iter = args.find("ctl_sp");
		// if( iter != args.end() ) {
		// 	istringstream s(iter->second);
		// 	s >> ctl_sp;
		// }

		// iter = args.find("ctl_bref");
		// if( iter != args.end() ) {
		// 	istringstream s(iter->second);
		// 	s >> ctl_bref;
		// }

		// iter = args.find("ctl_sticksp");
		// if( iter != args.end() ) {
		// 	istringstream s(iter->second);
		// 	s >> ctl_sticksp;
		// }

		// outout enable
		iter = args.find("output_enable");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> output_enable;
		}

		// XXX
		Logger::log("ctrl_bump::read_conf: component_id", component_id, Logger::LOGLEVEL_INFO);
		Logger::log("ctrl_bump::read_conf: output_enable", output_enable, Logger::LOGLEVEL_INFO);
		// Logger::log("ctrl_bump::read_conf: ctl_bias", ctl_bias, Logger::LOGLEVEL_INFO);
		// Logger::log("ctrl_bump::read_conf: ctl_Kc", ctl_Kc, Logger::LOGLEVEL_INFO);
		// Logger::log("ctrl_bump::read_conf: ctl_Ti", ctl_Ti, Logger::LOGLEVEL_INFO);
		// Logger::log("ctrl_bump::read_conf: ctl_Td", ctl_Td, Logger::LOGLEVEL_INFO);
		// Logger::log("ctrl_bump::read_conf: ctl_sp", ctl_sp, Logger::LOGLEVEL_INFO);
		// Logger::log("ctrl_bump::read_conf: ctl_bref", ctl_bref, Logger::LOGLEVEL_INFO);
		// Logger::log("ctrl_bump::read_conf: ctl_sticksp", ctl_sticksp, Logger::LOGLEVEL_INFO);

		return;
	}
}

#endif // HAVE_MKLINK_H
#endif // HAVE_MAVLINK_H

