// control altitude, simple version

#include "ctrl_lat_bump.h"

#ifdef HAVE_MAVLINK_H

// #include "core/datacenter.h"
#include "protocol/protocolstack.h"

#include <cstdlib>
#include <sstream>

#include <math.h>

#define RAND_MAX_TO_M1 1/(double)RAND_MAX

using namespace std;

namespace mavhub {
	Ctrl_Lat_Bump::Ctrl_Lat_Bump(const map<string, string> args) :
		AppInterface("ctrl_lat_bump"),
		ModuleBase(args, "ctrl_lat_bump")
	{
		read_conf(args);
		assign_variable_from_args(component_id);
		// assign_variable_from_args(output_enable);

		param_request_list = 0;
		// pid_roll = new PID(params["roll_bias"], params["roll_Kc"],
		// 									params["roll_Ti"], params["roll_Td"]);
		// pid_roll->setSp(params["roll_sp"]);
		// pid_roll->setPv_int_lim(params["roll_ilim"]);
		tmr = new Exec_Timing((int)params["ctl_update_rate"]);
		bump = new Bumper(0, 0.01, 0.5, 0.2);
		// bump = new Bumper(0, 0.01, 1.0);
	}

	Ctrl_Lat_Bump::~Ctrl_Lat_Bump() {
	}

  void Ctrl_Lat_Bump::handle_input(const mavlink_message_t &msg) {
		static char param_id[16];
		// static mavlink_huch_sensor_array_t sa;
		//Logger::log("Ctrl_Lat_Bump got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_DEBUG);
		switch(msg.msgid) {

#ifdef MAVLINK_ENABLED_HUCH
		// case MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY:
		// 	mavlink_msg_huch_sensor_array_decode(&msg, &sa);
		// 	// m1 = sa.data[0];
		// 	// m2 = sa.data[1];
		// 	z_hat = (sa.data[0] + sa.data[1]) * 0.5;
		// 	// Logger::log(name(), "z_hat: ", z_hat, Logger::LOGLEVEL_DEBUG);
		// 	break;
#endif // MAVLINK_ENABLED_HUCH	

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Ctrl_Lat_Bump::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
				param_request_list = 1;
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			Logger::log("Ctrl_Lat_Bump::handle_input: PARAM_SET", (int)system_id(), Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
				Logger::log("Ctrl_Lat_Bump::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Ctrl_Lat_Bump::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log("Ctrl_Lat_Bump::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);

					typedef map<string, double>::const_iterator ci;
					for(ci p = params.begin(); p!=params.end(); ++p) {
						// Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
						if(!strcmp(p->first.data(), (const char *)param_id)) {
							params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
							Logger::log("x Ctrl_Lat_Bump::handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_INFO);
						}
					}

					// update PID controllers
					// pid_roll->setKc(params["roll_Kc"]);
					// pid_roll->setTi(params["roll_Ti"]);
					// pid_roll->setTd(params["roll_Td"]);
					// pid_roll->setBias(params["roll_bias"]);
					// pid_roll->setSp(params["roll_sp"]);

					// output_enable = static_cast<int>(params["output_enable"]);
				}
			}
			break;

		case MAVLINK_MSG_ID_HUCH_ACTION:
			Logger::log(name(), "::handle_input: received action request", (int)msg.sysid, (int)msg.compid, Logger::LOGLEVEL_INFO);
			if(mavlink_msg_huch_action_get_target(&msg) == system_id()){
				if(mavlink_msg_huch_action_get_target_component(&msg) == component_id) {
					switch(mavlink_msg_huch_action_get_action(&msg)) {
					case ACTION_BUMP_ROLL:
						bump->bump(0.0);
						break;
					default:
						break;
					}
				}
			}

		default:
			break;
		}		
	}

  void Ctrl_Lat_Bump::run() {
		// generic
		static mavlink_message_t msg;

		// timing
		uint64_t dt = 0;
		double dtf = 0.0;
		int sleeptime;
		// double roll = 0.;

		Logger::log("Ctrl_Lat_Bump started:", name(), Logger::LOGLEVEL_INFO);

		// bump_roll->bump(0.0);

		while(true) {

			// run method exec timing stuff
			sleeptime = tmr->calcSleeptime();
			//Logger::log("sim_crrcsim.cpp: sleeptime: ", sleeptime, Logger::LOGLEVEL_INFO);
			usleep(sleeptime);
			dt = tmr->updateExecStats();

			// dt fractional in seconds
			dtf = (double)dt * 1e-6;

			// respond to parameter list request
			param_request_respond();

#ifdef MAVLINK_ENABLED_HUCH

			// use PID module to calculate new correcting variable
			// thrust = pid_roll->calc(dtf, z_hat);
			// Logger::log(name(), "thrust: ", thrust, params["roll_sp"], pid_roll->getSp(), Logger::LOGLEVEL_DEBUG);


			//ctl.thrust = pid_roll->calc((double)dt * 1e-6, z);

			// saturate

			// if(thrust > params["thr_max"]) {
			// 	thrust = params["thr_max"];
			// 	pid_roll->setIntegralM1();
			// }

			// if(thrust <= params["thr_min"]) {
			// 	thrust = params["thr_min"];
			// 	pid_roll->setIntegralM1();
			// }

			// if(thrust <= params[""]) {
			// 	thrust = params["thr_min"];
			// 	pid_roll->setIntegralM1();
			// }


			// // roll = pid_roll->calc(dtf, y * huch_visual_navigation.ego_speed);
			// roll = pid_roll->calc(dtf, 0);
			// // limit
			// if(roll > params["roll_limit"])
			// 	roll = params["roll_limit"];
			// if(roll < -params["roll_limit"])
			// 	roll = -params["roll_limit"];

			bumped_value = bump->calc(dtf);

			if(params["output_enable"] > 0) {
				if ((int)params["ctl_axis"] == 0)
					{
						chan.index = CHAN_PITCH;    
					}	
				else if ((int)params["ctl_axis"] == 1)
					{
						chan.index = CHAN_ROLL;
					}

				chan.usec = get_time_us();
				chan.value = bumped_value * params["ctl_direction"];
				
				// send setpoint
				send_debug(&msg, &dbg, 0+chan.index, chan.value);

				mavlink_msg_huch_generic_channel_encode(system_id(),
																								component_id,
																								&msg,
																								&chan);
				AppLayer<mavlink_message_t>::send(msg);
			}

#endif // MAVLINK_ENABLED_HUCH	

		}
	}

	void Ctrl_Lat_Bump::default_conf() {
		params["roll_Kc"] = 1.0;
		params["roll_Ti"] = 0.0;
		params["roll_Td"] = 0.0;
	}

	void Ctrl_Lat_Bump::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;

		// iter = args.find("component_id");
		// if( iter != args.end() ) {
		// 	istringstream s(iter->second);
		// 	s >> component_id;
		// }

		// update rate
		iter = args.find("ctl_update_rate");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_update_rate"];
		}
		else 
			params["ctl_update_rate"] = 10.0;

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
		iter = args.find("roll_ilim");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["roll_ilim"];
		}

		iter = args.find("thr_max");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["thr_max"];
		}
		iter = args.find("thr_min");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["thr_min"];
		}

		iter = args.find("output_enable");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["output_enable"];
		}
		else 
			params["output_enable"] = 0;

		iter = args.find("ctl_direction");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_direction"];
		}
		else 
			params["ctl_direction"] = 0;

		iter = args.find("ctl_axis");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_axis"];
		}
		else 
			params["ctl_axis"] = 0;

		Logger::log("ctrl_lat_bump::read_conf: component_id", component_id, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lat_bump::read_conf: roll_bias", params["roll_bias"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lat_bump::read_conf: roll_Kc", params["roll_Kc"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lat_bump::read_conf: roll_Ti", params["roll_Ti"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lat_bump::read_conf: roll_Td", params["roll_Td"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_lat_bump::read_conf: roll_limit", params["roll_limit"], Logger::LOGLEVEL_DEBUG);
	}
}

#endif // HAVE_MAVLINK_H
