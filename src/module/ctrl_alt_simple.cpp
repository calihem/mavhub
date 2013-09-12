// control altitude, simple version

#include "ctrl_alt_simple.h"

#ifdef HAVE_MAVLINK_H

// #include "core/datacenter.h"
#include "protocol/protocolstack.h"

#include <cstdlib>
#include <sstream>

#include <math.h>

#define RAND_MAX_TO_M1 1/(double)RAND_MAX

using namespace std;

namespace mavhub {
	Ctrl_Alt_Simple::Ctrl_Alt_Simple(const map<string, string> args) :
		AppInterface("ctrl_alt_simple"),
		ModuleBase(args, "ctrl_alt_simple"),
		z_hat(0.0)
	{
		read_conf(args);
		assign_variable_from_args(component_id);
		// assign_variable_from_args(output_enable);

		param_request_list = 0;
		pid_alt = new PID(params["alt_bias"], params["alt_Kc"],
											params["alt_Ti"], params["alt_Td"]);
		pid_alt->setSp(params["alt_sp"]);
		pid_alt->setPv_int_lim(params["alt_ilim"]);
		tmr = new Exec_Timing((int)params["ctl_update_rate"]);
	}

	Ctrl_Alt_Simple::~Ctrl_Alt_Simple() {
	}

  void Ctrl_Alt_Simple::handle_input(const mavlink_message_t &msg) {
		static char param_id[16];
		static mavlink_huch_sensor_array_t sa;
		//Logger::log("Ctrl_Alt_Simple got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_DEBUG);
		switch(msg.msgid) {

#ifdef MAVLINK_ENABLED_HUCH
		case MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY:
			mavlink_msg_huch_sensor_array_decode(&msg, &sa);
			// m1 = sa.data[0];
			// m2 = sa.data[1];
			z_hat = (sa.data[0] + sa.data[1]) * 0.5;
			// Logger::log(name(), "z_hat: ", z_hat, Logger::LOGLEVEL_DEBUG);
			break;
#endif // MAVLINK_ENABLED_HUCH	

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Ctrl_Alt_Simple::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
				param_request_list = 1;
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			Logger::log("Ctrl_Alt_Simple::handle_input: PARAM_SET", (int)system_id(), Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
				Logger::log("Ctrl_Alt_Simple::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Ctrl_Alt_Simple::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log("Ctrl_Alt_Simple::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);

					typedef map<string, double>::const_iterator ci;
					for(ci p = params.begin(); p!=params.end(); ++p) {
						// Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
						if(!strcmp(p->first.data(), (const char *)param_id)) {
							params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
							Logger::log("x Ctrl_Alt_Simple::handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_INFO);
						}
					}

					// update PID controllers
					pid_alt->setKc(params["alt_Kc"]);
					pid_alt->setTi(params["alt_Ti"]);
					pid_alt->setTd(params["alt_Td"]);
					pid_alt->setBias(params["alt_bias"]);
					pid_alt->setSp(params["alt_sp"]);

					// output_enable = static_cast<int>(params["output_enable"]);
				}
			}
			break;
		default:
			break;
		}		
	}

  void Ctrl_Alt_Simple::run() {
		// generic
		static mavlink_message_t msg;

		// timing
		uint64_t dt = 0;
		double dtf = 0.0;
		int sleeptime;
		// double alt = 0.;

		Logger::log("Ctrl_Alt_Simple started:", name(), Logger::LOGLEVEL_INFO);

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
			thrust = pid_alt->calc(dtf, z_hat);
			// Logger::log(name(), "thrust: ", thrust, params["alt_sp"], pid_alt->getSp(), Logger::LOGLEVEL_DEBUG);


			//ctl.thrust = pid_alt->calc((double)dt * 1e-6, z);

			// saturate

			// if(thrust > params["thr_max"]) {
			// 	thrust = params["thr_max"];
			// 	pid_alt->setIntegralM1();
			// }

			// if(thrust <= params["thr_min"]) {
			// 	thrust = params["thr_min"];
			// 	pid_alt->setIntegralM1();
			// }

			// if(thrust <= params[""]) {
			// 	thrust = params["thr_min"];
			// 	pid_alt->setIntegralM1();
			// }


			// // alt = pid_alt->calc(dtf, y * huch_visual_navigation.ego_speed);
			// alt = pid_alt->calc(dtf, 0);
			// // limit
			// if(alt > params["alt_limit"])
			// 	alt = params["alt_limit"];
			// if(alt < -params["alt_limit"])
			// 	alt = -params["alt_limit"];

			if(params["output_enable"] > 0) {
                          chan.usec = get_time_us();
                          chan.index = CHAN_THRUST;
                          chan.value = thrust;
                          mavlink_msg_huch_generic_channel_encode(system_id(),
                                                                  component_id,
                                                                  &msg,
                                                                  &chan);
                          AppLayer<mavlink_message_t>::send(msg);
			}

#endif // MAVLINK_ENABLED_HUCH	

		}
	}

	void Ctrl_Alt_Simple::default_conf() {
		params["alt_Kc"] = 1.0;
		params["alt_Ti"] = 0.0;
		params["alt_Td"] = 0.0;
	}

	void Ctrl_Alt_Simple::read_conf(const map<string, string> args) {
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

		// controller params alt
		iter = args.find("alt_bias");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["alt_bias"];
		}
		iter = args.find("alt_Kc");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["alt_Kc"];
		}
		iter = args.find("alt_Ti");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["alt_Ti"];
		}
		iter = args.find("alt_Td");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["alt_Td"];
		}
		iter = args.find("alt_limit");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["alt_limit"];
		}
		iter = args.find("alt_sp");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["alt_sp"];
		}
		iter = args.find("alt_ilim");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["alt_ilim"];
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

		Logger::log("ctrl_alt_simple::read_conf: component_id", component_id, Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_alt_simple::read_conf: alt_bias", params["alt_bias"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_alt_simple::read_conf: alt_Kc", params["alt_Kc"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_alt_simple::read_conf: alt_Ti", params["alt_Ti"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_alt_simple::read_conf: alt_Td", params["alt_Td"], Logger::LOGLEVEL_DEBUG);
		Logger::log("ctrl_alt_simple::read_conf: alt_limit", params["alt_limit"], Logger::LOGLEVEL_DEBUG);
	}
}

#endif // HAVE_MAVLINK_H
