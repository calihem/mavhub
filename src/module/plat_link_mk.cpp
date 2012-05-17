#include "plat_link_mk.h"

#ifdef HAVE_MAVLINK_H

#include "core/logger.h"
#include "utility.h"
#include "protocol/protocolstack.h"

//#include "opencv2/opencv.hpp"

#include <iostream> //cout
#include <stdlib.h>
using namespace std;
//using namespace cv;

namespace mavhub {
	Plat_Link_Mk::Plat_Link_Mk(const map<string, string> args) : 
		AppInterface("plat_link_mk_app"),
		ModuleBase(args, "plat_link_mk_app"),
		AppLayer<mkhuch_message_t>("plat_link_mk_app"),
		ctl_mode_alt(0),
		ctl_mode_lat(0),
		ctl_mode_yaw(0),
		thrust(0.0),
		roll(0.0),
		pitch(0.0),
		yaw(0.0),
		mask(0),
		ac_active(0),
		lc_active(0)
	{
		// try and set reasonable defaults
		conf_defaults();
		// initialize module parameters from conf
		read_conf(args);

		assign_variable_from_args(component_id);
		// assign_variable_from_args(output_enable);

		// init execution timer
		exec_tmr = new Exec_Timing((int)params["ctl_update_rate"]);

		// initialize communication strcutures
		pthread_mutex_init(&tx_mav_mutex, NULL);
		pthread_mutex_init(&tx_mkhuch_mutex, NULL);
		mkhuchlink_msg_init(&tx_mkhuch_msg);
	}

	Plat_Link_Mk::~Plat_Link_Mk() {}

	void Plat_Link_Mk::handle_input(const mavlink_message_t &msg) {
		mavlink_huch_sensor_array_t sa;
		//int i;
		static int8_t param_id[15];

		switch(msg.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:
			// Logger::log("Plat_Link_Mk got mavlink heartbeat, (msgid, sysid)", (int)msg.msgid, (int)msg.sysid, Logger::LOGLEVEL_INFO);
			break;

		case MAVLINK_MSG_ID_ATTITUDE:
			phi = mavlink_msg_attitude_get_roll(&msg);
			theta = mavlink_msg_attitude_get_pitch(&msg);
			psi = mavlink_msg_attitude_get_yaw(&msg);
			break;

		case MAVLINK_MSG_ID_LOCAL_POSITION:
			x = mavlink_msg_local_position_get_x(&msg);
			y = mavlink_msg_local_position_get_y(&msg);
			z = mavlink_msg_local_position_get_z(&msg);
			break;

		case MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY:
			mavlink_msg_huch_sensor_array_decode(&msg, &sa);
			m1 = sa.data[0];
			m2 = sa.data[1];
			z_hat = (sa.data[0] + sa.data[1]) * 0.5;
			break;

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Plat_Link_Mk::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
				param_request_list = true;
			}
			break;

		case MAVLINK_MSG_ID_PARAM_SET:
			if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
				Logger::log("Plat_Link_Mk::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Plat_Link_Mk::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log("Plat_Link_Mk::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);

					typedef map<string, double>::const_iterator ci;
					for(ci p = params.begin(); p!=params.end(); ++p) {
						// Logger::log("plat_link_mk param test", p->first, p->second, Logger::LOGLEVEL_INFO);
						if(!strcmp(p->first.data(), (const char *)param_id)) {
							params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
							Logger::log("x Plat_Link_Mk::handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_INFO);
						}
					}

					ctl_mode_alt = static_cast<int>(params["ctl_mode_alt"]);
					ctl_mode_lat = static_cast<int>(params["ctl_mode_lat"]);
					ctl_mode_yaw = static_cast<int>(params["ctl_mode_yaw"]);
					// output_enable = static_cast<int>(params["output_enable"]);
				}
			}
			break;

		case MAVLINK_MSG_ID_DEBUG:
			//Logger::log("Plat_Link_Mk::handle_input: received debug from", (int)msg.sysid, (int)msg.compid, Logger::LOGLEVEL_INFO);
			if(msg.sysid == system_id()) {
				if(msg.compid == component_id) {
					// Logger::log("Plat_Link_Mk::handle_input: received debug from", (int)msg.sysid, (int)msg.compid, Logger::LOGLEVEL_INFO);
					Logger::log("Plat_Link_Mk::handle_input: received debug from",
											(int)mavlink_msg_debug_get_ind(&msg),
											mavlink_msg_debug_get_value(&msg),
											Logger::LOGLEVEL_INFO);
					if((int)mavlink_msg_debug_get_ind(&msg) == 1) {
						thrust = mavlink_msg_debug_get_value(&msg);
					}
				}
			}
			break;

		case MAVLINK_MSG_ID_HUCH_GENERIC_CHANNEL:
			switch(mavlink_msg_huch_generic_channel_get_index(&msg)) {
			case CHAN_THRUST:
				thrust = mavlink_msg_huch_generic_channel_get_value(&msg);
				// Logger::log("Plat_Link_Mk::handle_input: received gen_chan thrust from", (int)msg.sysid, (int)msg.compid, thrust, Logger::LOGLEVEL_INFO);
				break;
			case CHAN_ROLL:
				roll = mavlink_msg_huch_generic_channel_get_value(&msg);
				// Logger::log("Plat_Link_Mk::handle_input: received gen_chan roll from", (int)msg.sysid, (int)msg.compid, roll, Logger::LOGLEVEL_INFO);
				break;
			case CHAN_PITCH:
				pitch = mavlink_msg_huch_generic_channel_get_value(&msg);
				// Logger::log("Plat_Link_Mk::handle_input: received gen_chan pitch from", (int)msg.sysid, (int)msg.compid, pitch, Logger::LOGLEVEL_INFO);
				break;
			case CHAN_YAW:
				yaw = mavlink_msg_huch_generic_channel_get_value(&msg);
				// Logger::log("Plat_Link_Mk::handle_input: received gen_chan yaw from", (int)msg.sysid, (int)msg.compid, yaw, Logger::LOGLEVEL_INFO);
				break;
			default:
				break;
			}
			break;

		case MAVLINK_MSG_ID_HUCH_SIM_CTRL:
			Logger::log("Plat_Link_Mk::handle_input: received sim_ctrl from", (int)msg.sysid, (int)msg.compid, Logger::LOGLEVEL_INFO);
			break;

		case MAVLINK_MSG_ID_ACTION:
			Logger::log("Plat_Link_Mk::handle_input: received action request", (int)msg.sysid, (int)msg.compid, Logger::LOGLEVEL_INFO);
			if(mavlink_msg_action_get_target(&msg) == system_id()){
				if(mavlink_msg_action_get_target_component(&msg) == component_id) {
					switch(mavlink_msg_action_get_action(&msg)) {
					case ACTION_TOGGLE_AC:
						ac_active = !ac_active; // Altitude control
						mask = THRUST_MANUAL_MASK * ac_active;
						thrust = thrust * ac_active;
						params["ctl_mode_alt"] = CTL_MODE_DIRECT * ac_active;
						ctl_mode_alt = static_cast<int>(params["ctl_mode_alt"]);
						Logger::log("Plat_Link_Mk::handle_input: received action TOGGLE_AC",
												(int)ac_active, (int)mask, params["ctl_mode_alt"],
												Logger::LOGLEVEL_INFO);
						break;
					case ACTION_TOGGLE_LC: // Lateral control
						lc_active = !lc_active;
						roll = 0.;
						pitch = 0.;
						params["ctl_mode_lat"] = CTL_MODE_DIRECT * lc_active;
						ctl_mode_lat = static_cast<int>(params["ctl_mode_lat"]);
						Logger::log("Plat_Link_Mk::handle_input: received action TOGGLE_LC",
												(int)lc_active, (int)mask, params["ctl_mode_lat"],
												Logger::LOGLEVEL_INFO);
						break;
					default:
						break;
					}
				}
			}
			break;
		default:
			break;

		}
		// AppLayer<mavlink_message_t>::send(msg);
	}

	void Plat_Link_Mk::handle_input(const mkhuch_message_t& msg) {
	}

	void Plat_Link_Mk::run() {
		if(!owner()) {
			Logger::log("Owner of Plat_Link_Mk not set", Logger::LOGLEVEL_WARN);
			return;
		}

		int sleeptime;
		uint64_t dt;
		int ctl_update_rate = 60; // pseye fps

		// subscribe to data streams
		Logger::log("Plat_Link_Mk sending stream requests", Logger::LOGLEVEL_INFO);
		send_stream_request(&msg, MAV_DATA_STREAM_POSITION, ctl_update_rate);
		send_stream_request(&msg, MAV_DATA_STREAM_RAW_SENSORS, ctl_update_rate);
		send_stream_request(&msg, MAV_DATA_STREAM_RC_CHANNELS, ctl_update_rate);
		send_stream_request(&msg, MAV_DATA_STREAM_EXTENDED_STATUS, ctl_update_rate);
		send_stream_request(&msg, MAV_DATA_STREAM_EXTRA1, ctl_update_rate);

		Logger::log("Plat_Link_Mk started, sys_id, comp_id", system_id(), component_id, Logger::LOGLEVEL_INFO);

		// extern_control.target = 43; // paramize
		extern_control.roll = 0;
		extern_control.pitch = 0;
		extern_control.yaw = 0;
		extern_control.thrust = 0;
		extern_control.mask = 0; // all inputs are additive to manual control

		while(1) {
			//Logger::log("plat_link_mk: system_id", static_cast<int>(system_id()), Logger::LOGLEVEL_INFO);

			// run method exec timing stuff
			sleeptime = exec_tmr->calcSleeptime();
			//Logger::log("plat_link_mk.cpp: sleeptime: ", sleeptime, Logger::LOGLEVEL_INFO);
			usleep(sleeptime);
			dt = exec_tmr->updateExecStats();

			// respond to parameter list request
			param_request_respond();

			switch(ctl_mode_alt) {
			case CTL_MODE_BUMP:
				extern_control.thrust = 0; // bump->calc((double)dt * 1e-6);
				break;
			case CTL_MODE_AC:
				// FIXME: built in controller is obsolete. Rather, use
				//        external controller module so identical output can
				//        be routed to any copter hardware
				extern_control.thrust = 0;

				break;
			case CTL_MODE_DIRECT:
				// pipe through
				extern_control.thrust = thrust;
				extern_control.mask = mask; // THRUST_MANUAL_MASK;
				break;
			case CTL_MODE_NULL:
			default:
				extern_control.thrust = 0.0;
				extern_control.mask = mask; // THRUST_MANUAL_MASK;
				break;
			}

			switch(ctl_mode_lat) {
			case CTL_MODE_BUMP:
				extern_control.roll = 0; // bump_lat->calc((double)dt * 1e-6);
				break;
			case CTL_MODE_DIRECT:
				extern_control.roll = roll;
				extern_control.pitch = pitch;
				break;
			case CTL_MODE_NULL:
			default:
				extern_control.pitch = 0.0;
				extern_control.roll = 0.0;
				break;
			}

			switch(ctl_mode_yaw) {
			case CTL_MODE_BUMP:
				extern_control.yaw = 0; // bump_lat->calc((double)dt * 1e-6);
				break;
			case CTL_MODE_DIRECT:
				extern_control.yaw = yaw;
				break;
			case CTL_MODE_NULL:
			default:
				extern_control.yaw = 0.0;
				break;
			}

			// send control output to (onboard) controller
			if(params["output_enable"] > 0) {

				// Logger::log("plat_link_mk: extern_control.roll = ", extern_control.roll, Logger::LOGLEVEL_INFO);
				// Logger::log("plat_link_mk: extern_control.pitch = ", extern_control.pitch, Logger::LOGLEVEL_INFO);
				// Logger::log("plat_link_mk: extern_control.yaw = ", extern_control.yaw, Logger::LOGLEVEL_INFO);
				// Logger::log("plat_link_mk: extern_control.thrust = ", extern_control.thrust, Logger::LOGLEVEL_INFO);
				// Logger::log("plat_link_mk: extern_control.mask = ", (int)extern_control.mask, Logger::LOGLEVEL_INFO);

				mkhuchlink_msg_encode(&tx_mkhuch_msg,
															MKHUCH_MSG_TYPE_EXT_CTRL,
															&extern_control,
															sizeof(mkhuch_extern_control_t));
				AppLayer<mkhuch_message_t>::send(tx_mkhuch_msg);
			}
		}
	}

	// read config file
	void Plat_Link_Mk::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;
		Logger::log("Plat_Link_Mk::read_conf", Logger::LOGLEVEL_INFO);

		// iter = args.find("component_id");
		// if( iter != args.end() ) {
		// 	istringstream s(iter->second);
		// 	s >> component_id;
		// }

		iter = args.find("bump_thr_low");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["bump_thr_low"];
		}
		iter = args.find("bump_thr_high");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["bump_thr_high"];
		}

		iter = args.find("ctl_update_rate");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_update_rate"];
		}

		iter = args.find("output_enable");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["output_enable"];
		}

		iter = args.find("ctl_mode_alt");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_mode_alt"];
			ctl_mode_alt = static_cast<int>(params["ctl_mode_alt"]);
		}

		iter = args.find("ctl_mode_lat");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_mode_lat"];
			ctl_mode_lat = static_cast<int>(params["ctl_mode_lat"]);
		}

		iter = args.find("ctl_mode_yaw");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["ctl_mode_yaw"];
			ctl_mode_yaw = static_cast<int>(params["ctl_mode_yaw"]);
		}

		Logger::log("Plat_Link_Mk::read_conf: sysid, compid", system_id(), component_id, Logger::LOGLEVEL_INFO);
	}

	// set defaults
	void Plat_Link_Mk::conf_defaults() {
		param_request_list = false;
		// 0.147893587316 0.14 0.00928571428571
		params["ac_pid_bias"] = 0.0;
		// manually tuned
		// params["ac_pid_Kc"] = 0.01;
		// params["ac_pid_Ki"] = 2.0;
		// params["ac_pid_Kd"] = 0.4;
		// from tuning recipe, doesn't work
		// params["ac_pid_Kc"] = 0.147893587316;
		// params["ac_pid_Ki"] = 0.14;
		// params["ac_pid_Kd"] = 0.00928571428571;
		// from evolution, old controller
		params["ac_pid_Kc"] = 0.066320932874519622;
		params["ac_pid_Ki"] = 1.2421727487431193;
		params["ac_pid_Kd"] = 0.28506217896710773;
		// from ol: 0.10861511  2.93254908  0.02315
		// from ol: 0.1, 2, 0.5
		params["ac_pid_int_lim"] = 10.0;
		params["ac_pid_scalef"] = 0.0;
		params["ac_sp"] = 2.23;
		params["ctl_mode_alt"] = 0;
		params["ctl_mode_lat"] = 0;
		params["ctl_mode_yaw"] = 0;
		params["ctl_update_rate"] = 100; // Hz
		params["bump_thr_low"] = 0.38;
		params["bump_thr_high"] = 0.39;
		params["thr_max"] = 0.6;
		params["thr_min"] = 0.1;
		params["output_enable"] = 0;
	}

	// // handle parameter list request
	// void Plat_Link_Mk::param_request_respond() {
	// 	Logger::log("Plat_Link_Mk::param_request_respond", Logger::LOGLEVEL_INFO);
	// 	param_request_list = false;
	// 	typedef map<string, double>::const_iterator ci;
	// 	for(ci p = params.begin(); p!=params.end(); ++p) {
	// 		// Logger::log("plat_link_mk param test", p->first, p->second, Logger::LOGLEVEL_INFO);
	// 		mavlink_msg_param_value_pack(system_id(), component_id, &msg, (const int8_t*) p->first.data(), p->second, 1, 0);
	// 		AppLayer<mavlink_message_t>::send(msg);
	// 	}
	// }

	// // send debug
	// void Plat_Link_Mk::send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value) {
	// 	dbg->ind = index;
	// 	dbg->value = value;
	// 	mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), msg, dbg);
	// 	//Logger::log("mk: sending debug", Logger::LOGLEVEL_INFO);
	// 	AppLayer<mavlink_message_t>::send(*msg);
	// }



} // namespace mavhub

#endif // HAVE_MAVLINK_H

