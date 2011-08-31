#include "sim_crrcsim.h"

#include "core/logger.h"
#include "utility.h"
#include "protocol/protocolstack.h"
#include <mavlink.h>

#include <iostream> //cout
#include <stdlib.h>
using namespace std;

namespace mavhub {
	Sim_Crrcsimule::Sim_Crrcsimule(const map<string, string> args) : 
		AppInterface("crrcsim"), AppLayer("crrcsim"),
		ctl_mode(0) {
		// try and set reasonable defaults
		conf_defaults();
		// initialize module parameters from conf
		read_conf(args);
		/* initializations */
		// char* bus = 0;
		// bus = getenv ("IVYBUS");
		// std::cout << bus << std::endl;
		// IvyInit("MAVHUB", "READY", 0, 0, 0, 0);
		// IvyStart(bus);
		// /* binding of HelloCallback to messages starting with 'Hello' */
		// IvyBindMsg (HelloCallback, 0, "^Hello(.*)");
		// /* binding of ByeCallback to 'Bye' */
		// IvyBindMsg (ByeCallback, 0, "^Bye$");

		exec_tmr = new Exec_Timing((int)params["ctl_update_rate"]);

		// kp="0.066320932874519622"
		// ki="0.05339107055892655"
		// kd="0.018905589636341851"
		z = 0.0;
		// double ctl_Kc = 0.066320932874519622;
		// double ctl_Ti = 1.2421727487431193;
		// double ctl_Td = 0.28506217896710773;
		// double scalef = 0.0;
		// double ctl_Kc = 0.01;
		// double ctl_Ti = 0.0;
		// double ctl_Td = 100.0;
		// ctl_Kc = 0.05;
		// ctl_Ti = 4.3538;
		// ctl_Td = 0.28882;
		pid_alt = new PID(params["ac_pid_bias"],
											params["ac_pid_Kc"],
											params["ac_pid_Ki"],
											params["ac_pid_Kd"]);
		pid_alt->setSp(params["ac_sp"]);
	}

	Sim_Crrcsimule::~Sim_Crrcsimule() {}

	void Sim_Crrcsimule::handle_input(const mavlink_message_t &msg) {
		mavlink_huch_sensor_array_t sa;
		//int i;
		static int8_t param_id[15];

		switch(msg.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:
			//Logger::log("Sim_Crrcsimule got ml heartbeat, (msgid, sysid)", (int)msg.msgid, (int)msg.sysid, Logger::LOGLEVEL_INFO);
			break;

		case MAVLINK_MSG_ID_ATTITUDE:
			phi = mavlink_msg_attitude_get_roll(&msg);
			theta = mavlink_msg_attitude_get_pitch(&msg);
			psi = mavlink_msg_attitude_get_yaw(&msg);
			//Logger::log("Sim_Crrcsimule got ml attitude", phi, theta, Logger::LOGLEVEL_INFO);
			//IvySendMsg("%d ATTITUDE %f %f %f", 155, phi, theta, psi);
			break;

		case MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY:
			//Logger::log("Sim_Crrcsimule got ml huch sensor array, (msgid, sysid)", (int)msg.msgid, (int)msg.sysid, Logger::LOGLEVEL_INFO);
			mavlink_msg_huch_sensor_array_decode(&msg, &sa);
			z = (sa.data[0] + sa.data[1]) * 0.5;
			//Logger::log("Sim_Crrcsimule: z = ", z, Logger::LOGLEVEL_INFO);
			// for(i = 0; i < MAVLINK_MSG_HUCH_SENSOR_ARRAY_FIELD_DATA_LEN; i++) {
			// 	Logger::log("Sim_Crrcsimule:", i, sa.data[i], Logger::LOGLEVEL_INFO);
			// }
			break;

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Sim_Crrcsimule::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
				param_request_list = true;
			}
			break;

		case MAVLINK_MSG_ID_PARAM_SET:
			if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
				Logger::log("Sim_Crrcsimule::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Sim_Crrcsimule::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log("Sim_Crrcsimule::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);

					typedef map<string, double>::const_iterator ci;
					for(ci p = params.begin(); p!=params.end(); ++p) {
						// Logger::log("sim_crrcsimule param test", p->first, p->second, Logger::LOGLEVEL_INFO);
						if(!strcmp(p->first.data(), (const char *)param_id)) {
							params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
							Logger::log("x Sim_Crrcsimule::handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_INFO);
						}
					}
					// update variables
					pid_alt->setSp(params["ac_sp"]);
					pid_alt->setBias(params["ac_pid_bias"]);
					pid_alt->setKc(params["ac_pid_Kc"]);
					pid_alt->setTi(params["ac_pid_Ki"]);
					pid_alt->setTd(params["ac_pid_Kd"]);
					pid_alt->setIntegral(0.0);
					ctl_mode = static_cast<int>(params["ctl_mode"]);
				}
			}
			break;

		case MAVLINK_MSG_ID_DEBUG:
			Logger::log("Sim_Crrcsimule::handle_input: received debug from", (int)msg.sysid, (int)msg.compid, Logger::LOGLEVEL_INFO);
			break;

		case MAVLINK_MSG_ID_HUCH_SIM_CTRL:
			Logger::log("Sim_Crrcsimule::handle_input: received sim_ctrl from", (int)msg.sysid, (int)msg.compid, Logger::LOGLEVEL_INFO);
			break;
		default:
			break;

		}
		// AppLayer<mavlink_message_t>::send(msg);
	}

	void Sim_Crrcsimule::run() {
		if(!owner()) {
			Logger::log("Owner of Sim_Crrcsimule not set", Logger::LOGLEVEL_WARN);
			return;
		}

		mavlink_message_t msg;
		mavlink_manual_control_t ctl;
		int sleeptime;
		uint64_t dt;

		Logger::log("Sim_Crrcsimule started, sys_id", system_id(), Logger::LOGLEVEL_INFO);

		ctl.target = 0;
		ctl.roll = 0.;
		ctl.pitch = 0.;
		ctl.yaw = 0.;
		ctl.roll_manual = 0;
		ctl.pitch_manual = 0;
		ctl.yaw_manual = 0;
		ctl.thrust_manual = 0;

		while(1) {
			//Logger::log("sim_crrcsim: system_id", static_cast<int>(system_id()), Logger::LOGLEVEL_INFO);
			//AppLayer<mavlink_message_t>::send(msg_heartbeat);
			//send(msg_heartbeat);
			//IvySendMsg("%d MAVHUB_ALIVE %f", 155, 1.0);

			// run method exec timing stuff
			sleeptime = exec_tmr->calcSleeptime();
			//Logger::log("sim_crrcsim.cpp: sleeptime: ", sleeptime, Logger::LOGLEVEL_INFO);
			usleep(sleeptime);
			dt = exec_tmr->updateExecStats();

			// respond to parameter list request
			param_request_respond(param_request_list);

			switch(ctl_mode) {
			case CTL_MODE_BUMP:
				ctl.thrust = 0.41;
				break;
			case CTL_MODE_AC:
				//ctl.thrust = 0.5;
				//ctl.thrust = pid_alt->calc2(0.0025, z);
				ctl.thrust = pid_alt->calc((double)dt * 1e-6, z);
				break;
			case CTL_MODE_NULL:
			default:
				ctl.thrust = 0.0;
				break;
			}
			//Logger::log("sim_crrcsimule: ctl.thust = ", ctl.thrust, Logger::LOGLEVEL_INFO);
			mavlink_msg_manual_control_encode(42, 0, &msg, &ctl);
			//AppLayer<mavlink_message_t>::send(msg);
		}
	}

	// read config file
	void Sim_Crrcsimule::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;
		Logger::log("Sim_Crrcsimule::read_conf", Logger::LOGLEVEL_INFO);

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}
		Logger::log("Sim_Crrcsimule::read_conf: sysid, compid", system_id(), component_id, Logger::LOGLEVEL_INFO);
	}

	// set defaults
	void Sim_Crrcsimule::conf_defaults() {
		param_request_list = false;
		params["ac_pid_bias"] = 0.39;
		params["ac_pid_Kc"] = 0.01;
		params["ac_pid_Ki"] = 2.0;
		params["ac_pid_Kd"] = 0.4;
		params["ac_pid_scalef"] = 0.0;
		params["ac_sp"] = 2.23;
		params["ctl_mode"] = 0;
		params["ctl_update_rate"] = 100; // Hz
	}

	// handle parameter list request
	void Sim_Crrcsimule::param_request_respond(bool param_request) {
		if(param_request) {
			mavlink_message_t msg;
			Logger::log("Sim_Crrcsimule::param_request_respond", Logger::LOGLEVEL_INFO);
			param_request_list = false;
			typedef map<string, double>::const_iterator ci;
			for(ci p = params.begin(); p!=params.end(); ++p) {
				// Logger::log("sim_crrcsimule param test", p->first, p->second, Logger::LOGLEVEL_INFO);
				mavlink_msg_param_value_pack(system_id(), component_id, &msg, (const int8_t*) p->first.data(), p->second, 1, 0);
				AppLayer<mavlink_message_t>::send(msg);
			}
		}
	}

} // namespace mavhub
