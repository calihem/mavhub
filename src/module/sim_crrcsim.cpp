#include "sim_crrcsim.h"

#include "core/logger.h"
#include "utility.h"
#include "protocol/protocolstack.h"
#include <mavlink.h>

#include <iostream> //cout
#include <stdlib.h>
using namespace std;

namespace mavhub {
	Sim_Crrcsimule::Sim_Crrcsimule(const map<string, string> args) : AppInterface("crrcsim"), AppLayer("crrcsim") {
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

		// kp="0.066320932874519622"
		// ki="0.05339107055892655"
		// kd="0.018905589636341851"
		z = 0.0;
		// double ctl_Kc = 0.066320932874519622;
		// double ctl_Ti = 1.2421727487431193;
		// double ctl_Td = 0.28506217896710773;
		double scalef = 0.0;
		double ctl_Kc = 0.01;
		double ctl_Ti = 0.0;
		double ctl_Td = 100.0;
		// ctl_Kc = 0.05;
		// ctl_Ti = 4.3538;
		// ctl_Td = 0.28882;
		pid_alt = new PID(0.41, ctl_Kc, ctl_Ti, ctl_Td);
		pid_alt->setSp(2.0);
	}

	Sim_Crrcsimule::~Sim_Crrcsimule() {}

	void Sim_Crrcsimule::handle_input(const mavlink_message_t &msg) {
		mavlink_huch_sensor_array_t sa;
		int i;
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
			// for(i = 0; i < MAVLINK_MSG_HUCH_SENSOR_ARRAY_FIELD_DATA_LEN; i++) {
			// 	Logger::log("Sim_Crrcsimule:", i, sa.data[i], Logger::LOGLEVEL_INFO);
			// }
			
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

		// int system_type = MAV_QUADROTOR;
		// mavlink_message_t msg_heartbeat;
		// mavlink_msg_heartbeat_pack(system_id(), component_id, &msg_heartbeat, system_type, MAV_AUTOPILOT_HUCH);
		mavlink_message_t msg;
		mavlink_manual_control_t ctl;

		Logger::log("Sim_Crrcsimule started", Logger::LOGLEVEL_INFO);

		// ivy_cb_t blub;
		// blub.a = 1;
		// blub.b = -3.14;
		
		//IvyAfterSelect
		//IvySetAfterSelectHook((void *)afterSelect_cb, &blub);
		//pt2Function = &afterSelect_cb;
		//IvySetBeforeSelectHook(pt2Function, (void*)0);


		//int delay = 1000;
		//tid = TimerRepeatAfter (0, delay, handle_timer, 0);

		//IvyMainLoop();

		while(1) {
			//Logger::log("sim_crrcsim: system_id", static_cast<int>(system_id()), Logger::LOGLEVEL_INFO);
			//AppLayer<mavlink_message_t>::send(msg_heartbeat);
			//send(msg_heartbeat);
			//IvySendMsg("%d MAVHUB_ALIVE %f", 155, 1.0);
			ctl.target = 0;
			ctl.roll = 0.;
			ctl.pitch = 0.;
			ctl.yaw = 0.;
			//ctl.thrust = 0.5;
			ctl.thrust = pid_alt->calc2(0.0025, z);
			mavlink_msg_manual_control_encode(42, 0, &msg, &ctl);
			AppLayer<mavlink_message_t>::send(msg);
			usleep(2500);
		}
	}

	void Sim_Crrcsimule::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;
		Logger::log("Sim_Crrcsimule::read_conf", Logger::LOGLEVEL_INFO);

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}
		Logger::log("Sim_Crrcsimule::read_conf: component_id", component_id, Logger::LOGLEVEL_INFO);
	}

} // namespace mavhub
