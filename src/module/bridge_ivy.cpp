#include "bridge_ivy.h"

#ifdef HAVE_IVY_IVY_H

#include "core/logger.h"
#include "utility.h"
#include "protocol/protocolstack.h"
#include <mavlink.h>

#include <iostream> //cout
#include <stdlib.h>
using namespace std;

TimerId tid;

/* callback associated to "Hello" messages */
void HelloCallback (IvyClientPtr app, void *data, int argc, char **argv)
{                                  
	const char* arg = (argc < 1) ? "" : argv[0];
	IvySendMsg ("Bonjour%s", arg);
}
/* callback associated to "Bye" messages */
void ByeCallback (IvyClientPtr app, void *data, int argc, char **argv)
{
	IvyStop ();
}

/* callback associated to "Hello" messages */
void DlSettingCallback (IvyClientPtr app, void *data, int argc, char **argv)
{                                  
	//mavlink_message_t msg;
	//const char* arg = (argc < 1) ? "" : argv[0];
	//IvySendMsg ("Bonjour%s", arg);
	if(argc == 4) {
		printf("bridge_ivy/DlSettingCallback: %s, %s, %s, %s\n",
					 argv[0], argv[1], argv[2], argv[3]);
	}
	// mavlink_msg_param_value_pack(atoi(argv[1]), 27, &msg, (const int8_t*) "ctl_mode", atoi(argv[3]), 1, 0);
	// AppLayer<mavlink_message_t>::send(msg);
}

// typedef struct {
// 	int a;
// 	double b;
// } ivy_cb_t;

void (*pt2Function)(void*) = NULL;                        // C
//void (mavhub::Bridge_Ivy::*pt2Member)(void*) = NULL;                // C++

//void afterSelect_cb(ivy_cb_t* arg) {
void afterSelect_cb(void* arg) {
	//IvyHookPtr* a;
	//std::cout << arg->a ", " << arg->b << std::endl;
	//printf("%d, %f\n", arg->a, arg->b);
	printf("afterSelect_cb: 1,2,3\n");
	//return a;
	//return 0;
}

void handle_timer (TimerId id, void *data, unsigned long delta) {
	//if (wakeup(3) && send_loop()) decode_and_send_to_ivy();
	//if (wakeup(3))
	//do_something();
	//printf("handle_timer\n");
	//IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", 155);
}

namespace mavhub {
	Bridge_Ivy::Bridge_Ivy(const map<string, string> args) : AppInterface("crrcsim"), AppLayer("crrcsim") {
		//, phi(0.0), theta(0.0), psi(0.0) 
		// initialize mod parameters from conf
		read_conf(args);
		/* initializations */
		char* bus = 0;
		bus = getenv ("IVYBUS");
		std::cout << bus << std::endl;
		IvyInit("MAVHUB", "READY", 0, 0, 0, 0);
		IvyStart(bus);
		/* binding of HelloCallback to messages starting with 'Hello' */
		IvyBindMsg (HelloCallback, 0, "^Hello(.*)");
		/* binding of ByeCallback to 'Bye' */
		IvyBindMsg (ByeCallback, 0, "^Bye$");
		IvyBindMsg (DlSettingCallback, 0, "^(dl DL_SETTING) ([0-9]*) ([0-9]*) ([0-9]*.[0-9]*)");
	}

	Bridge_Ivy::~Bridge_Ivy() {}

	void Bridge_Ivy::handle_input(const mavlink_message_t &msg) {
		mavlink_huch_sensor_array_t sa;
		switch(msg.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:
			Logger::log("Bridge_Ivy got ml heartbeat: (msgid, sysid)", (int)msg.msgid, (int)msg.sysid, Logger::LOGLEVEL_INFO);
			//IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", msg.sysid);
			IvySendMsg("%d ALIVE 50,162,250,192,221,27,111,57,63,249,122,206,139,11,197,244", msg.sysid);
			break;
		case MAVLINK_MSG_ID_ATTITUDE:
			// phi = mavlink_msg_attitude_get_roll(&msg);
			// theta = mavlink_msg_attitude_get_pitch(&msg);
			// psi = mavlink_msg_attitude_get_yaw(&msg);
			// //Logger::log("Bridge_Ivy got ml attitude", phi, theta, Logger::LOGLEVEL_INFO);
			//IvySendMsg("%d ATTITUDE %f %f %f", 155, phi, theta, psi);
			//IvySendMsg("%d ATTITUDE %f %f %f", msg.sysid, 0., 0., 0.);
			break;
		case MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY:
			mavlink_msg_huch_sensor_array_decode(&msg, &sa);
			//IvySendMsg("%d PRESSURE 0.0 0.0 %f %f", msg.sysid, sa.data[0], sa.data[1]);
			IvySendMsg("%d HUCH_SENSOR_RAW %f %f", msg.sysid, sa.data[0], sa.data[1]);
			break;
		default:
			break;
		}
		// AppLayer<mavlink_message_t>::send(msg);
	}

	void Bridge_Ivy::run() {
		if(!owner()) {
			Logger::log("Owner of Bridge_Ivy not set", Logger::LOGLEVEL_WARN);
			return;
		}

		//int system_type = MAV_QUADROTOR;
		// mavlink_message_t msg_heartbeat;
		// mavlink_msg_heartbeat_pack(system_id(), component_id, &msg_heartbeat, system_type, MAV_AUTOPILOT_HUCH);

		Logger::log("Bridge_Ivy started", Logger::LOGLEVEL_INFO);

		// ivy_cb_t blub;
		// blub.a = 1;
		// blub.b = -3.14;

		// how to add before/after select hook
		// pt2Function = &afterSelect_cb;
		// IvySetBeforeSelectHook(pt2Function, (void*)0);
		// IvySetAfterSelectHook(pt2Function, (void*)0);

		// with local member function, defunct
		// pt2Member = &mavhub::Bridge_Ivy::afterSelect_cb;
		// IvySetBeforeSelectHook(pt2Member, (void*)0);


		int delay = 1000;
		tid = TimerRepeatAfter (0, delay, handle_timer, 0);

		IvyMainLoop();

		// while(1) {
		// 	Logger::log("bridge_ivy: system_id", static_cast<int>(system_id()), Logger::LOGLEVEL_INFO);
		// 	//AppLayer<mavlink_message_t>::send(msg_heartbeat);
		// 	//send(msg_heartbeat);
		// 	//IvySendMsg("%d MAVHUB_ALIVE %f", 155, 1.0);
		// 	sleep(1);
		// }
	}

	void Bridge_Ivy::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;
		Logger::log("Bridge_Ivy::read_conf", Logger::LOGLEVEL_INFO);

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}
		Logger::log("Bridge_Ivy::read_conf: component_id", component_id, Logger::LOGLEVEL_INFO);
	}

	// void Bridge_Ivy::handle_timer (TimerId id, void *data, unsigned long delta) {
	// 	//if (wakeup(3) && send_loop()) decode_and_send_to_ivy();
	// 	//if (wakeup(3))
	// 	//do_something();
	// 	printf("handle_timer\n");
	// 	IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", 155);
	// }

	// void Bridge_Ivy::afterSelect_cb(void* arg) {
	// 	//IvyHookPtr* a;
	// 	//std::cout << arg->a ", " << arg->b << std::endl;
	// 	//printf("%d, %f\n", arg->a, arg->b);
	// 	printf("Bridge_Ivy::afterSelect_cb: 1,2,3\n");
	// 	//return a;
	// 	//return 0;
	// }

} // namespace mavhub

#endif // HAVE_IVY_IVY_H
