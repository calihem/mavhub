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

typedef struct {
	int a;
	double b;
} ivy_cb_t;

void (*pt2Function)(void*) = NULL;                        // C

//void afterSelect_cb(ivy_cb_t* arg) {
void afterSelect_cb(void* arg) {
	//IvyHookPtr* a;
	//std::cout << arg->a ", " << arg->b << std::endl;
	//printf("%d, %f\n", arg->a, arg->b);
	printf("1,2,3\n");
	//return a;
	//return 0;
}

void handle_timer (TimerId id, void *data, unsigned long delta) {
	//if (wakeup(3) && send_loop()) decode_and_send_to_ivy();
	//if (wakeup(3))
	//do_something();
	printf("handle_timer\n");
	IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", 155);
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

	}

	Bridge_Ivy::~Bridge_Ivy() {}

	void Bridge_Ivy::handle_input(const mavlink_message_t &msg) {
		switch(msg.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:
			Logger::log("Bridge_Ivy got ml heartbear, msgid:", (int)msg.msgid, (int)msg.len, Logger::LOGLEVEL_INFO);
			break;
		case MAVLINK_MSG_ID_ATTITUDE:
			// phi = mavlink_msg_attitude_get_roll(&msg);
			// theta = mavlink_msg_attitude_get_pitch(&msg);
			// psi = mavlink_msg_attitude_get_yaw(&msg);
			// //Logger::log("Bridge_Ivy got ml attitude", phi, theta, Logger::LOGLEVEL_INFO);
			//IvySendMsg("%d ATTITUDE %f %f %f", 155, phi, theta, psi);
			IvySendMsg("%d ATTITUDE %f %f %f", 155, 0., 0., 0.);
		default:
			break;
		}
		AppLayer<mavlink_message_t>::send(msg);
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
		
		//IvyAfterSelect
		//IvySetAfterSelectHook((void *)afterSelect_cb, &blub);
		pt2Function = &afterSelect_cb;
		IvySetBeforeSelectHook(pt2Function, (void*)0);


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

} // namespace mavhub

#endif // HAVE_IVY_IVY_H
