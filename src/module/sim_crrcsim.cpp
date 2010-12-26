#include "sim_crrcsim.h"

#include "core/logger.h"
#include "utility.h"
#include "core/protocolstack.h"
#include <mavlink.h>

#include <iostream> //cout
using namespace std;

namespace mavhub {
Sim_Crrcsimule::Sim_Crrcsimule(const map<string, string> args) {
	// initialize module parameters from conf
	read_conf(args);
	//app_id = 4;
	//app_name = "crrcsim";
}

Sim_Crrcsimule::~Sim_Crrcsimule() {}

void Sim_Crrcsimule::handle_input(const mavlink_message_t &msg) {
	//Logger::log("Sim_Crrcsimule got mavlink_message, msgid:", (int)msg.msgid, (int)msg.len, Logger::LOGLEVEL_INFO);
}

void Sim_Crrcsimule::run() {
	if(!owner) {
		Logger::log("Owner of Sim_Crrcsimule not set", Logger::LOGLEVEL_WARN);
		return;
	}

	int system_type = MAV_QUADROTOR;
	mavlink_message_t msg_heartbeat;
	mavlink_msg_heartbeat_pack(owner->system_id(), component_id, &msg_heartbeat, system_type, MAV_AUTOPILOT_GENERIC);

	Logger::log("Sim_Crrcsimule started", Logger::LOGLEVEL_INFO);

	while(1) {
		//Logger::log("sim_crrcsim: system_id", static_cast<int>(owner->system_id()), Logger::LOGLEVEL_INFO);
		send(msg_heartbeat);
		sleep(1);
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
