#include "mkrcmod.h"

#include "logger.h"
#include "utility.h"
#include "protocolstack.h"
#include <mavlink.h>

#include <iostream> //cout
using namespace std;

namespace mavhub {
MKRCModule::MKRCModule() {
}

MKRCModule::~MKRCModule() {}

void MKRCModule::handle_input(const mavlink_message_t &msg) {
	Logger::log("MKRCModule got mavlink_message", Logger::LOGLEVEL_DEBUG);

	switch(msg.msgid) {
		case MAVLINK_MSG_ID_PING:
			if(mavlink_msg_ping_get_target_system(&msg) == 0) { //ping request
				mavlink_msg_ping_pack(owner->system_id(),
					component_id,
					&tx_msg,
					mavlink_msg_ping_get_seq(&msg),
					msg.sysid,
					msg.compid,
					get_time_us());
				send(tx_msg);
			} else { //ping answer
				//TODO
			}
			break;
		default:
			break;
	}
}

void MKRCModule::run() {
	//MKRC is passive, so nothing to do here
	return;
}

} // namespace mavhub
