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
	Logger::log("MKRCModule got mavlink_message", Logger::LOGLEVEL_INFO);
	
	if(msg.sysid == owner->system_id() && msg.msgid == 0) {//FIXME: set right msgid
		//TODO
		
	}
}

void MKRCModule::run() {
	//MKRC is passive, so nothing to do here
	return;
}

} // namespace mavhub
