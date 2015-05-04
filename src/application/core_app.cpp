#include "core_app.h"

#ifdef HAVE_MAVLINK_H

#include "core/logger.h"

#include <sstream> //istringstream

namespace mavhub {

CoreApp::CoreApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) : 
	AppInterface("coreapp", loglevel),
	AppLayer<mavlink_message_t>("coreapp"),
	mav_type(MAV_TYPE_GENERIC),
	component_id(0),
	autopilot(MAV_AUTOPILOT_GENERIC) {

  Logger::log(name(), "CoreApp init", Logger::LOGLEVEL_DEBUG);

	std::map<std::string,std::string>::const_iterator iter = args.find("mav_type");
	if( iter != args.end() ) {
		std::istringstream istream(iter->second);
		istream >> mav_type;
	} else {
		log("CoreApp: mav_type argument missing", Logger::LOGLEVEL_DEBUG);
	}

	iter = args.find("component_id");
	if( iter != args.end() ) {
		std::istringstream istream(iter->second);
		istream >> component_id;
	} else {
		log("CoreApp: component_id argument missing", Logger::LOGLEVEL_DEBUG);
	}

	iter = args.find("autopilot");
	if( iter != args.end() ) {
		std::istringstream istream(iter->second);
		istream >> autopilot;
	} else {
		log("CoreApp: autopilot argument missing", Logger::LOGLEVEL_DEBUG);
	}
}

CoreApp::~CoreApp() {}

void CoreApp::handle_input(const mavlink_message_t &msg) {
  // Logger::log("CoreApp got mavlink_message, msgid:", static_cast<int>(msg.msgid), static_cast<int>(msg.len), Logger::LOGLEVEL_DEBUG);
}

void CoreApp::run() {
	if(!owner()) {
		log("Owner of CoreApp not set", Logger::LOGLEVEL_WARN);
		return;
	}

	mavlink_message_t heartbeat_msg;
	mavlink_msg_heartbeat_pack(system_id(),
		component_id,
		&heartbeat_msg,
		mav_type,
		autopilot,
		MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,	//base mode
		0,	//custom mode
		MAV_STATE_ACTIVE);	//system status

	union
	{
          uint8_t bytes[16];
          float val[4];
	} bfconvert;

	// float preval[4] = {0.1, 0.2, 0.3, 0.5};
	bfconvert.val[0] = 0.1;
	bfconvert.val[1] = 0.2;
	bfconvert.val[2] = 0.4;
	bfconvert.val[3] = 0.8;
	// uint8_t data_i[16];
	mavlink_message_t msg;
	mavlink_data16_t d16;
	// data_i = (uint8_t)&data;
	d16.type = 0;
	d16.len = 16;
	for (int i = 0; i < 16; i++) {
          d16.data[i] = bfconvert.bytes[i];
	}

	mavlink_msg_data16_encode(
                                  system_id(),
                                  component_id,
                                  &msg,
                                  &d16);

	log("CoreApp started", Logger::LOGLEVEL_DEBUG);

	while( !interrupted() ) {
          log("CoreApp entered loop", Logger::LOGLEVEL_DEBUG);
          send(heartbeat_msg);
          // test data16
          // send(msg);
          usleep(900000);
	}
}

} // namespace mavhub

#endif // HAVE_MAVLINK_H


