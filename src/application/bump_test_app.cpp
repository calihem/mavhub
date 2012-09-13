#include "bump_test_app.h"

#ifdef HAVE_MAVLINK_H

#include <sstream>
#include <sys/time.h>

#include "core/logger.h"
#include "utility.h"

using namespace std;
using namespace cpp_pthread;

namespace mavhub {

BumpTestApp::BumpTestApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) :
	AppInterface("bump_test_app", loglevel),
	MavlinkAppLayer("bump_test_app", loglevel),
  target_system(1),
  target_component(0),
  altitude(0.0),
  do_bump(false),
	execTiming(25)
#ifdef BUMP_TEST_LOG
	, logFile("bump_test_log.data")
#endif
	{
	
  assign_variable_from_args(target_system);
	assign_variable_from_args(target_component);

#ifdef BUMP_TEST_LOG
  logFile << "# time [ms]"
    << " | a[0] | a[1] | a[2] | a[3] | a[4] | a[5]"
    << " | d[0] | d[1] | d[2] | d[3]"
		<< std::endl;
	logFile << "#" << std::endl;
#endif
}

BumpTestApp::~BumpTestApp() {}

void BumpTestApp::handle_input(const mavlink_message_t &msg) {
	Logger::log(name(), "got mavlink_message", static_cast<int>(msg.msgid),
		"from", static_cast<int>(msg.sysid),
		static_cast<int>(msg.compid),
		Logger::LOGLEVEL_DEBUG, _loglevel);

  switch(msg.msgid) {
		case MAVLINK_MSG_ID_HUCH_POTIBOX:
			int16_t analogPoti[6];
			int16_t digitalPoti[4];
			mavlink_msg_huch_potibox_get_a(&msg, analogPoti);
			mavlink_msg_huch_potibox_get_d(&msg, digitalPoti);
      // TODO: handle actual poti values
    break;
		case MAVLINK_MSG_ID_VFR_HUD:
      double alt = mavlink_msg_vfr_hud_get_alt(&msg);
      altitude = double(alt)*100.0; // m -> cm
    break;
  }
}

void BumpTestApp::run()
{
	log(name(), ": running", Logger::LOGLEVEL_DEBUG);
  
  uint64_t dt = 0;
  int wait_time;
  
  int msgRoll   = 0;
  int msgPitch  = 0;
  int msgYaw    = 0;
  int msgThrust = 530;
  
  // request altitude information from APM
	request_data_stream(target_system, target_component, MAV_DATA_STREAM_EXTRA2, 25);

	while( !interrupted() ) {
    wait_time = execTiming.calcSleeptime();
    usleep(wait_time);
    dt = execTiming.updateExecStats();

    std::cout << dt << std::endl;

    if(do_bump)
    {
      
      msgRoll   = 0;
      msgPitch  = 0;
      msgYaw    = 0;
      msgThrust = 530;
      
      mavlink_message_t ctrlMsg;
      
      // first wait 1s
      msgThrust = 530;
      mavlink_msg_huch_ext_ctrl_pack(
        system_id(),
        component_id,
        &ctrlMsg,
        target_system,
        target_component,
        8, // mask
        msgRoll,
        msgPitch,
        msgYaw,
        msgThrust
      );
      send(ctrlMsg);
      usleep(1000);
      
      // then hooverThrust + thrust for 1s
      msgThrust = 530 + 50;
      mavlink_msg_huch_ext_ctrl_pack(
        system_id(),
        component_id,
        &ctrlMsg,
        target_system,
        target_component,
        8, // mask
        msgRoll,
        msgPitch,
        msgYaw,
        msgThrust
      );
      send(ctrlMsg);
      usleep(1000);
      
      // then hooverThrust - thrust for 1s
      msgThrust = 530 - 50;
      mavlink_msg_huch_ext_ctrl_pack(
        system_id(),
        component_id,
        &ctrlMsg,
        target_system,
        target_component,
        8, // mask
        msgRoll,
        msgPitch,
        msgYaw,
        msgThrust
      );
      send(ctrlMsg);
      usleep(1000);
      
      // then wait 1s
      msgThrust = 530;
      mavlink_msg_huch_ext_ctrl_pack(
        system_id(),
        component_id,
        &ctrlMsg,
        target_system,
        target_component,
        8, // mask
        msgRoll,
        msgPitch,
        msgYaw,
        msgThrust
      );
      send(ctrlMsg);
      usleep(1000);
      
    }
    
    int msgRoll   = 0;
    int msgPitch  = 0;
    int msgYaw    = 0;
    int msgThrust = 530;
    
    mavlink_message_t ctrlMsg;
    mavlink_msg_huch_ext_ctrl_pack(
      system_id(),
      component_id,
      &ctrlMsg,
      target_system,
      target_component,
      8, // mask
      msgRoll,
      msgPitch,
      msgYaw,
      msgThrust
    );
    send(ctrlMsg);
  }
}

}

#endif // HAVE_MAVLINK_H
