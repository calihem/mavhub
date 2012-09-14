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
  hooverThrust(500),
  do_bump(false),
	execTiming(100)
#ifdef BUMP_TEST_LOG
	, logFile("bump_test_log.data")
#endif
	{
	
  assign_variable_from_args(target_system);
	assign_variable_from_args(target_component);

#ifdef BUMP_TEST_LOG
  logFile << "# time [ms]"
    << "| " << "msgThrust" << " | " << "altitude"
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
      if(analogPoti[0] > 100)
        do_bump = true;
      hooverThrust = 500 + int((analogPoti[4]-500)/10.0);
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
  int msgThrust = 0;
  
  // request altitude information from APM
	request_data_stream(target_system, target_component, MAV_DATA_STREAM_EXTRA2, 100);

  std::cout << "foo" << std::endl;

  double T = 0.0;

	while( !interrupted() ) {
    wait_time = execTiming.calcSleeptime();
    usleep(wait_time);
    dt = execTiming.updateExecStats();

    msgRoll   = 0;
    msgPitch  = 0;
    msgYaw    = 0;
    msgThrust = 0;

    if(do_bump)
    {
      T += dt*1e-6;
      if(T < 3.0)
        msgThrust = 0.0;
      if(T >= 4.0 && T < 5.0)
        msgThrust = hooverThrust + 50;
      if(T >= 5.0 && T < 6.0)
        msgThrust = hooverThrust - 50;
      if(T >= 6.0 && T < 7.0)
        msgThrust = 0.0;
      if(T > 7.0)
      {
        T = 0.0;
        do_bump = false;
      }
    }
    
    std::cout << do_bump << " " << T << " " << msgThrust << " " << hooverThrust << std::endl;
#ifdef BUMP_TEST_LOG
    logFile 
      << get_time_ms() << " "
      << msgThrust << " "
      << altitude  << " "
      << std::endl;
#endif

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
