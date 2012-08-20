#include "fiducal_control_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_OPENCV2
#include <opencv/cv.h>

#include <sstream>

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

using namespace std;
using namespace cpp_pthread;

namespace mavhub {

FiducalControlApp::FiducalControlApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) :
	AppInterface("fiducal_control_app", loglevel),
	MavlinkAppLayer("fiducal_control_app", loglevel),
  target_system(1),
  target_component(1),
  rvec(cv::Mat::zeros(3, 1, CV_32FC1)),
  tvec(cv::Mat::zeros(3, 1, CV_32FC1)),
  fvec(cv::Mat::zeros(3, 1, CV_32FC1)),
  pidYaw(1e-1, 0.0, 0.0, 1e1, 1.0, 0.0),
  pidLatX(1e-1, 0.0, 0.0, 1e1, 1.0, 0.0),
  pidLatY(1e-1, 0.0, 0.0, 1e1, 1.0, 0.0),
  pidAlt(1e-1, 0.0, 0.0, 1e1, 1.0, 50.0),
	execTiming(100)
#ifdef FIDUCAL_CONTROL_LOG
	, logFile("fiducal_control_log.data")
#endif
	{
	
  assign_variable_from_args(target_system);
	assign_variable_from_args(target_component);

#ifdef FIDUCAL_CONTROL_LOG
	logFile << "# time [ms]"
    << " | ctrlYaw | ctrlLatX | ctrlLatY | ctrlAlt"
		<< std::endl;
	logFile << "#" << std::endl;
	logFile << setprecision(5) << fixed << setfill(' ');
#endif
}

FiducalControlApp::~FiducalControlApp() {}

void FiducalControlApp::handle_input(const mavlink_message_t &msg) {
	Logger::log(name(), "got mavlink_message", static_cast<int>(msg.msgid),
		"from", static_cast<int>(msg.sysid),
		static_cast<int>(msg.compid),
		Logger::LOGLEVEL_DEBUG, _loglevel);

	switch(msg.msgid) {
		case MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT:
			if( (mavlink_msg_set_local_position_setpoint_get_target_system(&msg) == system_id()) ) {
        if(mavlink_msg_local_position_setpoint_get_coordinate_frame(&msg) == MAV_FRAME_LOCAL_ENU) {
          double setYaw = mavlink_msg_set_local_position_setpoint_get_yaw(&msg);
          double setLatX = mavlink_msg_set_local_position_setpoint_get_x(&msg);
          double setLatY = mavlink_msg_set_local_position_setpoint_get_y(&msg);
          double setAlt = mavlink_msg_set_local_position_setpoint_get_z(&msg);
          
          Lock input_lock(input_mutex);
         
          pidYaw.setPoint = setYaw;
          pidLatX.setPoint = setLatX;
          pidLatY.setPoint = setLatY;
          pidAlt.setPoint = setAlt;
        }
      }
    break;
		case MAVLINK_MSG_ID_HUCH_POTIBOX:
			int16_t analogPoti[6];
			int16_t digitalPoti[4];
			mavlink_msg_huch_potibox_get_a(&msg, analogPoti);
			mavlink_msg_huch_potibox_get_d(&msg, digitalPoti);
      // TODO: handle actual poti values
    break;
  }
}

void FiducalControlApp::run()
{
	log(name(), ": running", Logger::LOGLEVEL_DEBUG);
  
  uint64_t dt = 0;
  int wait_time;

	while( !interrupted() ) {
    // get correct timings
    wait_time = execTiming.calcSleeptime();
    usleep(wait_time);
    dt = execTiming.updateExecStats();
    // do stuff
    rvec = DataCenter::get_fiducal_rot_raw();
    tvec = DataCenter::get_fiducal_trans_raw();

    if(!rvec.empty() && !tvec.empty())
    {
			Lock input_lock(input_mutex);
      cv::Mat rmat;
      cv::Rodrigues(-rvec, rmat);
      fvec = rmat*tvec;
      // now control:
      double ctrlYaw = pidYaw.step(rvec.at<double>(1), dt);
      double ctrlLatX = pidLatX.step(fvec.at<double>(0), dt);
      double ctrlLatY = pidLatY.step(fvec.at<double>(1), dt);
      double ctrlAlt = pidAlt.step(fvec.at<double>(2), dt); 

      std::stringstream ss;
      ss 
        << std::setw(8) << std::setprecision(6) << ctrlYaw << " "
        << std::setw(8) << std::setprecision(6) << ctrlLatX << " "
        << std::setw(8) << std::setprecision(6) << ctrlLatY << " "
        << std::setw(8) << std::setprecision(6) << ctrlAlt << " "
        << std::endl;
      std::cout << ss.str() << std::endl;
#ifdef FIDUCAL_CONTROL_LOG
      logFile 
	      << std::setw(14) << get_time_ms()
        << std::setw(10) << std::setprecision(6) << ctrlYaw << " "
        << std::setw(10) << std::setprecision(6) << ctrlLatX << " "
        << std::setw(10) << std::setprecision(6) << ctrlLatY << " "
        << std::setw(10) << std::setprecision(6) << ctrlAlt << " "
        << std::endl;
#endif

      mavlink_message_t ctrlMsg;
      mavlink_msg_huch_ext_ctrl_pack(
        system_id(),
        component_id,
        &ctrlMsg,
        target_system,
        target_component,
        0, // mask
        ctrlLatY * 100, // roll
        ctrlLatX * 100, // pitch
        ctrlYaw * 100, // yaw
        ctrlAlt * 100 // thrust 0..1000
      );
      send(ctrlMsg);
    }
  }
}

}

#endif // HAVE_OPENCV2

#endif // HAVE_MAVLINK_H
