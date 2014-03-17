#include "fiducal_control_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_OPENCV2
//#include <opencv/cv.h>

#include <sstream>
#include <sys/time.h>

#include "core/logger.h"
#include "core/datacenter.h"
#include "lib/hub/time.h"

using namespace std;
using namespace cpp_pthread;
using namespace hub;

namespace mavhub {

FiducalControlApp::FiducalControlApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) :
	AppInterface("fiducal_control_app", loglevel),
	MavlinkAppLayer("fiducal_control_app", loglevel),
  target_system(1),
  target_component(1),
  rvec(cv::Mat::zeros(3, 1, CV_32FC1)),
  tvec(cv::Mat::zeros(3, 1, CV_32FC1)),
  fvec(cv::Mat::zeros(3, 1, CV_32FC1)),
  pidYaw(0.0, 0.0, 0.0, 1e1, 1.0, 0.0),
  pidLatX(0.0, 0.0, 0.0, 1e1, 1.0, 0.0),
  pidLatY(0.0, 0.0, 0.0, 1e1, 1.0, 0.0),
  pidAlt(0.0, 0.0, 0.0, 1e1, 1.0, 50.0),
  minimalThrust(150),
  systemGain(100),
  hooverThrust(300),
  maxRollPitch(1000),
	execTiming(25)
#ifdef FIDUCAL_CONTROL_LOG
	, ctrlLogFile("fiducal_control_ctrl_log.data")
	, potiLogFile("fiducal_control_poti_log.data")
#endif
	{
	
  assign_variable_from_args(target_system);
	assign_variable_from_args(target_component);

#ifdef FIDUCAL_CONTROL_LOG
	ctrlLogFile << "# time [ms]"
    << " | ctrlYaw | ctrlLatX | ctrlLatY | ctrlAlt"
		<< std::endl;
	ctrlLogFile << "#" << std::endl;
	ctrlLogFile << setprecision(5) << fixed << setfill(' ');
	
  potiLogFile << "# time [ms]"
    << " | a[0] | a[1] | a[2] | a[3] | a[4] | a[5]"
    << " | d[0] | d[1] | d[2] | d[3]"
		<< std::endl;
	potiLogFile << "#" << std::endl;
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
#ifdef FIDUCAL_CONTROL_LOG
      potiLogFile 
	      << std::setw(14) << get_time_ms()
        << std::setw(6) << analogPoti[0]  << " "
        << std::setw(6) << analogPoti[1]  << " "
        << std::setw(6) << analogPoti[2]  << " "
        << std::setw(6) << analogPoti[3]  << " "
        << std::setw(6) << analogPoti[4]  << " "
        << std::setw(6) << analogPoti[5]  << " "
        << std::setw(6) << digitalPoti[0]  << " "
        << std::setw(6) << digitalPoti[1]  << " "
        << std::setw(6) << digitalPoti[2]  << " "
        << std::setw(6) << digitalPoti[3]  << " "
        << std::endl;
#endif
//      pidYaw.Kp;
//      pidYaw.Ki;
//      pidYaw.Kd;
//      pidYaw.setPoint;
//      pidLatX.Kp;
//      pidLatX.Ki;
//      pidLatX.Kd;
//      pidLatX.setPoint;
//      pidLatY.Kp;
//      pidLatY.Ki;
//      pidLatY.Kd;
//      pidLatY.setPoint;
      pidAlt.Kp = double(analogPoti[0]) / 1024 * 1e-1;
      pidAlt.Ki = double(analogPoti[1]) / 1024 * 2e-2;
      pidAlt.Kd = double(analogPoti[2]) / 1024 * 5e-3;
      pidAlt.setPoint = double(analogPoti[3]) / 1024 * 5e2;

      systemGain = int(double(analogPoti[4]) / 1024 * 500);
      hooverThrust = int(double(analogPoti[5]) / 1024 * 1000);
      
    break;
  }
}

void FiducalControlApp::run()
{
	log(name(), ": running", Logger::LOGLEVEL_DEBUG);
  
  uint64_t dt = 0;
  int wait_time;

  cv::Mat fvecOld;
  struct timeval lastMesTime;
	gettimeofday(&lastMesTime, NULL);
    
  int msgRoll   = 0;
  int msgPitch  = 0;
  int msgYaw    = 0;
  int msgThrust = hooverThrust;

	while( !interrupted() ) {
    // get correct timings
    wait_time = execTiming.calcSleeptime();
    usleep(wait_time);
    dt = execTiming.updateExecStats();
    // do stuff
    rvec = DataCenter::get_fiducal_rot_raw();
    tvec = DataCenter::get_fiducal_trans_raw();
  
    msgRoll   = 0;
    msgPitch  = 0;
    msgYaw    = 0;
    msgThrust = hooverThrust;
        
    if(!rvec.empty() && !tvec.empty())
    {
			Lock input_lock(input_mutex);
      cv::Mat rmat;
      cv::Rodrigues(-rvec, rmat);
      fvec = rmat*tvec;

      cv::Scalar fvecDiff = cv::sum(cv::abs(fvec-fvecOld));
      if(fvecDiff[0] > 1e-5)
      {
	      gettimeofday(&lastMesTime, NULL);
        fvec.copyTo(fvecOld);

        // now control:
        double ctrlYaw = pidYaw.step(rvec.at<double>(1), dt);
        double ctrlLatX = pidLatX.step(fvec.at<double>(0), dt);
        double ctrlLatY = pidLatY.step(fvec.at<double>(1), dt);
        double ctrlAlt = pidAlt.step(fvec.at<double>(2), dt); 

#ifdef FIDUCAL_CONTROL_LOG
        ctrlLogFile 
          << std::setw(14) << get_time_ms()
          << std::setw(10) << std::setprecision(6) << ctrlYaw << " "
          << std::setw(10) << std::setprecision(6) << ctrlLatX << " "
          << std::setw(10) << std::setprecision(6) << ctrlLatY << " "
          << std::setw(10) << std::setprecision(6) << ctrlAlt << " "
          << std::endl;
#endif
        msgRoll = ctrlLatX * systemGain;
        msgPitch = ctrlLatY * systemGain;
        msgYaw = ctrlYaw * systemGain;
        msgThrust = ctrlAlt * systemGain + hooverThrust; // 0..systemGain0

        msgRoll = msgRoll > maxRollPitch ? maxRollPitch : msgRoll;
        msgPitch = msgPitch > maxRollPitch ? maxRollPitch : msgPitch;
        msgThrust = msgThrust < minimalThrust? minimalThrust : msgThrust;

      }else
      {
        struct timeval currTime;
		    gettimeofday(&currTime, NULL);
        double diffTime = (currTime.tv_sec + currTime.tv_usec*1e-6) - (lastMesTime.tv_sec + lastMesTime.tv_usec*1e-6);
        if(diffTime > 1.5)
        {
          msgRoll   = 0;
          msgPitch  = 0;
          msgYaw    = 0;
          msgThrust = hooverThrust;
        }
      }
    }
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

#endif // HAVE_OPENCV2

#endif // HAVE_MAVLINK_H
