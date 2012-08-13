#include "fiducal_app.h"

#ifdef HAVE_MAVLINK_H

#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION >= 3)

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

using namespace std;
using namespace cpp_pthread;

namespace mavhub {

FiducalControlApp::FiducalControlApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) :
	AppInterface("slam_app", loglevel),
	MavlinkAppLayer("slam_app", loglevel),
  rcev(cv::Mat::zeros(3, 1, CV_32FC1)),
  tcev(cv::Mat::zeros(3, 1, CV_32FC1)),
  fcev(cv::Mat::zeros(3, 1, CV_32FC1)),
  setpoint(cv::Mat::zeros(3, 1, CV_32FC1)),
	execTiming(100),
#ifdef FIDUCAL_LOG
	, log_file("fiducal_log.data")
#endif
	{

#ifdef FIDUCAL_LOG
	log_file << "# time [ms]"
    << " | tag_rotation[0] | tag_rotation[1] | tag_rotation[2]"
    << " | tag_translation[0] | tag_translation[1] | tag_translation[2]"
    << " | camera_position[0] | camera_position[1] | camera_position[2]"
		<< std::endl;
	log_file << "#" << std::endl;
	log_file << setprecision(5) << fixed << setfill(' ');
#endif
}

FiducalControlApp::~FiducalControlApp() {}

void FiducalControlApp::handle_input(const mavlink_message_t &msg) {
}

void FiducalControlApp::run()
{
	log(name(), ": running", Logger::LOGLEVEL_DEBUG);
  
  uint64_t dt = 0;
  int wait_time;

	while( !interrupted() ) {
    // get correct timings
    wait_time = exec_tmr->calcSleeptime();
    usleep(wait_time);
    dt = exec_tmr->updateExecStats();
    // do stuff
    rvec = Datacenter::get_fiducal_rot_raw();
    tvec = Datacenter::get_fiducal_trans_raw();
    cv::Mat rmat;
    cv::Rodrigues(-rvec, rmat);
    fvec = rmat*tvec;
    // now control:
    // r[3] = yaw
    // fvec[3] = altitude
    // fvev[1-2] = lateral
  }
}

} // namespace mavhub

#endif // HAVE_OPENCV2 && CV_MINOR_VERSION > 1

#endif // HAVE_MAVLINK_H
