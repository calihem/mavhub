#include "fiducal_control_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_OPENCV2
#include <opencv/cv.h>

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

using namespace std;
using namespace cpp_pthread;

namespace mavhub {

FiducalControlApp::FiducalControlApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) :
	AppInterface("slam_app", loglevel),
	MavlinkAppLayer("slam_app", loglevel),
  rvec(cv::Mat::zeros(3, 1, CV_32FC1)),
  tvec(cv::Mat::zeros(3, 1, CV_32FC1)),
  fvec(cv::Mat::zeros(3, 1, CV_32FC1)),
  pidYaw(0.1, 0.01, 0.001, 100.0, 0.1, 0.0),
  pidLatX(0.1, 0.01, 0.001, 100.0, 0.1, 0.0),
  pidLatY(0.1, 0.01, 0.001, 100.0, 0.1, 0.0),
  pidAlt(0.1, 0.01, 0.001, 100.0, 0.1, 100.0),
	execTiming(100)
#ifdef FIDUCAL_CONTROL_LOG
	, logFile("fiducal_control_log.data")
#endif
	{

#ifdef FIDUCAL_CONTROL_LOG
	logFile << "# time [ms]"
    << " | tag_rotation[0] | tag_rotation[1] | tag_rotation[2]"
    << " | tag_translation[0] | tag_translation[1] | tag_translation[2]"
    << " | camera_position[0] | camera_position[1] | camera_position[2]"
		<< std::endl;
	logFile << "#" << std::endl;
	logFile << setprecision(5) << fixed << setfill(' ');
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
    wait_time = execTiming.calcSleeptime();
    usleep(wait_time);
    dt = execTiming.updateExecStats();
    // do stuff
    rvec = DataCenter::get_fiducal_rot_raw();
    tvec = DataCenter::get_fiducal_trans_raw();
    cv::Mat rmat;
    cv::Rodrigues(-rvec, rmat);
    fvec = rmat*tvec;
    // now control:
    double ctrlYaw = pidYaw.step(rvec.at<double>(3), dt);
    double ctrlLatX = pidLatX.step(fvec.at<double>(1), dt);
    double ctrlLatY = pidLatY.step(fvec.at<double>(2), dt);
    double ctrlAlt = pidAlt.step(fvec.at<double>(3), dt); 
  }
}

} // namespace mavhub

#endif // HAVE_OPENCV2 && CV_MINOR_VERSION > 1

#endif // HAVE_MAVLINK_H
