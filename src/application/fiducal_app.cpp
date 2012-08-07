#include "fiducal_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER

#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION >= 3)

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

using namespace std;
using namespace cpp_pthread;

namespace mavhub {

FiducalApp::FiducalApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) :
	AppInterface("slam_app", loglevel),
	MavlinkAppLayer("slam_app", loglevel),
	hub::gstreamer::VideoClient(),
	cam_matrix(3, 3, CV_32FC1),
	dist_coeffs( cv::Mat::zeros(4, 1, CV_32FC1) ),
#ifdef FIDUCAL_LOG
	, log_file("fiducal_log.data")
#endif
	{

	pthread_mutex_init(&sync_mutex, NULL);

	// set sink name
	std::map<std::string,std::string>::const_iterator iter = args.find("sink");
	if( iter != args.end() ) {
		sink_name.assign(iter->second);
	} else {
		log(name(), ": sink argument missing", Logger::LOGLEVEL_DEBUG);
		sink_name.assign("sink0");
	}

	get_value_from_args("out_stream", with_out_stream);

	// get calibration data of camera
	//FIXME: use image dimensions for default camera matrix
	cam_matrix = (cv::Mat_<double>(3,3) << 1.0, 0.0, 160.0, 0.0, 1.0, 120.0, 0.0, 0.0, 1.0);
	string calib_filename;
	get_value_from_args("calibration_data", calib_filename);
	if(!calib_filename.empty())
		load_calibration_data(calib_filename);

#ifdef FIDUCAL_LOG
	log_file << "# time [ms]"
		<< " | IMU roll angle [rad] | IMU pitch angle [rad] | IMU yaw angle [rad]"
		<< " | roll angular speed [rad/s] | pitch angular speed [rad/s] | yaw angular speed [rad/s]"
		<< " | altitude [cm]"
		<< " | Cam roll angle [rad] | Cam pitch angle [rad] | Cam yaw angle [rad]"
		<< " | Cam x position [cm] | Cam y positionn [cm] | Cam z position [cm]"
		<< std::endl;
	log_file << "#" << std::endl;
	log_file << setprecision(5) << fixed << setfill(' ');
#endif
}

FiducalApp::~FiducalApp() {}

void FiducalApp::handle_input(const mavlink_message_t &msg) {
}

void FiducalApp::handle_video_data(const unsigned char *data, const int width, const int height, const int bpp) {
	if(!data) return;

	Logger::log(name(), ": got new video data of size", width, "x", height, Logger::LOGLEVEL_DEBUG, _loglevel);
	if(bpp != 8) {
		log(name(), ": unsupported video data with bpp =", bpp, Logger::LOGLEVEL_WARN);
		return;
	}

	// make a matrix header for captured data
	unsigned char *image_data = const_cast<unsigned char*>(data);
	cv::Mat video_data(height, width, CV_8UC1, image_data);

	Lock sync_lock(sync_mutex);
  // TODO: new video data handling after mutex
	new_video_data = true;
}

} // namespace mavhub

#endif // HAVE_OPENCV2 && CV_MINOR_VERSION > 1

#endif // HAVE_GSTREAMER

#endif // HAVE_MAVLINK_H
