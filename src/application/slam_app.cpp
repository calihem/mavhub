#include "slam_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER

#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION >= 3)

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"
#include "lib/slam/pose.h"
#include "lib/slam/tracker.h"
#include "lib/hub/time.h"

#include <sstream> //istringstream

using namespace std;
using namespace cpp_pthread;
using namespace hub::slam;
using namespace hub;

namespace mavhub {

SLAMApp::SLAMApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) :
	AppInterface("slam_app", loglevel),
	MavlinkAppLayer("slam_app", loglevel),
	hub::gstreamer::VideoClient(),
	with_out_stream(false),
	use_extrinsic_guess(true),
	take_new_image(1),
	target_system(7),
	target_component(1),
	imu_rate(10),
	channel_rate(0),
	parameter_vector(6, 0),
	trigger_channel(0),
	altitude(0.0),
	tracker(NULL),
#ifdef SLAM_LOG
	log_file("slam_log.data")
#endif
	{

	pthread_mutex_init(&sync_mutex, NULL);

	// invalidate attitude
	bzero(&attitude, sizeof(mavlink_attitude_t));

	// set sink name
	std::map<std::string,std::string>::const_iterator iter = args.find("sink");
	if( iter != args.end() ) {
		sink_name.assign(iter->second);
	} else {
		log(name(), ": sink argument missing", Logger::LOGLEVEL_DEBUG);
		sink_name.assign("sink0");
	}

	get_value_from_args("out_stream", with_out_stream);

	//TODO: pipe_in, pipe_out

	get_value_from_args("extrinsic_guess", use_extrinsic_guess);

	assign_variable_from_args(target_system);
	assign_variable_from_args(target_component);
	assign_variable_from_args(imu_rate);
	assign_variable_from_args(channel_rate);

	// get camera calibration data
	unsigned int image_width = 320, image_height = 240;
	assign_variable_from_args(image_width);
	assign_variable_from_args(image_height);
	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 1.0, 0.0, image_width/2.0, 0.0, 1.0, image_height/2.0, 0.0, 0.0, 1.0);
	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);
	string calib_filename;
	get_value_from_args("calibration_data", calib_filename);
	if( !calib_filename.empty() ) {
		cv::FileStorage fs(calib_filename, cv::FileStorage::READ);
		if( fs.isOpened() ) {
			image_width = (int)fs["image_width"];
			image_height = (int)fs["image_height"];
			fs["camera_matrix"] >> camera_matrix;
			fs["distortion_coefficients"] >> distortion_coefficients;
		} else {
			log("Can't open calibration data", calib_filename, Logger::LOGLEVEL_DEBUG);
		}
	}

	//FIXME: move initialization to run
	// initialize tracker
	tracker = new hub::slam::Tracker(image_width, image_height, camera_matrix, distortion_coefficients);
	if(!tracker) {
		log("Can't initialize tracker", Logger::LOGLEVEL_ERROR);
	}

#ifdef SLAM_LOG
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

SLAMApp::~SLAMApp() {
	if(tracker)
		delete tracker;
}

void SLAMApp::handle_input(const mavlink_message_t &msg) {
	Logger::log(name(), "got mavlink_message", static_cast<int>(msg.msgid),
		"from", static_cast<int>(msg.sysid),
		static_cast<int>(msg.compid),
		Logger::LOGLEVEL_DEBUG, _loglevel);

	switch(msg.msgid) {
/*		case MAVLINK_MSG_ID_HUCH_ACTION:
			if( (mavlink_msg_huch_action_get_target(&msg) == system_id()) ) {
// 			&& (mavlink_msg_action_get_target_component(&msg) == component_id) ) {
				uint8_t action_id = mavlink_msg_huch_action_get_action(&msg);
				if(action_id == MAV_ACTION_GET_IMAGE) {
					Lock sync_lock(sync_mutex);
					// new image with ACK
					take_new_image = 3;
				}
			}
			break;
*/
		case MAVLINK_MSG_ID_ATTITUDE:
			if( (msg.sysid == system_id()) ) {
				Lock sync_lock(sync_mutex);
				mavlink_msg_attitude_decode(&msg, &attitude);
				// take system time for attitude
				attitude.time_boot_ms = get_time_ms();
			}
			break;
/*		case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
			if( (msg.sysid == system_id()) ) {
				uint16_t channel6 = mavlink_msg_rc_channels_raw_get_chan6_raw(&msg);
				log("got channel 6", channel6, Logger::LOGLEVEL_DEBUG);
				if( !trigger_channel ) { //first reading
					trigger_channel = channel6;
					break;
				}
				if(trigger_channel + 600 < channel6) {// chan6 switched to high
					log("trigger channel switched to high", Logger::LOGLEVEL_DEBUG);
					Lock sync_lock(sync_mutex);
					take_new_image = 1;
				}
				trigger_channel = channel6;
			}
			break;
*/
		case MAVLINK_MSG_ID_VFR_HUD:
			if( (msg.sysid == system_id()) ) {
				// scale altitude from m to cm
				altitude =  100.0 * mavlink_msg_vfr_hud_get_alt(&msg);
				log("set altitude to", altitude, Logger::LOGLEVEL_DEBUG);
			}
			break;
		default: break;
	}
}

void SLAMApp::handle_video_data(const unsigned char *data, const int width, const int height, const int bpp) {
	if(!data) return;

	Logger::log(name(), ": got new video data of size", width, "x", height, Logger::LOGLEVEL_DEBUG, _loglevel);
	if(bpp != 8) {
		log(name(), ": unsupported video data with bpp =", bpp, Logger::LOGLEVEL_WARN);
		return;
	}

	//FIXME: dirty hack
	//throw first frames away
	static int counter = 0;
	if(counter < 5) {
		counter++;
		return;
	}

	// make a matrix header for captured data
	unsigned char *image_data = const_cast<unsigned char*>(data);
	cv::Mat video_data(height, width, CV_8UC1, image_data);

	Lock sync_lock(sync_mutex);
	video_data.copyTo(image_buffer);
// 	memcpy(&(attitudes.back()), &attitude, sizeof(mavlink_attitude_t));

	new_video_data = true;
}

void SLAMApp::run() {
	log(name(), ": running", Logger::LOGLEVEL_DEBUG);


	if(Core::video_server) {
		int rc = Core::video_server->bind2appsink( dynamic_cast<VideoClient*>(this), sink_name.c_str());
		Logger::log(name(), ": binded to", sink_name, rc, Logger::LOGLEVEL_DEBUG, _loglevel);
	} else {
		log(name(), ": video server not running", Logger::LOGLEVEL_WARN);
		return;
	}

	// request IMU data
	Logger::log(name(), ": request data stream extra1 from",
		target_system, ":", target_component,
		Logger::LOGLEVEL_DEBUG, _loglevel);
	request_data_stream(target_system,
		target_component,
		MAV_DATA_STREAM_EXTRA1,
		imu_rate);
	// request altitude (is stream extra2 on arduocpter)
	Logger::log(name(), ": request data stream extra2 from",
		target_system, ":", target_component,
		Logger::LOGLEVEL_DEBUG, _loglevel);
	request_data_stream(target_system,
		target_component,
		MAV_DATA_STREAM_EXTRA2,
		imu_rate);
	// request RC channels
	Logger::log(name(), ": request data stream RC channels from",
		target_system, ":", target_component,
		Logger::LOGLEVEL_DEBUG, _loglevel);
	request_data_stream(target_system,
		target_component,
		MAV_DATA_STREAM_RC_CHANNELS,
		5);

	while( !interrupted() ) {
		log(name(), "enter main loop", Logger::LOGLEVEL_DEBUG);
		{ Lock sync_lock(sync_mutex);
			if(new_video_data) {
				//FIXME: set parameter_vector
				tracker->track_camera(image_buffer, parameter_vector, altitude);
				new_video_data = false;
			}
		}
		//FIXME: remove usleep
		usleep(5000);
	}

	//unbind from video server
	Core::video_server->release( dynamic_cast<VideoClient*>(this) );
	log(name(), ": stop running", Logger::LOGLEVEL_DEBUG);
}

} // namespace mavhub

#endif // HAVE_OPENCV2 && CV_MINOR_VERSION > 1

#endif // HAVE_GSTREAMER

#endif // HAVE_MAVLINK_H
