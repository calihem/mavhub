#include "slam_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER

#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION >= 2)

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"
#include "lib/slam/pose.h"

#include <sstream> //istringstream

using namespace std;
using namespace cpp_pthread;
using namespace hub::slam;

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
	trigger_channel(0),
	altitude(0.0),
	cam_matrix(3, 3, CV_32FC1),
	dist_coeffs( cv::Mat::zeros(4, 1, CV_32FC1) ),
	scenes(2),
	attitudes(2),
	feature_detector(60, 3) //threshold, octaves
// 	rotation_vector( cv::Mat::zeros(3, 1, CV_64FC1) ),
// 	translation_vector( cv::Mat::zeros(3, 1, CV_64FC1) )
#ifdef SLAM_LOG
	, log_file("slam_log.data")
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

	// get calibration data of camera
	//FIXME: use image dimensions for default camera matrix
	cam_matrix = (cv::Mat_<double>(3,3) << 1.0, 0.0, 160.0, 0.0, 1.0, 120.0, 0.0, 0.0, 1.0);
	string calib_filename;
	get_value_from_args("calibration_data", calib_filename);
	if(!calib_filename.empty())
		load_calibration_data(calib_filename);

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

SLAMApp::~SLAMApp() {}

void SLAMApp::extract_features() {
	//FIXME: remove benchmark
	uint64_t start_time = get_time_ms();

	if(landmarks.keypoints.empty()) {
		if(altitude == 0.0) { // aircraft is on the ground
			log("reject initial feature extraction on ground", Logger::LOGLEVEL_DEBUG);
			return;
		}

		// get features from image
		feature_detector.detect(scenes.front(), landmarks.keypoints);
		if(landmarks.keypoints.empty()) {
			log("didn't found reference features", Logger::LOGLEVEL_DEBUG);
			take_new_image = 1;
			return;
		}

		//TODO: filter features (shi-tomasi)

		// calculate descriptors which can remove or add some of the keypoints
		descriptor_extractor.compute(scenes.front(), landmarks.keypoints, landmarks.descriptors);

		// calculate corresponding 3D object points
		keypoints_to_objectpoints(landmarks.keypoints,
			altitude, //distance from ground
			landmarks.objectpoints,
			cam_matrix,
			dist_coeffs);

		//FIXME: deprecated
		// init counters
		landmarks.counters.assign(landmarks.keypoints.size(), 100);
		return;
	}

	// get features from image
	vector<cv::KeyPoint> keypoints;
	feature_detector.detect(scenes.back(), keypoints);
	if(keypoints.empty()) {
		log("didn't found features", Logger::LOGLEVEL_DEBUG);
		return;
	}

	uint64_t stop_time = get_time_ms();
	log( "feature extraction needed", stop_time-start_time, "ms", Logger::LOGLEVEL_DEBUG);
	start_time = stop_time;

	//TODO: filter features (shi-tomasi)

	// calculate descriptors
	cv::Mat descriptors;
	descriptor_extractor.compute(scenes.back(), keypoints, descriptors);
	if(keypoints.empty()) {
		log("keypoints empty after descriptor computation", Logger::LOGLEVEL_DEBUG);
		return;
	}

	stop_time = get_time_ms();
	log( "descriptors needed", stop_time-start_time, "ms", Logger::LOGLEVEL_DEBUG);
	start_time = stop_time;

	// create filter mask
// 	cv::Mat mask = cv::Mat::ones(landmarks.keypoints.size(), keypoints.size(), CV_8UC1);
// 	filter_landmarks(landmarks, mask);

	std::vector<cv::DMatch> matches;
	matcher.match(landmarks.descriptors, descriptors, matches);
	if(matches.empty()) {
		log("no matches fount", Logger::LOGLEVEL_DEBUG);
		return;
	}
	stop_time = get_time_ms();
	log( "forward matching needed", stop_time-start_time, "ms", Logger::LOGLEVEL_DEBUG);
	start_time = stop_time;

	std::vector<char> matches_mask(matches.size(), 1);

	filter_matches_by_robust_distribution(landmarks.keypoints, keypoints, matches, matches_mask);

	stop_time = get_time_ms();
	log( "distribution filtering needed", stop_time-start_time, "ms", Logger::LOGLEVEL_DEBUG);
	start_time = stop_time;

	// get change of attitude
	//TODO: time check of attitude
// 	mavlink_attitude_t attitude_change;
// 	attitude_change.roll = new_attitude.roll - old_attitude.roll;
// 	attitude_change.pitch = new_attitude.pitch - old_attitude.pitch;
// 	attitude_change.yaw = new_attitude.yaw - old_attitude.yaw;
// 	Logger::log(name(), "Changed attitude:",
// 		rad2deg(attitude_change.roll),
// 		rad2deg(attitude_change.pitch),
// 		rad2deg(attitude_change.yaw),
// 		Logger::LOGLEVEL_DEBUG, _loglevel);
	
	// calculate transformation matrix
// 	double distance = 1.0; //FIXME: use altitude information
// 	double factor = 300.0;
// 	float delta_x = factor*2.0*distance*sin(attitude_change.roll/2);
// 	float delta_y = factor*2.0*distance*sin(attitude_change.pitch/2);
	// rotate image
// 	cv::Point center(new_image.cols/2, new_image.rows/2);
// 	cv::Mat rotation_matrix = getRotationMatrix2D(center, -rad2deg(attitude_change.yaw), 1.0);
// 	cv::Mat rotated_image;
	//FIXME: warp features (not image)
// 	cv::warpAffine(new_image, rotated_image, rotation_matrix, new_image.size());
	// shift image
// 	double m[2][3] = {{1, 0, -delta_x}, {0, 1, -delta_y}};
// 	cv::Mat transform_matrix(2, 3, CV_64F, m);
// 	cv::Mat transformed_image;
// 	cv::warpAffine(rotated_image, transformed_image, transform_matrix, rotated_image.size());

	// match descriptors
// 	std::vector<std::vector<cv::DMatch> > matches;
// 	std::vector<std::vector<cv::DMatch> > forward_matches;
// 	std::vector<std::vector<cv::DMatch> > backward_matches;
// 	matcher.radiusMatch(old_descriptors, new_descriptors, forward_matches, 100.0);	//0.21 for L2
// 	matcher.radiusMatch(new_descriptors, old_descriptors, backward_matches, 100.0);	//0.21 for L2
// 	fusion_matches(forward_matches, backward_matches, matches);
// 	matcher.radiusMatch(old_descriptors, new_descriptors, matches, 100.0);	//0.21 for L2
// 	if(matches.empty()) {
// 		Logger::log("no matches were found", Logger::LOGLEVEL_DEBUG, _loglevel);
// 		return;
// 	}

	//TODO: check for ambigous matches

	// TODO: use RANSAC instead of IMU filter?
// 	std::vector<uint8_t> filter;
// 	int valid_matches = filter_matches_by_imu< cv::L1<float> >(old_features,
// 		new_features,
// 		matches,
// 		center,
// 		attitude_change.roll, attitude_change.pitch, attitude_change.yaw,
// 		delta_x, delta_y,
// 		filter);
// 	float valid_rate = (float)valid_matches/filter.size();
// 	Logger::log(name(), ": Valid match rate is", valid_rate, filter.size(), Logger::LOGLEVEL_INFO, _loglevel);

// 	cv::Mat H = find_homography(old_features, new_features, matches, CV_RANSAC);
// 	cv::Mat H = find_homography(old_features, new_features, matches, 0);
// 	std::cout << H << std::endl;
// 	double yaw = acos( (H.at<double>(0,0) + H.at<double>(1,1))/2.0 );
// 	std::cout << "yaw = " << rad2deg(yaw) << " (" << H.at<double>(0,2) << ", " << H.at<double>(1,2) << std::endl;

// 	const float _yaw = yaw(landmarks.keypoints,
// 			       keypoints,
// 				matches,
// 				cam_matrix,
// 				matches_mask);
// 	log("yaw:", _yaw, Logger::LOGLEVEL_INFO);

	// get change of attitude
	vector<float> parameter_vector(6);
	parameter_vector[0] = attitudes.back().pitch - attitudes.front().pitch;	//phi
	parameter_vector[1] = attitudes.back().roll - attitudes.front().roll;	//theta
	parameter_vector[2] = attitudes.back().yaw - attitudes.front().yaw;	//psi
	parameter_vector[3] = 0;
	parameter_vector[4] = 0;
	parameter_vector[5] = 0;
	int rc = estimate_pose(landmarks.objectpoints,
		keypoints,
		matches,
		cam_matrix,
		dist_coeffs,
		parameter_vector,
		matches_mask);
	if(rc <= 0) {
		log("position estimation failed with return code", rc, Logger::LOGLEVEL_DEBUG);
		return;
	}
// 	cv::Mat rotation_vector(3, 1, cv::DataType<double>::type);
// 	rotation_vector.at<double>(0, 1) = attitudes.back().roll - attitudes.front().roll;
// 	rotation_vector.at<double>(0, 0) = attitudes.back().pitch - attitudes.front().pitch;
// 	rotation_vector.at<double>(0, 2) = attitudes.back().yaw - attitudes.front().yaw;
// 	cv::Mat translation_vector = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
// 	egomotion(landmarks.objectpoints,
// 		keypoints,
// 		matches,
// 		cam_matrix,
// 		dist_coeffs,
// 		rotation_vector,
// 		translation_vector,
// 		use_extrinsic_guess,
// 		matches_mask);
// 	if(rotation_vector.empty() || translation_vector.empty()) {
// 		log("determination of egomotion failed", Logger::LOGLEVEL_DEBUG);
// 		return;
// 	}
	stop_time = get_time_ms();
	log( "egomotion needed", stop_time-start_time, "ms", Logger::LOGLEVEL_DEBUG);
	start_time = stop_time;

	// add offsets to rotation vector to get orientation out of relative changes
// 	rotation_vector.at<double>(0, 1) += attitudes.front().roll;
// 	rotation_vector.at<double>(0, 0) += attitudes.front().pitch;
// 	rotation_vector.at<double>(0, 2) += attitudes.front().yaw;
	//FIXME: add offsets to translation to get position out of egomotion
// 	translation_vector.at<double>(0, 2) += altitude;

#ifdef SLAM_LOG
	log_file << setw(13) << get_time_ms()
		<< setw(10) << right << attitudes.back().roll
		<< setw(10) << right << attitudes.back().pitch
		<< setw(10) << right << attitudes.back().yaw
		<< setw(10) << right << attitudes.back().rollspeed
		<< setw(10) << right << attitudes.back().pitchspeed
		<< setw(10) << right << attitudes.back().yawspeed
		<< setw(10) << right << altitude
		<< setw(10) << right << parameter_vector[1]
		<< setw(10) << right << parameter_vector[0]
		<< setw(10) << right << parameter_vector[2]
		<< setw(10) << right << parameter_vector[4]
		<< setw(10) << right << parameter_vector[3]
		<< setw(10) << right << parameter_vector[5]
		<< std::endl;
#endif
// 	log("rotation vector", rotation_vector, Logger::LOGLEVEL_INFO);
// 	log("translation_vector", translation_vector, Logger::LOGLEVEL_INFO);
	log("parameter_vector_vector", parameter_vector, Logger::LOGLEVEL_INFO);
	{
	Lock tx_lock(tx_mav_mutex);
	mavlink_msg_attitude_pack(system_id(),
		component_id,
		&tx_mav_msg,
		get_time_us(),
		parameter_vector[1],
		parameter_vector[0],
		parameter_vector[2],
		0, 0, 0);
	AppLayer<mavlink_message_t>::send(tx_mav_msg);
	}

	if(with_out_stream && Core::video_server) {
		cv::Mat match_img;
		cv::drawMatches(scenes.front(), landmarks.keypoints,
			scenes.back(), keypoints,
			matches,
			match_img,
			cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
			matches_mask,
// 			std::vector<char>(),
// 			std::vector<std::vector<char> >(),
			cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
		);
		//FIXME:
		GstAppSrc *appsrc = GST_APP_SRC( Core::video_server->element("source", -1) );
		if(appsrc)
			Core::video_server->push(appsrc, match_img.data, match_img.cols, match_img.rows, 24);
		else
			log(name(), ": no appsrc found", Logger::LOGLEVEL_DEBUG);
	}
// 	uint64_t stop_time = get_time_ms();
	stop_time = get_time_ms();
	Logger::log(name(), "sending and displaying needed", stop_time-start_time, "ms", Logger::LOGLEVEL_DEBUG, _loglevel);
}

void SLAMApp::handle_input(const mavlink_message_t &msg) {
	Logger::log(name(), "got mavlink_message", static_cast<int>(msg.msgid),
		"from", static_cast<int>(msg.sysid),
		static_cast<int>(msg.compid),
		Logger::LOGLEVEL_DEBUG, _loglevel);

	switch(msg.msgid) {
		case MAVLINK_MSG_ID_ACTION:
			if( (mavlink_msg_action_get_target(&msg) == system_id()) ) {
// 			&& (mavlink_msg_action_get_target_component(&msg) == component_id) ) {
				uint8_t action_id = mavlink_msg_action_get_action(&msg);
				if(action_id == MAV_ACTION_GET_IMAGE) {
					Lock sync_lock(sync_mutex);
					// new image with ACK
					take_new_image = 3;
				}
			}
			break;
		case MAVLINK_MSG_ID_ATTITUDE:
			if( (msg.sysid == system_id()) ) {
				Lock sync_lock(sync_mutex);
				mavlink_msg_attitude_decode(&msg, &attitude);
				// take system time for attitude
				attitude.usec = get_time_us();
			}
			break;
		case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
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
	if(take_new_image) {
		// make a new reference image
		video_data.copyTo(scenes.front());
		landmarks.clear();
		memcpy(&(attitudes.front()), &attitude, sizeof(mavlink_attitude_t));
		if(take_new_image & (1 << 1)) {
			//TODO: send ACK
		}
		take_new_image = 0;
		log("took new reference image", Logger::LOGLEVEL_DEBUG);
#ifdef SLAM_LOG
		log_file << "# new reference image" << std::endl;
#endif

	} else {
		video_data.copyTo(scenes.back());
// 		new_features.clear();
		memcpy(&(attitudes.back()), &attitude, sizeof(mavlink_attitude_t));
// 		Logger::log(name(), "attitude of new image",
// 			rad2deg(new_attitude.roll),
// 			rad2deg(new_attitude.pitch),
// 			rad2deg(new_attitude.yaw),
// 			Logger::LOGLEVEL_DEBUG, _loglevel);
	}
	new_video_data = true;
}

void SLAMApp::load_calibration_data(const std::string &filename) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if( !fs.isOpened() ) {
		log("Can't open calibration data", filename, Logger::LOGLEVEL_DEBUG);
		return;
	}

	fs["camera_matrix"] >> cam_matrix;
	fs["distortion_coefficients"] >> dist_coeffs;
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
				extract_features();
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
