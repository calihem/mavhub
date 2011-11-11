#include "slam_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER

#ifdef HAVE_OPENCV_CV_H

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

#include "lib/slam/features.h"

#include <sstream> //istringstream

using namespace cpp_pthread;

namespace mavhub {

SLAMApp::SLAMApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel) :
	AppInterface("slam_app", loglevel),
	MavlinkAppLayer("slam_app", loglevel),
	hub::gstreamer::VideoClient(),
	with_out_stream(false),
	take_new_image(1),
	target_system(7),
	target_component(1),
	imu_rate(10),
	feature_detector(60, 3) //threshold, octaves
	{

	pthread_mutex_init(&sync_mutex, NULL);
	// invalidate attitude
	attitude.usec = 0;

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

	assign_variable_from_args(target_system);
	assign_variable_from_args(target_component);
	assign_variable_from_args(imu_rate);
}

SLAMApp::~SLAMApp() {}

void SLAMApp::extract_features() {
	//FIXME: remove benchmark
	uint64_t start_time = get_time_ms();

	//FIXME: remove doubled code
	if( old_features.empty() ) {
		feature_detector.detect(old_image, old_features);
		Logger::log(name(), ": found", old_features.size(), "(old) features", Logger::LOGLEVEL_DEBUG, _loglevel);
		
		if( old_features.empty() ) {
			log(name(), ": didn't found features in snapshot image", Logger::LOGLEVEL_WARN);
			return;
		}

		//TODO: filter features (shi-tomasi)

		descriptor_extractor.compute(old_image, old_features, old_descriptors);
		if( new_descriptors.empty() )
			return;
	} else {
		feature_detector.detect(new_image, new_features);
		Logger::log(name(), ": found", new_features.size(), "(new) features", Logger::LOGLEVEL_DEBUG, _loglevel);

		if( new_features.empty() ) {
			log(name(), ": didn't found features in current image", Logger::LOGLEVEL_WARN);
			return;
		}

		//TODO: filter features (shi-tomasi)

		descriptor_extractor.compute(new_image, new_features, new_descriptors);
	}

	// get change of attitude
	//TODO: time check of attitude
	mavlink_attitude_t attitude_change;
	attitude_change.roll = new_attitude.roll - old_attitude.roll;
	attitude_change.pitch = new_attitude.pitch - old_attitude.pitch;
	attitude_change.yaw = new_attitude.yaw - old_attitude.yaw;
	Logger::log(name(), "Changed attitude:",
		rad2deg(attitude_change.roll),
		rad2deg(attitude_change.pitch),
		rad2deg(attitude_change.yaw),
		Logger::LOGLEVEL_DEBUG, _loglevel);
	
	// calculate transformation matrix
	double distance = 1.0; //FIXME: use altitude information
	double factor = 300.0;
	double delta_x = factor*2.0*distance*sin(attitude_change.roll/2);
	double delta_y = factor*2.0*distance*sin(attitude_change.pitch/2);
	// rotate image
	cv::Point center(new_image.cols/2, new_image.rows/2);
	cv::Mat rotation_matrix = getRotationMatrix2D(center, -rad2deg(attitude_change.yaw), 1.0);
	cv::Mat rotated_image;
	//FIXME: warp features (not image)
	cv::warpAffine(new_image, rotated_image, rotation_matrix, new_image.size());
	// shift image
	double m[2][3] = {{1, 0, -delta_x}, {0, 1, -delta_y}};
	cv::Mat transform_matrix(2, 3, CV_64F, m);
	cv::Mat transformed_image;
	cv::warpAffine(rotated_image, transformed_image, transform_matrix, rotated_image.size());

	// match descriptors
	std::vector<std::vector<cv::DMatch> > matches;
	matcher.radiusMatch(old_descriptors, new_descriptors, matches, 100.0);	//0.21 for L2

	//FIXME:
	new_features.clear();
	matches.clear();

	//TODO: check for ambigous matches

	if(with_out_stream && Core::video_server) {
		cv::Mat match_img;
		cv::drawMatches(old_image, old_features,
			transformed_image, new_features,
			matches,
			match_img,
			cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
			std::vector<std::vector<char> >(),
			cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
		);
		//FIXME:
		GstAppSrc *appsrc = GST_APP_SRC( Core::video_server->element("source", -1) );
		if(appsrc)
			Core::video_server->push(appsrc, match_img.data, match_img.cols, match_img.rows, 24);
		else
			log(name(), ": no appsrc found", Logger::LOGLEVEL_DEBUG);
	}
	uint64_t stop_time = get_time_ms();
	Logger::log(name(), ": needed", stop_time-start_time, "ms", Logger::LOGLEVEL_DEBUG, _loglevel);
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

	// make a matrix header for captured data
	unsigned char *image_data = const_cast<unsigned char*>(data);
	cv::Mat video_data(height, width, CV_8UC1, image_data);

	Lock sync_lock(sync_mutex);
	if(take_new_image) {
		// make a new reference image
		video_data.copyTo(old_image);
		old_features.clear();
		memcpy(&old_attitude, &attitude, sizeof(mavlink_attitude_t));
		if(take_new_image & (1 << 1)) {
			//TODO: send ACK
		}
		take_new_image = 0;
		log(name(), ": took new image", Logger::LOGLEVEL_DEBUG);
	} else {
		video_data.copyTo(new_image);
		new_features.clear();
		memcpy(&new_attitude, &attitude, sizeof(mavlink_attitude_t));
		Logger::log(name(), "attitude of new image",
			rad2deg(new_attitude.roll),
			rad2deg(new_attitude.pitch),
			rad2deg(new_attitude.yaw),
			Logger::LOGLEVEL_DEBUG, _loglevel);
	}
	new_video_data = true;
}

void SLAMApp::print(std::ostream &os) const {
	AppLayer<mavlink_message_t>::print(os);
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

	Logger::log(name(), ": request data stream extra1 from",
		target_system, ":", target_component,
		Logger::LOGLEVEL_DEBUG, _loglevel);
	request_data_stream(target_system,
		target_component,
		MAV_DATA_STREAM_EXTRA1,
		imu_rate);

	while( !interrupted() ) {
		log(name(), "enter main loop", Logger::LOGLEVEL_DEBUG);
		{ Lock sync_lock(sync_mutex);
			if(new_video_data)
				extract_features();
			new_video_data = false;
		}
		//FIXME: remove usleep
		usleep(5000);
	}

	//unbind from video server
	Core::video_server->release( dynamic_cast<VideoClient*>(this) );
	log(name(), ": stop running", Logger::LOGLEVEL_DEBUG);
}

} // namespace mavhub

#endif // HAVE_OPENCV_CV_H

#endif // HAVE_GSTREAMER

#endif // HAVE_MAVLINK_H
