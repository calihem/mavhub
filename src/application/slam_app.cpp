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
	AppLayer<mavlink_message_t>("slam_app", loglevel),
	hub::gstreamer::VideoClient(),
	with_out_stream(false),
	take_new_image(1),
	feature_detector(60, 3) //threshold, octaves
	{

	pthread_mutex_init(&tx_mav_mutex, NULL);
	pthread_mutex_init(&sync_mutex, NULL);

	// set sink name
	std::map<std::string,std::string>::const_iterator iter = args.find("sink");
	if( iter != args.end() ) {
		sink_name.assign(iter->second);
	} else {
		log("SLAMApp: sink argument missing", Logger::LOGLEVEL_DEBUG);
		sink_name.assign("sink0");
	}

	iter = args.find("out_stream");
	if( iter != args.end() ) {
		std::istringstream istream(iter->second);
		istream >> with_out_stream;
	} else {
		log("SLAMApp: out_stream argument missing", Logger::LOGLEVEL_DEBUG);
	}

	//TODO: pipe_in, pipe_out
}

SLAMApp::~SLAMApp() {}

void SLAMApp::extract_features() {
	//FIXME: remove benchmark
	uint64_t start_time = get_time_ms();

	//FIXME: remove doubled code
	if( old_features.empty() ) {
		feature_detector.detect(old_image, old_features);
		log("SLAMApp: found", old_features.size(), "(old) features", Logger::LOGLEVEL_DEBUG);
		
		if( old_features.empty() ) {
			log("SLAMApp: didn't found features in snapshot image", Logger::LOGLEVEL_WARN);
			return;
		}

		//TODO: filter features (shi-tomasi)

		descriptor_extractor.compute(old_image, old_features, old_descriptors);
		if( new_descriptors.empty() )
			return;
	} else {
		feature_detector.detect(new_image, new_features);
		log("SLAMApp: found", new_features.size(), "(new) features", Logger::LOGLEVEL_DEBUG);
		
		if( new_features.empty() ) {
			log("SLAMApp: didn't found features in current image", Logger::LOGLEVEL_WARN);
			return;
		}

		//TODO: filter features (shi-tomasi)

		descriptor_extractor.compute(new_image, new_features, new_descriptors);
	}

	// match descriptors
	std::vector<std::vector<cv::DMatch> > matches;
	matcher.radiusMatch(old_descriptors, new_descriptors, matches, 100.0);	//0.21 for L2

	//TODO: check for ambigous matches

	if(with_out_stream && Core::video_server) {
		cv::Mat match_img;
		cv::drawMatches(old_image, old_features,
			new_image, new_features,
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
			log("SLAMApp: no appsrc found", Logger::LOGLEVEL_DEBUG);
	}
	uint64_t stop_time = get_time_ms();
	log("SLAMApp: needed", stop_time-start_time, "ms", Logger::LOGLEVEL_DEBUG);
}

void SLAMApp::handle_input(const mavlink_message_t &msg) {
// 	log("SLAMApp got mavlink_message", static_cast<int>(msg.msgid), Logger::LOGLEVEL_DEBUG);
	//FIXME
	Logger::log("SLAMApp got mavlink_message", static_cast<int>(msg.msgid),
		"for target", static_cast<int>(mavlink_msg_action_get_target(&msg)),
		"and component", static_cast<int>(mavlink_msg_action_get_target_component(&msg)),
		Logger::LOGLEVEL_WARN, _loglevel);

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
		default: break;
	}
}


void SLAMApp::handle_video_data(const unsigned char *data, const int width, const int height, const int bpp) {
	if(!data) return;

	Logger::log("SLAMApp: got new video data of size", width, "x", height, Logger::LOGLEVEL_DEBUG, _loglevel);
	if(bpp != 8) {
		log("SLAMApp: unsupported video data with bpp =", bpp, Logger::LOGLEVEL_WARN);
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
// 		old_descriptors.clear();
		if(take_new_image & (1 << 1)) {
			//TODO: send ACK
		}
		take_new_image = 0;
		//FIXME
		log("SLAMApp took new image", Logger::LOGLEVEL_WARN);
	} else {
		video_data.copyTo(new_image);
		new_features.clear();
// 		new_descriptors.clear();
	}
	new_video_data = true;
}

void SLAMApp::print(std::ostream &os) const {
	AppLayer<mavlink_message_t>::print(os);
}

void SLAMApp::run() {
	log("SLAMApp running", Logger::LOGLEVEL_DEBUG);


	if(Core::video_server) {
		int rc = Core::video_server->bind2appsink( dynamic_cast<VideoClient*>(this), sink_name.c_str());
		log("SLAMApp binded to", sink_name, rc, Logger::LOGLEVEL_DEBUG);
	} else {
		log("video server not running", Logger::LOGLEVEL_WARN);
		return;
	}

	while( !interrupted() ) {
		Lock sync_lock(sync_mutex);
		if(new_video_data)
			extract_features();
		new_video_data = false;
		//FIXME: remove usleep
		usleep(50);
	}

	//unbind from video server
	Core::video_server->release( dynamic_cast<VideoClient*>(this) );
	log("SLAMApp stop running", Logger::LOGLEVEL_DEBUG);
}

} // namespace mavhub

#endif // HAVE_OPENCV_CV_H

#endif // HAVE_GSTREAMER

#endif // HAVE_MAVLINK_H
