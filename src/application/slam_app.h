#ifndef _SLAM_APP_H_
#define _SLAM_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER
#include "lib/gstreamer/video_server.h"
#include "lib/gstreamer/video_client.h"

#ifdef HAVE_OPENCV2
#include <opencv/cv.h>

#if CV_MINOR_VERSION >= 2
#include <brisk/brisk.h>
#include "lib/slam/features.h"
#include "protocol/protocollayer.h"

#include <inttypes.h> //uint8_t

namespace mavhub {

class SLAMApp : public MavlinkAppLayer,
	public hub::gstreamer::VideoClient {

	public:
		SLAMApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
		virtual ~SLAMApp();

		virtual void handle_input(const mavlink_message_t &msg);
		virtual void handle_video_data(const unsigned char *data, const int width, const int height, const int bpp);

	protected:
		virtual void run();

	private:
		bool with_out_stream;
		uint8_t take_new_image;
		bool new_video_data;
		/// Mutex to sync between application thread and input calls
		pthread_mutex_t sync_mutex;
		unsigned int target_system;
		unsigned int target_component;
		unsigned int imu_rate;
		/// Current attitude of system
		mavlink_attitude_t attitude;

		//FIXME: replace old_* by database of these informations
		std::string sink_name;
		cv::Mat cam_matrix;
		cv::Mat dist_coeffs;
		cv::Mat old_image;
		cv::Mat new_image;
// 		std::list<hub::slam::brisk_landmark_t> landmarks;
		hub::slam::landmarks_t landmarks;
		/// List of images
		std::list<cv::Mat> scenes;
		mavlink_attitude_t old_attitude;
		mavlink_attitude_t new_attitude;
		std::vector<cv::KeyPoint> old_features;
		std::vector<cv::KeyPoint> new_features;
		std::vector<cv::Point3f> old_object_points;
		cv::BriskFeatureDetector feature_detector;
		cv::Mat old_descriptors;
		cv::Mat new_descriptors;
		cv::BriskDescriptorExtractor descriptor_extractor;
#ifdef HAVE_SSSE3
		cv::BruteForceMatcher<cv::HammingSse> matcher;
#else
		cv::BruteForceMatcher<cv::Hamming> matcher;
#endif
		cv::Mat rotation_vector;
		cv::Mat translation_vector;

		void extract_features();
		void load_calibration_data(const std::string &filename);
};

} // namespace mavhub

#endif // CV_MINOR_VERSION >= 2
#endif // HAVE_OPENCV2
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
#endif // _OPENGL_APP_H_
