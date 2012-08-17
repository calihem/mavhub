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

#define SLAM_LOG 1

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
		bool use_extrinsic_guess;
		uint8_t take_new_image;
		bool new_video_data;
		pthread_mutex_t sync_mutex; ///< Mutex to sync between application thread and input calls
		unsigned int target_system;
		unsigned int target_component;
		unsigned int imu_rate; ///< Update rate of IMU sensor values.
		unsigned int channel_rate; ///< Update rate of raw RC channel stream.
		uint16_t trigger_channel; ///< Current raw value of trigger channel (channel 6).
		mavlink_attitude_t attitude; ///< Current attitude of system.
		float altitude; ///< Current altitude in cm.

		std::string sink_name; ///< Sink name of video server input.
		cv::Mat cam_matrix; ///< Camera matrix of intrinsic parameters.
		cv::Mat dist_coeffs; ///< distortion coefficients of camera.
		hub::slam::landmarks_t landmarks; ///< landmark database
		std::list<cv::Mat> scenes; ///< List of images
		std::list<mavlink_attitude_t> attitudes; ///< Attitude of scenes
		cv::BriskFeatureDetector feature_detector; ///< BRISK feature detector using AGAST
		cv::BriskDescriptorExtractor descriptor_extractor;
#ifdef HAVE_SSSE3
		cv::BruteForceMatcher<cv::HammingSse> matcher;
#else
		cv::BruteForceMatcher<cv::Hamming> matcher;
#endif
// 		cv::Mat rotation_vector;
// 		cv::Mat translation_vector;
#ifdef SLAM_LOG
		std::ofstream log_file;
#endif
		void extract_features();
		void load_calibration_data(const std::string &filename);
};

} // namespace mavhub

#endif // CV_MINOR_VERSION >= 2
#endif // HAVE_OPENCV2
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
#endif // _SLAM_APP_H_
