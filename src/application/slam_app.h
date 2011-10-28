#ifndef _SLAM_APP_H_
#define _SLAM_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER
#include "lib/gstreamer/video_server.h"
#include "lib/gstreamer/video_client.h"

#ifdef HAVE_OPENCV_CV_H
#include <brisk/brisk.h>

#include "protocol/protocollayer.h"

#include <inttypes.h> //uint8_t

namespace mavhub {

	class SLAMApp : public AppLayer<mavlink_message_t>
		, public hub::gstreamer::VideoClient
	{
		public:
			static const int component_id = 28;

			SLAMApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
			virtual ~SLAMApp();

			virtual void handle_input(const mavlink_message_t &msg);
			virtual void handle_video_data(const unsigned char *data, const int width, const int height, const int bpp);

		protected:
			virtual void print(std::ostream &os) const;
			virtual void run();

		private:
			bool with_out_stream;
			uint8_t take_new_image;
			bool new_video_data;
			/// tx buffer for mavlink messages
			mavlink_message_t tx_mav_msg;
			/// Mutex to protect tx_mav_msg
			pthread_mutex_t tx_mav_mutex;
			/// Mutex to sync between application thread and input calls
			pthread_mutex_t sync_mutex;

			//FIXME: replace old_* by database of these informations
			std::string sink_name;
			cv::Mat old_image;
			cv::Mat new_image;
			std::vector<cv::KeyPoint> old_features;
			std::vector<cv::KeyPoint> new_features;
			cv::BriskFeatureDetector feature_detector;
			cv::Mat old_descriptors;
			cv::Mat new_descriptors;
			cv::BriskDescriptorExtractor descriptor_extractor;
#ifdef HAVE_SSSE3
			cv::BruteForceMatcher<cv::HammingSse> matcher;
#else
			cv::BruteForceMatcher<cv::Hamming> matcher;
#endif
			void extract_features();
	};

} // namespace mavhub

#endif // HAVE_OPENCV_CV_H

#endif // HAVE_GSTREAMER

#endif // HAVE_MAVLINK_H

#endif // _OPENGL_APP_H_
