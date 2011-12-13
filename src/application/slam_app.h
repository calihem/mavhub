#ifndef _SLAM_APP_H_
#define _SLAM_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER
#include "lib/gstreamer/video_server.h"
#include "lib/gstreamer/video_client.h"

#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION > 1)
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
			//FIXME: replace old_* by database of these informations
			std::string sink_name;
			std::vector<cv::KeyPoint> old_features;
			std::vector<cv::KeyPoint> new_features;
			cv::BriskFeatureDetector feature_detector;
			cv::Mat old_descriptors;
			cv::Mat new_descriptors;
			cv::BriskDescriptorExtractor descriptor_extractor;
			cv::Mat old_image;
#ifdef HAVE_SSSE3
			cv::BruteForceMatcher<cv::HammingSse> matcher;
#else
			cv::BruteForceMatcher<cv::Hamming> matcher;
#endif
	};

} // namespace mavhub

#endif // HAVE_OPENCV2 && CV_MINOR_VERSION > 1

#endif // HAVE_GSTREAMER

#endif // HAVE_MAVLINK_H

#endif // _OPENGL_APP_H_
