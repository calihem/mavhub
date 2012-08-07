#ifndef _FIDUCAL_APP_H_
#define _FIDUCAL_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GSTREAMER
#include "lib/gstreamer/video_server.h"
#include "lib/gstreamer/video_client.h"

#ifdef HAVE_OPENCV2
#include <opencv/cv.h>

#include <inttypes.h> //uint8_t

#define FIDUCAL_LOG 1

namespace mavhub {

class FiducalApp : public MavlinkAppLayer,
	public hub::gstreamer::VideoClient {

	public:
		FiducalApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
		virtual ~FiducalApp();

		virtual void handle_input(const mavlink_message_t &msg);
		virtual void handle_video_data(const unsigned char *data, const int width, const int height, const int bpp);

	protected:
		virtual void run();

	private:
		pthread_mutex_t sync_mutex; ///< Mutex to sync between application thread and input calls
		std::string sink_name; ///< Sink name of video server input.
		cv::Mat cam_matrix; ///< Camera matrix of intrinsic parameters.
		cv::Mat dist_coeffs; ///< distortion coefficients of camera.
    bool new_video_data;
#ifdef _LOG
		std::ofstream log_file;
#endif
};

} // namespace mavhub

#endif // HAVE_OPENCV2
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
#endif // _FIDUCAL_APP_H_
