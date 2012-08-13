#ifndef _FIDUCAL_CONTROL_APP_H_
#define _FIDUCAL_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_OPENCV2
#include <opencv/cv.h>

#include <vector>

#define FIDUCAL_LOG 1

namespace mavhub {

class FiducalControlApp : public MavlinkAppLayer{

	public:
		FiducalControlApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
		virtual ~FiducalApp();

		virtual void handle_input(const mavlink_message_t &msg);
		virtual void handle_video_data(const unsigned char *data, const int width, const int height, const int bpp);

	protected:
		virtual void run();

	private:
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat fvec;
    cv::Mat setpoint;
#ifdef _LOG
		std::ofstream log_file;
#endif
};

} // namespace mavhub

#endif // HAVE_OPENCV2
#endif // HAVE_MAVLINK_H
#endif // _FIDUCAL_APP_CONTROL_H_
