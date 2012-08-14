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

#include <vector>
#include "protocol/protocollayer.h"

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
    cv::Mat video_data;
    bool new_video_data;
    double resizeFactor;
    double outer_tag_size;
    double inner_tag_size;
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat fvec;
    void load_calibration_data(const std::string &filename);
    cv::Point calcCentroid(std::vector<cv::Point> points);
    void findTwoRectangles(const cv::Mat &grayscale, std::vector< std::vector<cv::Point> > &markers);
    void refineMarkers(const cv::Mat &grayscale, std::vector<std::vector<cv::Point> > &markers, std::vector<cv::Point2f> &outerMarkers, std::vector<cv::Point2f> &innerMarkers);
    void orderMarkers(const cv::Mat &grayscale, std::vector<cv::Point2f> &outerMarkers, std::vector<cv::Point2f> &innerMarkers);
    void doGeometry(const std::vector<cv::Point2f> &outerMarkers, const std::vector<cv::Point2f> &innerMarkers, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, cv::Mat &rvec, cv::Mat &tvec);
#ifdef FIDUCAL_LOG
		std::ofstream log_file;
#endif
};

} // namespace mavhub

#endif // HAVE_OPENCV2
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
#endif // _FIDUCAL_APP_H_
