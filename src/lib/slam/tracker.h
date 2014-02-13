#ifndef _HUB_TRACKER_H_
#define _HUB_TRACKER_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_OPENCV2
#include <opencv2/opencv.hpp>
#include <opencv2/legacy/legacy.hpp>
#if CV_MINOR_VERSION >= 2

#include <brisk/brisk.h>
#include "lib/slam/features.h"
#include "lib/slam/map.h"

namespace hub {
namespace slam {

//FIXME: make it more generic template class
class Tracker {
public:
	/**
	 * \brief Constructor
	 */
	Tracker(const int image_width,
		const int image_height,
		const cv::Mat &camera_matrix = cv::Mat(),
		const cv::Mat &distortion_coefficients = cv::Mat());
	~Tracker();

	/**
	 * \brief Save map points (point cloud) and camera views as polygon files (ply).
	 */
	int save_map(const std::string &name) const;

	/**
	 * \brief Get pose of camera by identifying the image in the map.
	 * \param[in] image Camera image.
	 * \param[in,out] parameter_vector 6D-Vector containing quaternion vector part and translation. Input will be used as a first guess.
	 * \param[in] avg_depth Average distance between camera and objects.
	 */
	int track_camera(const cv::Mat &image, std::vector<float> &parameter_vector, const float avg_depth = 100.0);

protected:

private:
	cv::Mat camera_matrix; ///< Camera matrix of intrinsic parameters.
	cv::Mat distortion_coefficients; ///< distortion coefficients of camera.

	cv::BriskFeatureDetector feature_detector; ///< BRISK feature detector using AGAST
	cv::BriskDescriptorExtractor descriptor_extractor;
#ifdef HAVE_SSSE3
	// see slam_app.h
	// cv::BruteForceMatcher<cv::HammingSse> matcher;
	cv::BruteForceMatcher<cv::Hamming> matcher;
#else
	cv::BruteForceMatcher<cv::Hamming> matcher;
#endif
	Map<> map;

};

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------
inline int Tracker::save_map(const std::string &name) const {
	map.save_points(name+"_map.ply");
	return map.save_views(name+"_view.ply");
}

} // namespace slam
} // namespace hub

#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // _HUB_TRACKER_H_
