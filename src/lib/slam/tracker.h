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

class Tracker {
public:
	Tracker();

	void track_camera(const cv::Mat &image, std::vector<float> &parameter_vector);

protected:

private:
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
	

} // namespace slam
} // namespace hub

#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // _HUB_TRACKER_H_
