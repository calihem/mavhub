#include "tracker.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

#define VERBOSE 1

// little debug makro
#if VERBOSE >= 1
#define dout if(1) std::cout << typeid(*this).name() << ": "
#else
#define dout if(0) std::cout
#endif

namespace hub {
namespace slam {

Tracker::Tracker() :
	feature_detector(60, 3), //threshold, octaves
	descriptor_extractor(),
	matcher() {
		
}

void Tracker::track_camera(const cv::Mat &image, std::vector<float> &parameter_vector) {

	// get features from image
	std::vector<cv::KeyPoint> keypoints;
	feature_detector.detect(image, keypoints);
	if(keypoints.empty()) {
		dout << "didn't found reference features" << std::endl;
		return;
	}

	// calculate descriptors which can remove or add some of the keypoints
	cv::Mat descriptors;
	descriptor_extractor.compute(image, keypoints, descriptors);
	if(keypoints.empty()) {
		dout << "keypoints empty after descriptor computation" << std::endl;
		return;
	}

	//TODO: get possible descriptors from map
// 	std::vector<char> mask;
// 	map.visible_points(parameter_vector,
// 		camera_matrix,
// 		width,
// 		height,
// 		mask);

#if 0
	// match descriptors
	std::vector<cv::DMatch> matches;
	matcher.match(landmarks.descriptors, descriptors, matches);
	if(matches.empty()) {
		dout << "no matches fount" << std::endl;
		return;
	}
#endif
}

} // namespace slam
} // namespace hub

#endif // defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2
