#include "pose.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

#include <iostream>     // cout
#include <iomanip>	//setprecision
#include <cmath>	//sin, cos
#include <limits>	//epsilon

#include <levmar/levmar.h>

#define VERBOSE 1

// little debug makro
#if VERBOSE >= 1
#define dout if(1) std::cout
#else
#define dout if(0) std::cout
#endif

namespace hub {
namespace slam {

using namespace std;
using namespace TooN;

int estimate_pose(const std::vector<cv::Point3f> &object_points,
	const std::vector<cv::KeyPoint>& dst_keypoints,
	const std::vector<cv::DMatch>& matches,
	const cv::Mat &camera_matrix,
	const cv::Mat &distortion_coefficients,
	const std::vector<char> &matches_mask,
	std::vector<float> &parameter_vector) {

	if(parameter_vector.size() != 6)
		return -1;

	unsigned int num_matches; //number of valid matches
	if(matches_mask.empty()) { //empty mask => all matches are valid
		num_matches = matches.size();
	} else if(matches_mask.size() != matches.size()) { //mask doesn't fit the matches
		return -2;
	} else { //count number of valid matches
		num_matches = 0;
		for(size_t i = 0; i < matches.size(); i++) {
			if( matches_mask[i] == 0) continue;
			num_matches++;
		}
	}

	vector<float> matched_object_points(3*num_matches); // allocate matrix 3 x num_matches
// 	vector<float> matched_image_points(2*num_matches); // allocate matrix 2 x num_matches
	std::vector<cv::Point2f> matched_image_points(num_matches);
	
	unsigned int next_point_index = 0;
	for(size_t i = 0; i < matches.size(); i++) {
		if( !matches_mask.empty() && matches_mask[i] == 0) continue;

		const int src_index = matches[i].queryIdx;
		matched_object_points[next_point_index] = object_points[src_index].x;
		matched_object_points[next_point_index+num_matches] = object_points[src_index].y;
		matched_object_points[next_point_index+2*num_matches] = object_points[src_index].z;

		const int dst_index = matches[i].trainIdx;
		const cv::KeyPoint& dst_keypoint = dst_keypoints[dst_index];
// 		matched_image_points[next_point_index] = dst_keypoint.pt.x;
// 		matched_image_points[next_point_index+num_matches] = dst_keypoint.pt.y;
		matched_image_points[i].x = dst_keypoint.pt.x;
		matched_image_points[i].y = dst_keypoint.pt.y;

		next_point_index++;
	}

	std::vector<cv::Point2f> undistorted_points(num_matches);
	undistortPoints(matched_image_points, undistorted_points, camera_matrix, distortion_coefficients);

	pinhole_model_data_t<float> pinhole_model_data(matched_object_points,
		matched_image_points,
		camera_matrix);

//	float info[LM_INFO_SZ];

	int rc = slevmar_der(pinhole_model<float>,
		jac_pinhole_model<float>,
		&(parameter_vector[0]),	//parameter vector
		&(matched_image_points[0].x),	//measurement vector
		6,	//parameter vector dimension
		2*num_matches,	//measurement vector dimension
		100,	//max. number of iterations
		NULL,	//opts
		NULL,	//info
		NULL,	//work
		NULL,	//covar
		(void*)&pinhole_model_data);	//data

	return rc;
}

} // namespace slam
} // namespace hub

#endif // defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2
