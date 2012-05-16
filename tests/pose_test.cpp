#include "lib/slam/pose.h"

#include <boost/test/unit_test.hpp>
#include <cstdlib>

using namespace hub::slam;

BOOST_AUTO_TEST_SUITE(PoseTestSuite)

BOOST_AUTO_TEST_CASE(Test_estimate_pose)
{
	std::vector<cv::Point3f> object_points;
	object_points.push_back( cv::Point3f(0, 0, 10) );
	object_points.push_back( cv::Point3f(-20, -15, 20) );
	object_points.push_back( cv::Point3f(-25, 35, 30) );
	object_points.push_back( cv::Point3f(22, 17, 40) );
	object_points.push_back( cv::Point3f(10, -20, 50) );
	object_points.push_back( cv::Point3f(-15, 6, 60) );
	object_points.push_back( cv::Point3f(5, -28, 80) );

	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 288.0,   0.0, 160.0,
	                                                    0.0, 284.0, 120.0,
	                                                    0.0,   0.0,   1.0);
	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0912, -0.287, 0.001889, 0.001793, 0.09828);

	std::vector<float> rotation_vector(3);
	rotation_vector[0] = 0.1;	//phi
	rotation_vector[1] = 0.2;	//theta
	rotation_vector[2] = 0.3;	//psi
	std::vector<float> translation_vector(3);
	translation_vector[3] = 1.0;	//t1
	translation_vector[4] = -1.0;	//t2
	translation_vector[5] = 0.5;	//t3

	std::vector<cv::Point2f> image_points;
	projectPoints(object_points,
		rotation_vector,
		translation_vector,
		camera_matrix,
		distortion_coefficients,
		image_points);

	std::vector<cv::KeyPoint> keypoints;
	for(std::vector<cv::Point2f>::iterator iter = image_points.begin(); iter != image_points.end(); ++iter) {
		keypoints.push_back( cv::KeyPoint(*iter, 1.0) );
	}

	std::vector<cv::DMatch> matches;
	for(int index = 0; index < object_points.size(); index++) {
		matches.push_back( cv::DMatch(index, index, 0) );
	}

	std::vector<char> matches_mask;

	std::vector<float> parameter_vector(6, 0);

	int rc = 1;
// 	int rc = estimate_pose(object_points,
// 		keypoints,
// 		matches,
// 		camera_matrix,
// 		distortion_coefficients,
// 		matches_mask,
// 		parameter_vector);
	BOOST_CHECK(rc > 0);
}

BOOST_AUTO_TEST_SUITE_END()
