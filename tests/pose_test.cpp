#include "lib/slam/pose.h"

#include <boost/test/unit_test.hpp>
#include <cstdlib>

// #define PRECISION float
#define PRECISION double

using namespace hub::slam;

BOOST_AUTO_TEST_SUITE(PoseTestSuite)

BOOST_AUTO_TEST_CASE(Test_estimate_pose)
{

	std::vector< cv::Point3_<PRECISION> > object_points;
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
// 	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0912, -0.287, 0.001889, 0.001793, 0.09828);
	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);

	std::vector<PRECISION> rotation_vector(3);
// 	rotation_vector[0] = 0.0;	//phi
// 	rotation_vector[1] = 0.0;	//theta
// 	rotation_vector[2] = 0.0;	//psi
	rotation_vector[0] = 0.1;	//phi
	rotation_vector[1] = 0.2;	//theta
	rotation_vector[2] = 0.3;	//psi
	std::vector<PRECISION> translation_vector(3);
// 	translation_vector[0] = 1.0;	//t1
// 	translation_vector[1] = 0.0;	//t2
// 	translation_vector[2] = 0.0;	//t3
	translation_vector[0] = 1.0;	//t1
	translation_vector[1] = -1.0;	//t2
	translation_vector[2] = 0.5;	//t3

	std::vector< cv::Point_<PRECISION> > image_points;
	projectPoints(object_points,
		rotation_vector,
		translation_vector,
		camera_matrix,
		distortion_coefficients,
		image_points);

// std::cout << "_Projected points_" << std::endl;
	std::vector<cv::KeyPoint> keypoints;
	for(std::vector< cv::Point_<PRECISION> >::iterator iter = image_points.begin(); iter != image_points.end(); ++iter) {
		keypoints.push_back( cv::KeyPoint(*iter, 1.0) );
// std::cout << "(" << iter->x << ", " << iter->y << ")" << std::endl;
	}
	//apply some error to image points
	keypoints[0].pt.x = 0;

	std::vector<cv::DMatch> matches;
	for(unsigned int index = 0; index < object_points.size(); index++) {
		matches.push_back( cv::DMatch(index, index, 0) );
	}

	std::vector<char> matches_mask;

	std::vector<PRECISION> parameter_vector(6, 0);

	int rc = estimate_pose<PRECISION>(object_points,
		keypoints,
		matches,
		camera_matrix,
		distortion_coefficients,
		matches_mask,
		parameter_vector);
	BOOST_CHECK(rc >= 0);

	std::cout << "_Pose Estimation_ (" << rc << ")" << std::endl;
	std::cout << "phi:   " << parameter_vector[0] << " (" << rotation_vector[0] << ")" << std::endl;
	std::cout << "theta: " << parameter_vector[1] << " (" << rotation_vector[1] << ")" << std::endl;
	std::cout << "psi:   " << parameter_vector[2] << " (" << rotation_vector[2] << ")" << std::endl;
	std::cout << "dx:    " << parameter_vector[3] << " (" << translation_vector[0] << ")" << std::endl;
	std::cout << "dy:    " << parameter_vector[4] << " (" << translation_vector[1] << ")" << std::endl;
	std::cout << "dz:    " << parameter_vector[5] << " (" << translation_vector[2] << ")" << std::endl;

	
}

BOOST_AUTO_TEST_SUITE_END()
