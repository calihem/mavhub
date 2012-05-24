#include "lib/slam/features.h"

#include <boost/test/unit_test.hpp>
#include <cstdlib>
#include <limits>	//epsilon

#define PRECISION float
// #define PRECISION double

static const PRECISION DISTANCE = 50;

using namespace hub::slam;

// template<typename PointT, typename T>
// T ssd(const PointT &point_a, const PointT &point_b);

template<typename T>
inline T ssd(const cv::Point_<T> &point_a, const cv::Point_<T> &point_b) {
	T sq_diff = (point_a.x-point_b.x)*(point_a.x-point_b.x);
	sq_diff += (point_a.y-point_b.y)*(point_a.y-point_b.y);

	return sq_diff;
}

template<typename T>
inline T ssd(const cv::Point3_<T> &point_a, const cv::Point3_<T> &point_b) {
	T sq_diff = (point_a.x-point_b.x)*(point_a.x-point_b.x);
	sq_diff += (point_a.y-point_b.y)*(point_a.y-point_b.y);
	sq_diff += (point_a.z-point_b.z)*(point_a.z-point_b.z);

	return sq_diff;
}

template<typename PointT, typename T>
T error(const std::vector< PointT >& points_a,
	const std::vector< PointT >& points_b) {
	
	if( points_a.size() != points_b.size() )
		return -1;
	
	T rc = 0;
	for(size_t i=0; i<points_a.size(); i++) {
		rc += sqrt( ssd<T>(points_a[i], points_b[i]) );
	}

	return rc;
}

BOOST_AUTO_TEST_SUITE(FeatureTestSuite)

BOOST_AUTO_TEST_CASE(Test_features)
{

	std::vector< cv::Point3_<PRECISION> > object_points;
	object_points.push_back( cv::Point3f(0, 0, DISTANCE) );
	object_points.push_back( cv::Point3f(-20, -15, DISTANCE) );
	object_points.push_back( cv::Point3f(-25, 35, DISTANCE) );
	object_points.push_back( cv::Point3f(22, 17, DISTANCE) );
	object_points.push_back( cv::Point3f(10, -20, DISTANCE) );
	object_points.push_back( cv::Point3f(-15, 6, DISTANCE) );
	object_points.push_back( cv::Point3f(5, -28, DISTANCE) );

	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 288.0,   0.0, 160.0,
	                                                    0.0, 284.0, 120.0,
	                                                    0.0,   0.0,   1.0);
	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0912, -0.287, 0.001889, 0.001793, 0.09828);

	std::vector<PRECISION> rotation_vector(3, 0);
	std::vector<PRECISION> translation_vector(3, 0);

	// use OpenCV routine for a distorted projection without rotation or translation
	std::vector< cv::Point_<PRECISION> > image_points;
	projectPoints(object_points,
		rotation_vector,
		translation_vector,
		camera_matrix,
		distortion_coefficients,
		image_points);

// std::cout << "_Pojected image points_" << std::endl;
// for(size_t i=0; i<image_points.size(); i++) {
// 	std::cout << "(" << image_points[i].x
// 		<< ", " << image_points[i].y
// 		<< ")" << std::endl;
// }

	// get the object points out of the image points
	std::vector< cv::Point3_<PRECISION> > reprojected_object_points;
	imagepoints_to_objectpoints(image_points,
		DISTANCE,	//distance
		camera_matrix,
		distortion_coefficients,
		reprojected_object_points);

// std::cout << "_Reprojected object points_" << std::endl;
// for(size_t i=0; i<reprojected_object_points.size(); i++) {
// 	std::cout << "(" << reprojected_object_points[i].x
// 		<< ", " << reprojected_object_points[i].y
// 		<< ", " << reprojected_object_points[i].z
// 		<< ")" << std::endl;
// }

	PRECISION err = error<cv::Point3_<PRECISION>, PRECISION>(object_points, reprojected_object_points);
	std::cout << "feature reprojection errror: " << err << std::endl;
// 	BOOST_CHECK(err <= 100*std::numeric_limits<PRECISION>::min());
	BOOST_CHECK(err <= 0.01);

	// now test own projection implementation "objectpoints_to_imagepoints" against OpenCV "projectPoints"
	rotation_vector[0] = 0.1;	//phi
	rotation_vector[1] = 0.2;	//theta
	rotation_vector[2] = 0.3;	//psi
	translation_vector[0] = 1.0;	//t1
	translation_vector[1] = -1.0;	//t2
	translation_vector[2] = 0.5;	//t3
	// no distortion
	distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);

	objectpoints_to_imagepoints(object_points,
		rotation_vector,
		translation_vector,
		camera_matrix,
		image_points);

	std::vector< cv::Point_<PRECISION> > ctrl_image_points;
	projectPoints(object_points,
		rotation_vector,
		translation_vector,
		camera_matrix,
		distortion_coefficients,
		ctrl_image_points);

	err = error<cv::Point_<PRECISION>, PRECISION>(image_points, ctrl_image_points);
	std::cout << "feature projection errror: " << err << std::endl;
	BOOST_CHECK(err <= 0.0001);
}

BOOST_AUTO_TEST_SUITE_END()
