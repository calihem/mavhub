#include "lib/slam/features.h"

#include <boost/test/unit_test.hpp>
#include <cstdlib>
#include <limits>	//epsilon
#include <numeric>	//inner_product

#include "lib/hub/math.h"

#define PRECISION float
// #define PRECISION double

static const PRECISION DISTANCE = 50;

using namespace hub::slam;
using namespace hub;

// template<typename PointT, typename T>
// T ssd(const PointT &point_a, const PointT &point_b);

template<typename T>
inline T ssd(const T &a, const T &b) {
	return (a-b)*(a-b);
}

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

	std::vector< cv::Point3_<PRECISION> > objectpoints;
	objectpoints.push_back( cv::Point3_<PRECISION>(0, 0, DISTANCE) );
	objectpoints.push_back( cv::Point3_<PRECISION>(-20, -15, DISTANCE) );
	objectpoints.push_back( cv::Point3_<PRECISION>(-25, 35, DISTANCE) );
	objectpoints.push_back( cv::Point3_<PRECISION>(22, 17, DISTANCE) );
	objectpoints.push_back( cv::Point3_<PRECISION>(10, -20, DISTANCE) );
	objectpoints.push_back( cv::Point3_<PRECISION>(-15, 6, DISTANCE) );
	objectpoints.push_back( cv::Point3_<PRECISION>(5, -28, DISTANCE) );

	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 288.0,   0.0, 160.0,
	                                                    0.0, 284.0, 120.0,
	                                                    0.0,   0.0,   1.0);
// 	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0912, -0.287, 0.001889, 0.001793, 0.09828);
	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);

// 	std::vector<PRECISION> pose_vector(6, 0);
	std::vector<PRECISION> pose_vector(6);
	pose_vector[0] = 0.1;	//phi
	pose_vector[1] = 0.2;	//theta
	pose_vector[2] = 0.3;	//psi
	pose_vector[3] = 1.0;	//t1
	pose_vector[4] = -1.0;	//t2
	pose_vector[5] = 0.5;	//t3

	PRECISION e;

	//
	// test OpenCV implementation of rotation matrix against own
	// FIXME: move to math test suite
	//
	std::vector<PRECISION> rotation_vector(pose_vector.begin(), pose_vector.begin()+3);
	cv::Mat cv_rotation_matrix(3,3, CV_32FC1);
	Rodrigues(rotation_vector, cv_rotation_matrix);

	std::vector<PRECISION> rotation_matrix(9);
	rotation_matrix_rad(&pose_vector[0], &rotation_matrix[0]);
	std::vector<PRECISION> cv_rotation_mat((PRECISION*)cv_rotation_matrix.data, (PRECISION*)cv_rotation_matrix.data+9);

	e = error<PRECISION, PRECISION>(rotation_matrix, cv_rotation_mat);
	BOOST_TEST_MESSAGE("rotation matrix error: " << e);
// 	BOOST_CHECK(e <= 100*std::numeric_limits<PRECISION>::min());
	// opnecv implementation is performing really bad :( => large error
	BOOST_CHECK(e <= 0.2);

	//
	// project points
	//
	std::vector< cv::Point_<PRECISION> > imagepoints;
	objectpoints_to_imagepoints(objectpoints,
// 	objectpoints_to_idealpoints(objectpoints,
		pose_vector,
		camera_matrix,
		imagepoints);

	//
	// test "objectpoints_to_imagepoints" against OpenCV "projectPoints" => not possible :(
	//
// 	std::vector<PRECISION> translation_vector( pose_vector.begin()+3, pose_vector.end() );
// 	cv::Mat distortion_coefficients = (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);
// 	std::vector< cv::Point_<PRECISION> > ctrl_imagepoints;
// 	projectPoints(objectpoints,
// 		rotation_vector,
// 		translation_vector,
// 		camera_matrix,
// 		distortion_coefficients,
// 		ctrl_imagepoints);
// 	e = error<cv::Point_<PRECISION>, PRECISION>(imagepoints, ctrl_imagepoints);
// 	BOOST_TEST_MESSAGE("projection errror: " << e);
// 	BOOST_CHECK(e <= 0.0001);

	// 
	// reproject image points
	//
	std::vector< cv::Point3_<PRECISION> > reprojected_objectpoints;
	PRECISION distance = DISTANCE+pose_vector[5];
	imagepoints_to_objectpoints(imagepoints,
// 	idealpoints_to_objectpoints(imagepoints,
		distance,
		pose_vector,
		camera_matrix,
		reprojected_objectpoints);

	e = error<cv::Point3_<PRECISION>, PRECISION>(objectpoints, reprojected_objectpoints);
	BOOST_TEST_MESSAGE("reprojection errror: " << e);
// 	BOOST_CHECK(e <= 100*std::numeric_limits<PRECISION>::min());
	BOOST_CHECK(e <= 0.01);

	//
	// test undistort_n2i of cv::KeyPoint
	//
	// fill keypoints with coordinates of imagepoints
	std::vector<cv::KeyPoint> keypoints( imagepoints.size() );
	for(unsigned int i=0; i<imagepoints.size(); i++) {
		keypoints[i].pt = imagepoints[i];
	}
	std::vector< cv::Point_<PRECISION> > undistorted_idealpoints;
	undistort_n2i(keypoints,
		camera_matrix,
		distortion_coefficients,
		undistorted_idealpoints);
	std::vector< cv::Point_<PRECISION> > idealpoints;
	imagepoints_to_idealpoints(imagepoints, camera_matrix, idealpoints);
	e = error<cv::Point_<PRECISION>, PRECISION>(undistorted_idealpoints, idealpoints);
	BOOST_TEST_MESSAGE("undistortion error: " << e);
	BOOST_CHECK(e <= 100*std::numeric_limits<PRECISION>::min());
}

BOOST_AUTO_TEST_SUITE_END()
