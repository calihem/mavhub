#include "lib/slam/camera.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

#include <boost/test/unit_test.hpp>
#include <cstdlib>

#include "lib/slam/features.h"

using namespace hub;
using namespace hub::slam;

const float point_array[] = {
	0.0, 0.0, 10.0,
	-20.0, -25.0, 20.0,
	-25.0, 35.0, 30.0,
	22.0, 17.0, 40.0,
	10.0, -20.0, 40.0,
	10.0, -20.0, 50.0,
	-15.0, 6.0, 60.0,
	5.0, -28.0, 80.0
};

BOOST_AUTO_TEST_SUITE(CameraTestSuite)

BOOST_AUTO_TEST_CASE(Test_ideal_pinhole) {
	size_t num_points = sizeof(point_array)/sizeof(float)/3;

	std::vector<float> parameter_vector(6);
	parameter_vector[0] = 0.1;	//phi
	parameter_vector[1] = 0.2;	//theta
	parameter_vector[2] = 0.3;	//psi
	parameter_vector[3] = 1.0;	//t1
	parameter_vector[4] = -1.0;	//t2
	parameter_vector[5] = 0.5;	//t3

	// test euler
	std::vector<float> model_points(2*num_points);
	ideal_pinhole_model_euler(point_array,
		&parameter_vector[0],
		&model_points[0],
		num_points);

	std::vector< cv::Point3f > objectpoints(num_points);
	for(unsigned int i=0; i<num_points; i++) {
		objectpoints[i] = cv::Point3f(point_array[i*3], point_array[i*3 +1], point_array[i*3 + 2]);
	}
	std::vector< cv::Point2f > idealpoints;
	objectpoints_to_idealpoints(objectpoints, parameter_vector, idealpoints);

	for(unsigned int i=0; i<num_points; i++) {
		BOOST_CHECK_EQUAL( point_array[i*3], objectpoints[i].x);
		BOOST_CHECK_EQUAL( point_array[i*3 + 1], objectpoints[i].y);
		BOOST_CHECK_EQUAL( point_array[i*3 + 2], objectpoints[i].z);

		BOOST_CHECK_CLOSE( model_points[2*i], idealpoints[i].x, 0.1);
		BOOST_CHECK_CLOSE( model_points[2*i + 1], idealpoints[i].y, 0.1);
	}

	// test quaternion
	for(unsigned int i=0; i<2*num_points; i++) {
		model_points[i] = 0; // invalidate data
	}
	float quaternion[4];
	euler_to_quaternion(&parameter_vector[0], quaternion);
	parameter_vector[0] = quaternion[1];
	parameter_vector[1] = quaternion[2];
	parameter_vector[2] = quaternion[3];
	ideal_pinhole_model_quatvec(point_array,
		&parameter_vector[0],
		&model_points[0],
		num_points);
	for(unsigned int i=0; i<num_points; i++) {
		BOOST_CHECK_CLOSE( model_points[2*i], idealpoints[i].x, 0.1);
		BOOST_CHECK_CLOSE( model_points[2*i + 1], idealpoints[i].y, 0.1);
	}
}

BOOST_AUTO_TEST_CASE(Test_pinhole) {
	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 288.0,   0.0, 160.0,
	                                                    0.0, 284.0, 120.0,
	                                                    0.0,   0.0,   1.0);

	size_t num_points = sizeof(point_array)/sizeof(float)/3;

	std::vector<float> parameter_vector(6);
	parameter_vector[0] = 0.1;	//phi
	parameter_vector[1] = 0.2;	//theta
	parameter_vector[2] = 0.3;	//psi
	parameter_vector[3] = 1.0;	//t1
	parameter_vector[4] = -1.0;	//t2
	parameter_vector[5] = 0.5;	//t3

	// test euler
	std::vector<float> model_points(2*num_points);
	pinhole_model_euler(point_array,
		&parameter_vector[0],
		camera_matrix,
		&model_points[0],
		num_points);

	std::vector< cv::Point3f > objectpoints(num_points);
	for(unsigned int i=0; i<num_points; i++) {
		objectpoints[i] = cv::Point3f(point_array[i*3], point_array[i*3 +1], point_array[i*3 + 2]);
	}
	std::vector< cv::Point2f > imagepoints;
	objectpoints_to_imagepoints(objectpoints, parameter_vector, camera_matrix, imagepoints);

	for(unsigned int i=0; i<num_points; i++) {
		BOOST_CHECK_EQUAL( point_array[i*3], objectpoints[i].x);
		BOOST_CHECK_EQUAL( point_array[i*3 + 1], objectpoints[i].y);
		BOOST_CHECK_EQUAL( point_array[i*3 + 2], objectpoints[i].z);

		BOOST_CHECK_CLOSE( model_points[2*i], imagepoints[i].x, 0.1);
		BOOST_CHECK_CLOSE( model_points[2*i + 1], imagepoints[i].y, 0.1);
	}

	// test quaternion
	for(unsigned int i=0; i<2*num_points; i++) {
		model_points[i] = 0; // invalidate data
	}
	float quaternion[4];
	euler_to_quaternion(&parameter_vector[0], quaternion);
	parameter_vector[0] = quaternion[1];
	parameter_vector[1] = quaternion[2];
	parameter_vector[2] = quaternion[3];
	pinhole_model_quatvec(point_array,
		&parameter_vector[0],
		camera_matrix,
		&model_points[0],
		num_points);
	for(unsigned int i=0; i<num_points; i++) {
		BOOST_CHECK_CLOSE( model_points[2*i], imagepoints[i].x, 0.1);
		BOOST_CHECK_CLOSE( model_points[2*i + 1], imagepoints[i].y, 0.1);
	}
}

BOOST_AUTO_TEST_CASE(Test_ideal_pinhole_jac) {
	float rt[6] = {0.1, 0.2, 0.3, 1.0, -2.0, 0.5};

	// euler
	float euler_jacobian[12];
	ideal_pinhole_model_euler_jac(&point_array[0],
		rt,
		&euler_jacobian[0],
		&euler_jacobian[6]);

	//quaternion
	float qt[6];
	float quatvec_jacobian[12];
	euler_to_quaternion(rt, quatvec_jacobian);
	qt[0] = quatvec_jacobian[1]; qt[1] = quatvec_jacobian[2]; qt[2] = quatvec_jacobian[3];
	qt[3] = rt[3]; qt[4] = rt[4]; qt[5] = rt[5];
	ideal_pinhole_model_quatvec_jac(&point_array[0],
		qt,
		&quatvec_jacobian[0],
		&quatvec_jacobian[6]);

	for(unsigned int i=3; i<6; i++) {
		BOOST_CHECK_CLOSE(euler_jacobian[i], quatvec_jacobian[i], 10);
	}
}

BOOST_AUTO_TEST_SUITE_END()

#endif // HAVE_OPENCV2 && CV_MINOR_VERSION >= 2
