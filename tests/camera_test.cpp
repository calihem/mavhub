#include "lib/slam/camera.h"

#if defined HAVE_OPENCV2 && CV_MINOR_VERSION >= 2

#include <boost/test/unit_test.hpp>
#include <cstdlib>

using namespace hub;
using namespace hub::slam;

static const float epsilon = 0.0001;

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

static const size_t num_points = sizeof(point_array)/sizeof(float)/3;

#define CHECK_ARRAYS(expected_array, observed_array, size) \
	for(unsigned int i=0; i<size; i++) { \
		if(std::fabs(expected_array[i]) < epsilon) \
			BOOST_CHECK_SMALL(observed_array[i], epsilon); \
		else \
			BOOST_CHECK_CLOSE(expected_array[i], observed_array[i], 0.05); \
	}

BOOST_AUTO_TEST_SUITE(hub_camera_tests)

BOOST_AUTO_TEST_CASE(Test_ideal_pinhole) {

	// initialize parameter vector (phi, theta, psi, t1, t2, t3) with euler angles
// 	float parameter_vector_euler[] = {0.1, 0.2, 0.3, 1.0, -1.0, 0.5};
	float parameter_vector_euler[] = {-deg2rad<float>(15.0), -deg2rad<float>(15.0), -deg2rad<float>(15.0), -5.0, -5.0, -5.0};
	float parameter_vector_quat[7];
	float ideal_points_euler[2*num_points];
	float ideal_points_quat[2*num_points];
	float reprojected_points[3*num_points];

	static const unsigned int max_iterations = 6*6;
	for(unsigned int j=0; j<max_iterations; j++) {

		// initialize parameter vector with quaternions
		euler_to_quaternion(&parameter_vector_euler[0], &parameter_vector_quat[0]);
		memcpy(&parameter_vector_quat[4], &parameter_vector_euler[3], 3*sizeof(float));

		// camera model with euler angles
		memset(ideal_points_euler, 0, sizeof(ideal_points_euler));
		ideal_pinhole_model<float, rotation_matrix_rad>(point_array,
			parameter_vector_euler,
			ideal_points_euler,
			num_points);

		// camera model using quaternions
		memset(ideal_points_quat, 0, sizeof(ideal_points_quat));
		for(unsigned int i=0; i<num_points; i++) {
			ideal_pinhole_model_quat(&point_array[i*3],
				parameter_vector_quat,
				&ideal_points_quat[i*2]);
		}
		CHECK_ARRAYS(ideal_points_euler, ideal_points_quat, 2*num_points);

		memset(ideal_points_quat, 0, sizeof(ideal_points_quat));
		ideal_pinhole_model_quat(point_array,
			parameter_vector_quat,
			ideal_points_quat,
			num_points);
		CHECK_ARRAYS(ideal_points_euler, ideal_points_quat, 2*num_points);

		// inverse camera model using euler angles
		memset(reprojected_points, 0, sizeof(reprojected_points));
		for(unsigned int i=0; i<num_points; i++) {
			inverse_ideal_pinhole_model<float, rotation_matrix_rad>(&ideal_points_euler[i*2],
				parameter_vector_euler,
				point_array[i*3+2] + parameter_vector_euler[5],
				&reprojected_points[i*3],
				1);
		}
		// check inverse model
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points);

		// inverse camera model using quaternions
		memset(reprojected_points, 0, sizeof(reprojected_points));
		for(unsigned int i=0; i<num_points; i++) {
			inverse_ideal_pinhole_model_quat(&ideal_points_quat[i*2],
				parameter_vector_quat,
				point_array[i*3+2] + parameter_vector_quat[6],
				&reprojected_points[i*3],
				1);
		}
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points);

		// camera model using quatvec
		memset(ideal_points_quat, 0, sizeof(ideal_points_quat));
		ideal_pinhole_model<float, rotation_matrix_quatvec>(point_array,
			&parameter_vector_quat[1],
			ideal_points_quat,
			num_points);
		CHECK_ARRAYS(ideal_points_euler, ideal_points_quat, 2*num_points);

		// inverse camera model using quaternion vector
		memset(reprojected_points, 0, sizeof(reprojected_points));
		for(unsigned int i=0; i<num_points; i++) {
			inverse_ideal_pinhole_model<float, rotation_matrix_quatvec>(&ideal_points_quat[i*2],
				&parameter_vector_quat[1],
				point_array[i*3+2] + parameter_vector_quat[6],
				&reprojected_points[i*3],
				1);
		}
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points);

		// modify parameter vector
		switch(j % 6) {
			case 0: parameter_vector_euler[5] += 5.0;
				break;
			case 1:
				parameter_vector_euler[4] += 5.0;
				break;
			case 2:
				parameter_vector_euler[3] += 5.0;
				break;
			case 3:
				parameter_vector_euler[2] += deg2rad(15.0);
				break;
			case 4:
				parameter_vector_euler[1] += deg2rad(15.0);
				break;
			case 5:
				parameter_vector_euler[0] += deg2rad(15.0);
				break;
		}
	}
}

BOOST_AUTO_TEST_CASE(Test_pinhole) {
	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 288.0,   0.0, 160.0,
	                                                    0.0, 284.0, 120.0,
	                                                    0.0,   0.0,   1.0);

	std::vector<float> parameter_vector(6);
	parameter_vector[0] = 0.1;	//phi
	parameter_vector[1] = 0.2;	//theta
	parameter_vector[2] = 0.3;	//psi
	parameter_vector[3] = 1.0;	//t1
	parameter_vector[4] = -1.0;	//t2
	parameter_vector[5] = 0.5;	//t3

	// project by euler angles
	std::vector<float> image_points_euler(2*num_points);
	pinhole_model_euler(point_array,
		&parameter_vector[0],
		camera_matrix,
		&image_points_euler[0],
		num_points);

	// project by quaternions
	std::vector<float> parameter_vector_quat(7);
	euler_to_quaternion(&parameter_vector[0], &parameter_vector_quat[0]);
	parameter_vector_quat[4] = parameter_vector[3];
	parameter_vector_quat[5] = parameter_vector[4];
	parameter_vector_quat[6] = parameter_vector[5];
	std::vector<float> image_points_quat(2*num_points);
	pinhole_model_quat(point_array,
		&parameter_vector_quat[0],
		camera_matrix,
		&image_points_quat[0],
		num_points);
	// check euler against quaternions
	CHECK_ARRAYS( image_points_euler, image_points_quat, 2*num_points);

	// project by quaternion vector
	memset(&image_points_quat[0], 0, sizeof(image_points_quat));
	pinhole_model_quatvec(point_array,
		&parameter_vector_quat[1],
		camera_matrix,
		&image_points_quat[0],
		num_points);

	// check euler against quaternion vector
	CHECK_ARRAYS( image_points_euler, image_points_quat, 2*num_points);
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
