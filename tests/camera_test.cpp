#include "adept/adept.h"
#include "lib/slam/camera.h"

//FIXME check OPENCV dependency
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
			BOOST_CHECK_SMALL( (double)observed_array[i], (double)epsilon); \
		else \
			BOOST_CHECK_CLOSE(expected_array[i], observed_array[i], 0.05); \
	}

template<typename T>
void myalgorithm(const T x[2], T y[2]) {
	y[0] = 4.0*cos(x[0]); y[1] = 8.0;
	const T s = 2.0*x[0] + 3.0*x[1]*x[1];
	y[0] *= sin(s);
	y[1] *= sin(s);
}

BOOST_AUTO_TEST_SUITE(hub_camera_tests)

BOOST_AUTO_TEST_CASE(pinhole_model_test) {
	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 288.0,   0.0, 160.0,
	                                                    0.0, 284.0, 120.0,
	                                                    0.0,   0.0,   1.0);
	// initialize parameter vector (phi, theta, psi, t1, t2, t3) with euler angles
	float parameter_vector_euler[] = {-deg2rad<float>(15.0), -deg2rad<float>(15.0), -deg2rad<float>(15.0), -5.0, -5.0, -5.0};
	float parameter_vector_quat[7];
	float projected_points_euler[2*num_points];
	float projected_points_quat[2*num_points];
	float reprojected_points[3*num_points];

	static const unsigned int max_iterations = 6*6;
	for(unsigned int j=0; j<max_iterations; j++) {

		/*
		 * Ideal model
		 */
		// initialize parameter vector with quaternions
		euler_to_quaternion(&parameter_vector_euler[0], &parameter_vector_quat[0]);
		memcpy(&parameter_vector_quat[4], &parameter_vector_euler[3], 3*sizeof(float));

		// ideal camera model with euler angles
		memset(projected_points_euler, 0, sizeof(projected_points_euler));
		ideal_pinhole_model<float, rotation_matrix_rad>(point_array,
			parameter_vector_euler,
			projected_points_euler,
			num_points);

		// ideal camera model using quaternions
		memset(projected_points_quat, 0, sizeof(projected_points_quat));
		for(unsigned int i=0; i<num_points; i++) {
			// single point version
			ideal_pinhole_model_quat(&point_array[i*3],
				parameter_vector_quat,
				&projected_points_quat[i*2]);
		}
		CHECK_ARRAYS(projected_points_euler, projected_points_quat, 2*num_points);

		memset(projected_points_quat, 0, sizeof(projected_points_quat));
		// multi point version
		ideal_pinhole_model_quat(point_array,
			parameter_vector_quat,
			projected_points_quat,
			num_points);
		CHECK_ARRAYS(projected_points_euler, projected_points_quat, 2*num_points);

		// inverse ideal camera model using euler angles
		memset(reprojected_points, 0, sizeof(reprojected_points));
		for(unsigned int i=0; i<num_points; i++) {
			inverse_ideal_pinhole_model<float, rotation_matrix_rad>(&projected_points_euler[i*2],
				parameter_vector_euler,
				point_array[i*3+2] + parameter_vector_euler[5],
				&reprojected_points[i*3],
				1);
		}
		// check inverse model
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points);

		// ideal inverse camera model using quaternions
		memset(reprojected_points, 0, sizeof(reprojected_points));
		for(unsigned int i=0; i<num_points; i++) {
			inverse_ideal_pinhole_model_quat(&projected_points_quat[i*2],
				parameter_vector_quat,
				point_array[i*3+2] + parameter_vector_quat[6],
				&reprojected_points[i*3],
				1);
		}
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points);

		// ideal camera model using quaternion vector
		memset(projected_points_quat, 0, sizeof(projected_points_quat));
		ideal_pinhole_model<float, rotation_matrix_quatvec>(point_array,
			&parameter_vector_quat[1],
			projected_points_quat,
			num_points);
		CHECK_ARRAYS(projected_points_euler, projected_points_quat, 2*num_points);

		// ideal inverse camera model using quaternion vector
		memset(reprojected_points, 0, sizeof(reprojected_points));
		for(unsigned int i=0; i<num_points; i++) {
			inverse_ideal_pinhole_model<float, rotation_matrix_quatvec>(&projected_points_quat[i*2],
				&parameter_vector_quat[1],
				point_array[i*3+2] + parameter_vector_quat[6],
				&reprojected_points[i*3],
				1);
		}
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points);

		/*
		 * Normal pinhole model
		 */
		// camera model using euler angles
		memset(projected_points_euler, 0, sizeof(projected_points_euler));
		pinhole_model_euler(point_array,
			parameter_vector_euler,
			camera_matrix,
			projected_points_euler,
			num_points);

		// inverse camera model using euler angles
		memset(reprojected_points, 0, sizeof(reprojected_points));
		for(unsigned int i=0; i<num_points; i++) {
			inverse_pinhole_model<float, rotation_matrix_rad>(&projected_points_euler[i*2],
				parameter_vector_euler,
				point_array[i*3+2] + parameter_vector_euler[5],
				camera_matrix,
				&reprojected_points[i*3],
				1,
				NULL);
		}
		// check inverse model
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points);

		// camera model using quaternions
		memset(projected_points_quat, 0, sizeof(projected_points_quat));
		pinhole_model_quat(point_array,
			parameter_vector_quat,
			camera_matrix,
			projected_points_quat,
			num_points);
		// check euler against quaternions
		CHECK_ARRAYS(projected_points_euler, projected_points_quat, 2*num_points);

		//TODO inverse camera model using quaternions

		// camera model using quaternion vector
		memset(projected_points_quat, 0, sizeof(projected_points_quat));
		pinhole_model_quatvec(point_array,
			&parameter_vector_quat[1],
			camera_matrix,
			projected_points_quat,
			num_points);
		// check euler against quaternion vector
		CHECK_ARRAYS(projected_points_euler, projected_points_quat, 2*num_points);

		// inverse camera model using quaternion vector
		memset(reprojected_points, 0, sizeof(reprojected_points));
		for(unsigned int i=0; i<num_points; i++) {
			inverse_pinhole_model<float, rotation_matrix_quatvec>(&projected_points_quat[i*2],
				&parameter_vector_quat[1],
				point_array[i*3+2] + parameter_vector_quat[6],
				camera_matrix,
				&reprojected_points[i*3],
				1,
				NULL);
		}
		// check inverse model
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points);

		/*
		 * Mixed ideal and normal pinhole model
		 */
		// project points by normal camera model
		memset(projected_points_quat, 0, sizeof(projected_points_quat));
		pinhole_model_quatvec(point_array,
			&parameter_vector_quat[1],
			camera_matrix,
			projected_points_quat,
			num_points);
		// normalize points
		memset(projected_points_euler, 0, sizeof(projected_points_euler));
		imagepoints_to_idealpoints(projected_points_quat, camera_matrix, projected_points_euler, num_points);
		// reproject points by ideal camera model
		memset(reprojected_points, 0, sizeof(reprojected_points));
		for(unsigned int i=0; i<num_points; i++) {
			inverse_ideal_pinhole_model<float, rotation_matrix_quatvec>(&projected_points_euler[i*2],
				&parameter_vector_quat[1],
				point_array[i*3+2] + parameter_vector_quat[6],
				&reprojected_points[i*3],
				1);
		}
		// check reprojected points
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points);

		// project by ideal camera model
		memset(projected_points_euler, 0, sizeof(projected_points_euler));
		ideal_pinhole_model<float, rotation_matrix_rad>(point_array,
			parameter_vector_euler,
			projected_points_euler,
			num_points);
		// convert normalized points to imagepoints
		idealpoints_to_imagepoints(projected_points_euler, camera_matrix, projected_points_quat, num_points);
		// reproject points by normal camera model
		memset(reprojected_points, 0, sizeof(reprojected_points));
		for(unsigned int i=0; i<num_points; i++) {
			inverse_pinhole_model<float, rotation_matrix_rad>(&projected_points_quat[i*2],
				parameter_vector_euler,
				point_array[i*3+2] + parameter_vector_euler[5],
				camera_matrix,
				&reprojected_points[i*3],
				1,
				NULL);
		}
		// check reprojected points
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points);

		/*
		 * modify parameter vector 
		 */
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

BOOST_AUTO_TEST_CASE(pinhole_translation_test) {
	float parameter_vector_euler[] = {0, 0, 0, 0, 0, 0};
	float projections[2*num_points];
	float moved_projections[2*num_points];

	ideal_pinhole_model<float, rotation_matrix_rad>(point_array,
		parameter_vector_euler,
		projections,
		num_points);

	// x-translation (forward)
	parameter_vector_euler[3] = 10;
	ideal_pinhole_model<float, rotation_matrix_rad>(point_array,
		parameter_vector_euler,
		moved_projections,
		num_points);
	for(unsigned int i=0; i<2*num_points; i+=2) {
		BOOST_CHECK(projections[i] < moved_projections[i]); 
		BOOST_CHECK_EQUAL(projections[i+1], moved_projections[i+1]); 
	}

	//TODO backward, left, right, up, down
}


// template<typename T>
// template<typename T, typename D, void(*R)(const D[3], D[9])>
template<typename T, void(*R)(const T[3], T[9])>
void my_pinhole_model(const T x[3], const T p[6], T y[2]) {

	// determine rotation matrix
	T rotation_matrix[9];
	R(p, rotation_matrix);


	y[0] = p[0] + 2*p[1] + 3*p[2] + 4*p[3] + 5*p[4] + 6*p[5];
	y[1] = p[0]*x[0] + p[1]*x[1] + p[2]*x[2] + 10*p[3]*x[0] + 10*p[4]*x[1] + 10*p[5]*x[2];
}

BOOST_AUTO_TEST_CASE(jacobian_test) {
	static const unsigned int num_input = 3;
	static const unsigned int num_param = 6;
	static const unsigned int num_output = 2;

	// TODO: test with more points and params

	//
	// compute reference jacobians via adept library
	//
	// init stack for automatic differentiation
	adept::Stack der_stack;

	// init independent variables
	adept::adouble x[num_input];
	adept::set_values(&x[0], num_input, &point_array[3]);
// 	adept::adouble x[num_input] = {1, 1, 1};
	adept::adouble p[num_param] = {1, 1, 1, 1, 1, 1};

	// run recording
	der_stack.new_recording();
	adept::adouble y[num_output];

	// run algorithm
	ideal_pinhole_model<adept::adouble, rotation_matrix_rad>(x, p, y);

	// set (in)dependent variables
// 	der_stack.independent(&x[0], num_input);
	der_stack.independent(&p[0], num_param);
	der_stack.dependent(&y[0], num_output);
	
	// determine jacobian
	double adept_jac[num_output*num_param];
	der_stack.jacobian(adept_jac);

	//
	// compute jacobians with analytical solution
	//
	double ana_x[num_input];
	for(unsigned int i=0; i<num_input; i++)
		ana_x[i] = x[i].value();
	double ana_p[num_param];
	for(unsigned int i=0; i<num_param; i++)
		ana_p[i] = p[i].value();
	double ana_jac[num_output*num_param];
	ideal_pinhole_model_euler_jac<double>(ana_x, ana_p, ana_jac);

	CHECK_ARRAYS(adept_jac, ana_jac, num_output*num_param);

/*
	std::cout << "  adept::dy0_dp = (";
	for(unsigned int i=0; i<num_param; i++) {
		std::cout << adept_jac[2*i];
		if(i != num_param-1)
			std::cout << ", ";
	}
	std::cout << ")" << std::endl;
	std::cout << "  adept::dy1_dp = (";
	for(unsigned int i=0; i<num_param; i++) {
		std::cout << adept_jac[2*i+1];
		if(i != num_param-1)
			std::cout << ", ";
	}
	std::cout << ")" << std::endl;

	std::cout << "  hub::dy0_dp   = (";
	for(unsigned int i=0; i<num_param; i++) {
		std::cout << ana_jac[2*i];
		if(i != num_param-1)
			std::cout << ", ";
	}
	std::cout << ")" << std::endl;
	std::cout << "  hub::dy1_dp   = (";
	for(unsigned int i=0; i<num_param; i++) {
		std::cout << ana_jac[2*i+1];
		if(i != num_param-1)
			std::cout << ", ";
	}
	std::cout << ")" << std::endl;
*/
}

BOOST_AUTO_TEST_CASE(Test_ideal_pinhole_jac) {
//FIXME
/*
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
*/
}

BOOST_AUTO_TEST_SUITE_END()

#endif // HAVE_OPENCV2 && CV_MINOR_VERSION >= 2
