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

#define CHECK_ARRAYS(expected_array, observed_array, size, accepted_difference) \
	for(unsigned int i=0; i<size; i++) { \
		if(std::fabs(expected_array[i]) < epsilon) \
			BOOST_CHECK_SMALL( (double)observed_array[i], (double)epsilon); \
		else \
			BOOST_CHECK_CLOSE(expected_array[i], observed_array[i], accepted_difference); \
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
		CHECK_ARRAYS(projected_points_euler, projected_points_quat, 2*num_points, 0.05);

		memset(projected_points_quat, 0, sizeof(projected_points_quat));
		// multi point version
		ideal_pinhole_model_quat(point_array,
			parameter_vector_quat,
			projected_points_quat,
			num_points);
		CHECK_ARRAYS(projected_points_euler, projected_points_quat, 2*num_points, 0.05);

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
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points, 0.05);

		// ideal inverse camera model using quaternions
		memset(reprojected_points, 0, sizeof(reprojected_points));
		for(unsigned int i=0; i<num_points; i++) {
			inverse_ideal_pinhole_model_quat(&projected_points_quat[i*2],
				parameter_vector_quat,
				point_array[i*3+2] + parameter_vector_quat[6],
				&reprojected_points[i*3],
				1);
		}
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points, 0.05);

		// ideal camera model using quaternion vector
		memset(projected_points_quat, 0, sizeof(projected_points_quat));
		ideal_pinhole_model<float, rotation_matrix_quatvec>(point_array,
			&parameter_vector_quat[1],
			projected_points_quat,
			num_points);
		CHECK_ARRAYS(projected_points_euler, projected_points_quat, 2*num_points, 0.05);

		// ideal inverse camera model using quaternion vector
		memset(reprojected_points, 0, sizeof(reprojected_points));
		for(unsigned int i=0; i<num_points; i++) {
			inverse_ideal_pinhole_model<float, rotation_matrix_quatvec>(&projected_points_quat[i*2],
				&parameter_vector_quat[1],
				point_array[i*3+2] + parameter_vector_quat[6],
				&reprojected_points[i*3],
				1);
		}
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points, 0.05);

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
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points, 0.05);

		// camera model using quaternions
		memset(projected_points_quat, 0, sizeof(projected_points_quat));
		pinhole_model_quat(point_array,
			parameter_vector_quat,
			camera_matrix,
			projected_points_quat,
			num_points);
		// check euler against quaternions
		CHECK_ARRAYS(projected_points_euler, projected_points_quat, 2*num_points, 0.05);

		//TODO inverse camera model using quaternions

		// camera model using quaternion vector
		memset(projected_points_quat, 0, sizeof(projected_points_quat));
		pinhole_model_quatvec(point_array,
			&parameter_vector_quat[1],
			camera_matrix,
			projected_points_quat,
			num_points);
		// check euler against quaternion vector
		CHECK_ARRAYS(projected_points_euler, projected_points_quat, 2*num_points, 0.05);

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
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points, 0.05);

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
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points, 0.05);

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
		CHECK_ARRAYS(point_array, reprojected_points, 3*num_points, 0.05);

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

BOOST_AUTO_TEST_CASE(pinhole_jacobian_test) {
	static const unsigned int num_input = 3;
	static const unsigned int num_param = 6;
	static const unsigned int num_output = 2;

	// init stack for automatic differentiation
	adept::Stack der_stack;

	// init independent variables
	adept::adouble x[num_input];
	adept::adouble p[num_param] = {deg2rad<double>(5.0), -deg2rad<float>(12.0), deg2rad<float>(33.0), -5.0, 8.0, 3.0};

	for(unsigned int point_idx = 0; point_idx<num_points; point_idx++) {
		adept::set_values(&x[0], num_input, &point_array[point_idx*num_input]);

		//
		// compute reference jacobians via adept library
		//
		// run recording
		der_stack.new_recording();
		adept::adouble y[num_output];
		// run algorithm
		ideal_pinhole_model<adept::adouble, rotation_matrix_rad>(x, p, y);
		// set (in)dependent variables
		der_stack.independent(&p[0], num_param);
		der_stack.dependent(&y[0], num_output);
		// determine jacobian
		double adept_jac[num_output*num_param];
		der_stack.jacobian(adept_jac, true); // get jacobian in row-major order

		//
		// compute jacobians with analytical solution (euler)
		//
		double ana_x[num_input];
		for(unsigned int i=0; i<num_input; i++)
			ana_x[i] = x[i].value();
		double ana_p[num_param];
		for(unsigned int i=0; i<num_param; i++)
			ana_p[i] = p[i].value();
		double ana_jac[num_output][num_param];
		ideal_pinhole_model_euler_jac<double>(ana_x, ana_p, ana_jac);

		// jacobian with euler angles seem to be rather noisy :(
		CHECK_ARRAYS(adept_jac, ana_jac[0], num_output*num_param, 33.0);

		//
		// check quaternion (vector) solution
		//
		double quaternion[4];
		euler_to_quaternion(ana_p, quaternion);
		adept::set_values(&p[0], 3, &quaternion[1]);
		// determine jacobian reference solution with adept
		der_stack.new_recording();
		ideal_pinhole_model<adept::adouble, rotation_matrix_quatvec>(x, p, y);
		der_stack.independent(&p[0], num_param);
		der_stack.dependent(&y[0], num_output);
		der_stack.jacobian(adept_jac, true);
		// determine jacobian with analytical solution
		for(unsigned int i=0; i<3; i++)
			ana_p[i] = p[i].value();
		ideal_pinhole_model_quatvec_jac(ana_x, ana_p, ana_jac);

		CHECK_ARRAYS(adept_jac, ana_jac[0], num_output*num_param, 0.05);

		//
		// check quaternion (vector) solution with structure
		//
		// determine jacobian reference solution with adept
		der_stack.new_recording();
		ideal_pinhole_model<adept::adouble, rotation_matrix_quatvec>(x, p, y);
		der_stack.independent(&p[0], num_param);
		der_stack.independent(&x[0], num_input);
		der_stack.dependent(&y[0], num_output);
		double adept_jac_qts[num_output*(num_param+num_input)];
		der_stack.jacobian(adept_jac_qts, true);
		// determine motion and structure jacobians using analytical solution
		double ana_jac_s[num_output][num_input];
		ideal_pinhole_model_quatvec_jac(ana_x, ana_p, ana_jac, ana_jac_s);

		CHECK_ARRAYS(adept_jac_qts, ana_jac[0], num_param, 0.05);
		CHECK_ARRAYS((&adept_jac_qts[num_param]), ana_jac_s[0], num_input, 0.05);
		CHECK_ARRAYS((&adept_jac_qts[num_param+num_input]), ana_jac[1], num_param, 0.05);
		CHECK_ARRAYS((&adept_jac_qts[2*num_param+num_input]), ana_jac_s[1], num_input, 0.05);
	}
}

BOOST_AUTO_TEST_SUITE_END()

#endif // HAVE_OPENCV2 && CV_MINOR_VERSION >= 2
