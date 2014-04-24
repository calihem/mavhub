#include "lib/hub/math.h"

#include <boost/test/unit_test.hpp>

using namespace hub;

BOOST_AUTO_TEST_SUITE(hub_math_tests)

BOOST_AUTO_TEST_CASE(euler_quaternion_test) {
	static const double start_value = -pi/2+0.1; //avoid singularities
	const unsigned int factor = 4;
	double euler_angles[3] = {start_value, start_value, start_value};
	double quaternion[4];
	double euler_result[3];

	const unsigned int max_iterations = pow(3, factor);
	for(unsigned int i=1; i<max_iterations; i++) {
		euler_to_quaternion(&euler_angles[0], &quaternion[0]);
		quaternion_to_euler(&quaternion[0], &euler_result[0]);

		// test against rotation matrix
		double rot_matrix_euler[9];
		double rot_matrix_result[9];
		rotation_matrix_rad(&euler_angles[0], &rot_matrix_euler[0]);
		rotation_matrix_rad(&euler_result[0], &rot_matrix_result[0]);
		for(unsigned int j=0; j<9; j++) {
			//skip values near 0
			if( abs(rot_matrix_euler[j]) < 0.000001
			|| abs(rot_matrix_result[j]) < 0.000001)
				continue;
			BOOST_CHECK_CLOSE(rot_matrix_euler[j], rot_matrix_result[j], 1.0);
		}
/*
		// to get rid of 0 != 2pi test against sin of value
		if(euler_angles[0] != 0) //skip zero
			BOOST_CHECK_CLOSE(euler_angles[0], euler_result[0], 0.1);
		if(euler_angles[1] != 0) //skip zero
			BOOST_CHECK_CLOSE(sin(euler_angles[1]), sin(euler_result[1]), 0.1);
		if(euler_angles[2] != 0) //skip zero
			BOOST_CHECK_CLOSE(euler_angles[2], euler_result[2], 0.1);
*/
		if( !(i % (factor)) ) {
			euler_angles[2] = start_value;
			if( !(i % (int)(pow(2,factor))) ) {
				euler_angles[1] = start_value;
				euler_angles[0] += pi/factor;
			} else
				euler_angles[1] += pi/factor;
		} else
			euler_angles[2] += pi/factor;
	}
}

BOOST_AUTO_TEST_CASE(multiply_test) {
	float a[3] = {0.5, 0.75, 0.33};
	float M[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	float b[3];

	// identity matrix
	multiply(M, a, b);
	BOOST_CHECK_EQUAL_COLLECTIONS(a, a+3, b, b+3);
	// null matrix
	memset(M, 0, 9*sizeof(float));
	multiply(M, a, b);
	BOOST_CHECK_EQUAL(b[0], 0); BOOST_CHECK_EQUAL(b[1], 0); BOOST_CHECK_EQUAL(b[2], 0);
	// randomly well chosen example :)
	M[0] = 1.5; M[1] = 2.75; M[2] = 3.11;
	M[3] = 0.8; M[4] = 25.33; M[5] = 4.57;
	M[6] = 88.8; M[7] = 5.42; M[8] = 1.66;
	multiply(M, a, b);
	BOOST_CHECK_EQUAL(b[0], a[0]*M[0]+a[1]*M[1]+a[2]*M[2]);
	BOOST_CHECK_EQUAL(b[1], a[0]*M[3]+a[1]*M[4]+a[2]*M[5]);
	BOOST_CHECK_EQUAL(b[2], a[0]*M[6]+a[1]*M[7]+a[2]*M[8]);
	
	//TODO matrix multiplication
	
	//TODO quaternion multiplication
}

BOOST_AUTO_TEST_CASE(rotation_test) {
// macro to check similarities of rotation matrices
#define CHECK_ROT_MATRICES(m_euler, m_quat) \
	for(unsigned int j=0; j<9; j++) { \
		if(std::fabs(m_euler[j]) < 0.000001) \
			BOOST_CHECK_SMALL(m_quat[j], 0.000001); \
		else \
			BOOST_CHECK_CLOSE(m_euler[j], m_quat[j], 0.1); \
	}
// macro to check orthogonality of a matrix FIXME with all the variables it is very nasty and not reusable
#define CHECK_ORTHOGONALITY(matrix) \
	double _inv_matrix[9]; \
	_inv_matrix[0] = matrix[0]; _inv_matrix[1] = matrix[3]; _inv_matrix[2] = matrix[6]; \
	_inv_matrix[3] = matrix[1]; _inv_matrix[4] = matrix[4]; _inv_matrix[5] = matrix[7]; \
	_inv_matrix[6] = matrix[2]; _inv_matrix[7] = matrix[5]; _inv_matrix[8] = matrix[8]; \
	double _matrix_product[9]; \
	multiply_matrix(matrix, _inv_matrix, _matrix_product); \
	_matrix_product[0] -= 1.0; _matrix_product[4] -= 1.0; _matrix_product[8] -= 1.0; \
	for(unsigned int j=0; j<9; j++) { \
		BOOST_CHECK_SMALL(_matrix_product[j], 0.000001); \
	}

	const unsigned int factor = 4;
	double euler_angles[3] = {0.0, 0.0, 0.0};
	double quaternion[4];
	double rot_matrix_euler[9];
	double rot_matrix_quat[9];

	// check rotation around fraction of pi
	const unsigned int max_iterations = pow(3, factor);
	for(unsigned int i=1; i<(max_iterations-1); i++) {
		rotation_matrix_rad(&euler_angles[0], &rot_matrix_euler[0]);
		CHECK_ORTHOGONALITY(rot_matrix_euler);

		euler_to_quaternion(&euler_angles[0], quaternion);
		rotation_matrix_quat(quaternion, &rot_matrix_quat[0]);
		CHECK_ROT_MATRICES(rot_matrix_euler, rot_matrix_quat);

		rotation_matrix_quatvec(&quaternion[1], &rot_matrix_quat[0]);
		CHECK_ROT_MATRICES(rot_matrix_euler, rot_matrix_quat);

		if( !(i % (factor)) ) {
			euler_angles[2] = 0;
			if( !(i % (int)(pow(2,factor))) ) {
				euler_angles[1] = 0;
				euler_angles[0] += pi/factor;
			} else
				euler_angles[1] += pi/factor;
		} else
			euler_angles[2] += pi/factor;
	}
}

BOOST_AUTO_TEST_CASE(coordinate_system_test) {
	static const double tolerance = 0.0001;
	double point[] = {1.0, 2.0, 3.0};
	double euler_angles[3];
	double rotation_matrix[9];
	double result[3];

	//rotate 90 deg around roll
	euler_angles[0] = pi/2; euler_angles[1] = 0; euler_angles[2] = 0; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], result[0], tolerance);
	BOOST_CHECK_CLOSE(point[1], result[2], tolerance);
	BOOST_CHECK_CLOSE(point[2], -result[1], tolerance);

	//rotate 90 deg around pitch
	euler_angles[0] = 0; euler_angles[1] = pi/2; euler_angles[2] = 0; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], -result[2], tolerance);
	BOOST_CHECK_CLOSE(point[1], result[1], tolerance);
	BOOST_CHECK_CLOSE(point[2], result[0], tolerance);

	//rotate 90 deg around yaw
	euler_angles[0] = 0; euler_angles[1] = 0; euler_angles[2] = pi/2; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], result[1], tolerance);
	BOOST_CHECK_CLOSE(point[1], -result[0], tolerance);
	BOOST_CHECK_CLOSE(point[2], result[2], tolerance);

	//rotate 90 deg around roll and pitch
	euler_angles[0] = pi/2; euler_angles[1] = pi/2; euler_angles[2] = 0; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], -result[2], tolerance);
	BOOST_CHECK_CLOSE(point[1], result[0], tolerance);
	BOOST_CHECK_CLOSE(point[2], -result[1], tolerance);

	//rotate 90 deg around pitch and yaw
	euler_angles[0] = 0; euler_angles[1] = pi/2; euler_angles[2] = pi/2; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], -result[2], tolerance);
	BOOST_CHECK_CLOSE(point[1], -result[0], tolerance);
	BOOST_CHECK_CLOSE(point[2], result[1], tolerance);

	//rotate 90 deg around roll and yaw
	euler_angles[0] = pi/2; euler_angles[1] = 0; euler_angles[2] = pi/2; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], result[1], tolerance);
	BOOST_CHECK_CLOSE(point[1], result[2], tolerance);
	BOOST_CHECK_CLOSE(point[2], result[0], tolerance);

	//rotate 90 deg around roll, pitch and yaw
	euler_angles[0] = pi/2; euler_angles[1] = pi/2; euler_angles[2] = pi/2; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], -result[2], tolerance);
	BOOST_CHECK_CLOSE(point[1], result[1], tolerance);
	BOOST_CHECK_CLOSE(point[2], result[0], tolerance);
}

BOOST_AUTO_TEST_CASE(intersection_test) {
	double plane_point[] = {0, 0, 100};
	double plane_normal_vector[] = { 0, 0, 1};
	double line_point[] = {0, 0, 0};

	//rotate roll by 10deg
	double line_direction[3];
	double euler_angles[] = {10.0*pi/180.0, 0, 0};
	double rotation_matrix[9];
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, plane_normal_vector, line_direction);
	double intersection_point[3];

	int rc = intersection(plane_point,
		plane_normal_vector,
		line_point,
		line_direction,
		intersection_point);
	BOOST_CHECK_EQUAL(rc, 0);
	BOOST_CHECK_CLOSE( intersection_point[0], 0, 1.0 );
	BOOST_CHECK_CLOSE( intersection_point[1], -plane_point[2]*tan(euler_angles[0]), 1.0 );
	BOOST_CHECK_CLOSE( intersection_point[2], plane_point[2], 1.0 );

	//rotate pitch by 25deg
	euler_angles[0] = 0; euler_angles[1] = 25*pi/180; euler_angles[2] = 0;
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, plane_normal_vector, line_direction);

	// move by 5
	line_point[0] = 5;

	rc = intersection(plane_point,
		plane_normal_vector,
		line_point,
		line_direction,
		intersection_point);
	BOOST_CHECK_EQUAL(rc, 0);
	BOOST_CHECK_CLOSE( intersection_point[0], plane_point[2]*tan(euler_angles[1]) + line_point[0], 1.0 );
	BOOST_CHECK_CLOSE( intersection_point[1], 0, 1.0 );
	BOOST_CHECK_CLOSE( intersection_point[2], plane_point[2], 1.0 );

// 	line_point[0] = 0.169848;
// 	line_point[1] = 2.20749;
// 	line_point[2] = -0.152189;
// 	double quatvec[] = {-0.0312501, -0.0052537, 0.00544638};
// 	rotation_matrix_quatvec(quatvec, rotation_matrix);
// 	multiply(rotation_matrix, plane_normal_vector, line_direction);
// 	rc = intersection(plane_point,
// 		plane_normal_vector,
// 		line_point,
// 		line_direction,
// 		intersection_point);
// std::cout << "quatvec:        " << quatvec[0] << ", " << quatvec[1] << ", " << quatvec[2] << std::endl;
// double quaternion[4];
// vec2quat(quatvec, quaternion);
// quaternion_to_euler(quaternion, euler_angles);
// euler_angles[0] = rad2deg(euler_angles[0]);
// euler_angles[1] = rad2deg(euler_angles[1]);
// euler_angles[2] = rad2deg(euler_angles[2]);
// std::cout << "euler angles:   " << euler_angles[0] << ", " << euler_angles[1] << ", " << euler_angles[2] << std::endl;
// std::cout << "line direction: " << line_direction[0] << ", " << line_direction[1] << ", " << line_direction[2] << std::endl;
// std::cout << "intersection:   " << intersection_point[0] << ", " << intersection_point[1] << ", " << intersection_point[2] << std::endl;
// exit(0);
}

BOOST_AUTO_TEST_CASE(cosine_similarity_test) {
	float vector_a_data[] = { 0.0766469, 0.0108075, -0.633263, 0.0244403, -0.103504, 0.171257};
	float vector_b_data[] = {-0.035064,  0.015965,  -0.734642, 0.080307,   0.005341, 0.096551};
	std::vector<float> vector_a( vector_a_data, vector_a_data + sizeof(vector_a_data)/sizeof(float) ); 
	std::vector<float> vector_b( vector_b_data, vector_b_data + sizeof(vector_b_data)/sizeof(float) ); 

	float cos_sim = cosine_similarity(vector_a, vector_b);
	BOOST_CHECK_CLOSE(cos_sim, 0.962583953865, 0.001);

	vector_b.resize(3);
	cos_sim = cosine_similarity(vector_a, vector_b);
	BOOST_CHECK_CLOSE(cos_sim, 0.94008, 0.001);
	cos_sim = cosine_similarity(vector_b, vector_a);
	BOOST_CHECK_CLOSE(cos_sim, 0.94008, 0.001);

	vector_a.resize(0); vector_b.resize(0);
	cos_sim = cosine_similarity(vector_a, vector_b);
	BOOST_CHECK_EQUAL(cos_sim, 1.0);
}

BOOST_AUTO_TEST_SUITE_END()
