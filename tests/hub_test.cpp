#include "lib/hub/math.h"
#include "lib/hub/time.h"

#include <boost/test/unit_test.hpp>

using namespace hub;

BOOST_AUTO_TEST_SUITE(LibHUBTestSuite)

BOOST_AUTO_TEST_CASE(Test_math_euler_quaternion) {
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

BOOST_AUTO_TEST_CASE(Test_math_multiply) {
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
}

BOOST_AUTO_TEST_CASE(Test_math_rotation) {
	const unsigned int factor = 4;
	double euler_angles[3] = {0.0, 0.0, 0.0};
	double quaternion[4];
	double rot_matrix_euler[9];
	double rot_matrix_quat[9];

	const unsigned int max_iterations = pow(3, factor);
	for(unsigned int i=1; i<(max_iterations-1); i++) {
		rotation_matrix_rad(&euler_angles[0], &rot_matrix_euler[0]);

		euler_to_quaternion(&euler_angles[0], quaternion);
		rotation_matrix_quat(quaternion, &rot_matrix_quat[0]);
		for(unsigned int j=0; j<9; j++) {
			//skip values near 0
			if( abs(rot_matrix_euler[j]) < 0.000001
			|| abs(rot_matrix_quat[j]) < 0.000001)
				continue;
			BOOST_CHECK_CLOSE(rot_matrix_euler[j], rot_matrix_quat[j], 1.0);
		}

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

BOOST_AUTO_TEST_CASE(Test_math_coordinate_system) {
	double point[] = {1.0, 2.0, 3.0};
	double euler_angles[3];
	double rotation_matrix[9];
	double result[3];

	//rotate 90 deg around roll
	euler_angles[0] = pi/2; euler_angles[1] = 0; euler_angles[2] = 0; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], result[0], 1.0);
	BOOST_CHECK_CLOSE(point[1], result[2], 1.0);
	BOOST_CHECK_CLOSE(point[2], -result[1], 1.0);

	//rotate 90 deg around pitch
	euler_angles[0] = 0; euler_angles[1] = pi/2; euler_angles[2] = 0; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], -result[2], 1.0);
	BOOST_CHECK_CLOSE(point[1], result[1], 1.0);
	BOOST_CHECK_CLOSE(point[2], result[0], 1.0);

	//rotate 90 deg around yaw
	euler_angles[0] = 0; euler_angles[1] = 0; euler_angles[2] = pi/2; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], result[1], 1.0);
	BOOST_CHECK_CLOSE(point[1], -result[0], 1.0);
	BOOST_CHECK_CLOSE(point[2], result[2], 1.0);

	//rotate 90 deg around roll and pitch
	euler_angles[0] = pi/2; euler_angles[1] = pi/2; euler_angles[2] = 0; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], -result[2], 1.0);
	BOOST_CHECK_CLOSE(point[1], result[0], 1.0);
	BOOST_CHECK_CLOSE(point[2], -result[1], 1.0);

	//rotate 90 deg around pitch and yaw
	euler_angles[0] = 0; euler_angles[1] = pi/2; euler_angles[2] = pi/2; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], -result[2], 1.0);
	BOOST_CHECK_CLOSE(point[1], -result[0], 1.0);
	BOOST_CHECK_CLOSE(point[2], result[1], 1.0);

	//rotate 90 deg around roll and yaw
	euler_angles[0] = pi/2; euler_angles[1] = 0; euler_angles[2] = pi/2; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], result[1], 1.0);
	BOOST_CHECK_CLOSE(point[1], result[2], 1.0);
	BOOST_CHECK_CLOSE(point[2], result[0], 1.0);

	//rotate 90 deg around roll, pitch and yaw
	euler_angles[0] = pi/2; euler_angles[1] = pi/2; euler_angles[2] = pi/2; 
	rotation_matrix_rad(euler_angles, rotation_matrix);
	multiply(rotation_matrix, point, result);
	BOOST_CHECK_CLOSE(point[0], -result[2], 1.0);
	BOOST_CHECK_CLOSE(point[1], result[1], 1.0);
	BOOST_CHECK_CLOSE(point[2], result[0], 1.0);
}

BOOST_AUTO_TEST_CASE(Test_math_intersection) {
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

BOOST_AUTO_TEST_CASE(Test_timediff)
{
	struct timeval time1, time2, res;
	
	// Test zero diff
	time1.tv_sec = 0;
	time1.tv_usec = 0;
	time2.tv_sec = 0;
	time2.tv_usec = 0;
	
	timediff(res, time1, time2);
	
	BOOST_CHECK_EQUAL(res.tv_sec, 0);
	BOOST_CHECK_EQUAL(res.tv_usec, 0);
	
	// Test time2 non zero, time1 zero diff
	time2.tv_sec = 123;
	time2.tv_usec = 12345;
	
	timediff(res, time1, time2);
	
	BOOST_CHECK_EQUAL(res.tv_sec, 123);
	BOOST_CHECK_EQUAL(res.tv_usec, 12345);
	
	// Test time2 and time1 nonzero diff
	time1.tv_sec = 1;
	time1.tv_usec = 5;
	
	timediff(res, time1, time2);
	
	BOOST_CHECK_EQUAL(res.tv_sec, 122);
	BOOST_CHECK_EQUAL(res.tv_usec, 12340);

	// Test time2 and time1 with overflow diff
	time1.tv_sec = 120;
	time1.tv_usec = 12346;
	
	timediff(res, time1, time2);
	
	BOOST_CHECK_EQUAL(res.tv_sec, 2);
	BOOST_CHECK_EQUAL(res.tv_usec, 999999);

}

BOOST_AUTO_TEST_CASE(Test_add_delta_us_to_timeval)
{
	// Test some delta adds
	struct timeval time1;
	time1.tv_sec = 0;
	time1.tv_usec = 0;
	
	struct timeval timeret;
	
	timeret = add_delta_us_to_timeval(time1, 1);
	
	BOOST_CHECK_EQUAL(timeret.tv_sec, 0);
	BOOST_CHECK_EQUAL(timeret.tv_usec, 1);
	
	// Test another add
	time1.tv_sec = 1;
	time1.tv_usec = 512;
	
	timeret = add_delta_us_to_timeval(time1, 1);
	
	BOOST_CHECK_EQUAL(timeret.tv_sec, 1);
	BOOST_CHECK_EQUAL(timeret.tv_usec, 513);
	
	// Test an add with second overflow
	time1.tv_sec = 5000;
	time1.tv_usec = 500003;
	
	timeret = add_delta_us_to_timeval(time1, 3000001);
	
	BOOST_CHECK_EQUAL(timeret.tv_sec, 5003);
	BOOST_CHECK_EQUAL(timeret.tv_usec, 500004);
}

BOOST_AUTO_TEST_CASE(Test_add_delta_ns_to_timespec)
{
	// Test some delta adds
	struct timespec time1;
	time1.tv_sec = 0;
	time1.tv_nsec = 0;
	
	struct timespec timeret;
	
	timeret = add_delta_ns_to_timespec(time1, 1);
	
	BOOST_CHECK_EQUAL(timeret.tv_sec, 0);
	BOOST_CHECK_EQUAL(timeret.tv_nsec, 1);
	
	// Test another add
	time1.tv_sec = 1;
	time1.tv_nsec = 512;
	
	timeret = add_delta_ns_to_timespec(time1, 1);
	
	BOOST_CHECK_EQUAL(timeret.tv_sec, 1);
	BOOST_CHECK_EQUAL(timeret.tv_nsec, 513);
	
	// Test an add with second overflow
	time1.tv_sec = 5000;
	time1.tv_nsec = 500000003;
	
	timeret = add_delta_ns_to_timespec(time1, 2000000001);
	
	BOOST_CHECK_EQUAL(timeret.tv_sec, 5002);
	BOOST_CHECK_EQUAL(timeret.tv_nsec, 500000004);
}

BOOST_AUTO_TEST_CASE(Test_timeval_to_timespec)
{
	// Test conversion with zeros
	struct timeval time;
	struct timespec timeret;
	
	time.tv_sec = 0;
	time.tv_usec = 0;
	
	timeret = timeval_to_timespec(time);
	
	BOOST_CHECK_EQUAL(timeret.tv_sec, 0);
	BOOST_CHECK_EQUAL(timeret.tv_nsec, 0);
	
	// Test conversion with non-zeros
	time.tv_sec = 15;
	time.tv_usec = 22345;
	
	timeret = timeval_to_timespec(time);
	
	BOOST_CHECK_EQUAL(timeret.tv_sec, 15);
	BOOST_CHECK_EQUAL(timeret.tv_nsec, 22345000);
}

BOOST_AUTO_TEST_SUITE_END()