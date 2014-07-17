#include "lib/hub/condition.h"

#include <boost/test/unit_test.hpp>

using namespace hub;
using namespace TooN;

BOOST_AUTO_TEST_SUITE(hub_condition_tests)

BOOST_AUTO_TEST_CASE(condition_test) {

	// 3x3 hilbert matrix
	Matrix<3> matrix_a(Data(1.00000, 0.50000, 0.33333,
		0.50000, 0.33333, 0.25000,
		0.33333, 0.25000, 0.20000));
	double c = condition<3, 3, double>(matrix_a);
	BOOST_CHECK_CLOSE(c , 524.06, 0.1);

	// random 6x2 static matrix
	Matrix<2,6> matrix_b(Data(0.438801, 0.235866, 0.589415, 0.024590, 0.062443, 0.780826,
		0.464417, 0.661649, 0.296793, 0.232212, 0.640708, 0.561566));
	c = condition<2, 6, double>(matrix_b);
	BOOST_CHECK_CLOSE(c, 2.6756, 0.1);
	c = condition<6, 2, double>(matrix_b.T());
	BOOST_CHECK_CLOSE(c, 2.6756, 0.1);

	// random 4x3 dynamic matrix
	Matrix<> matrix_c(4,3);
	Fill(matrix_c) = 0.91320, 0.21212, 0.31493,
		0.64008, 0.49275, 0.89388,
		0.84612, 0.29287, 0.58894,
		0.31986, 0.63137, 0.66012;
	c = condition<-1, -1, double>(matrix_c);
	BOOST_CHECK_CLOSE(c, 11.559, 0.1);
/*
	// random 2x3 dynamic matrix from data array
	double matrix_data[] = {0.062463, 0.079047, 0.602560, 0.817685, 0.328538, 0.903340};
	Matrix<> matrix_d(matrix_data, 2, 3);
	c = condition<Dynamic, Dynamic, double>(matrix_d);
	BOOST_CHECK_CLOSE(c, 4.0609, 0.1);

	Matrix<> matrix_e(2, 3);
	memcpy(&matrix_e(0,0), matrix_data, sizeof(matrix_data));
	c = condition<Dynamic, Dynamic, double>(matrix_e);
	BOOST_CHECK_CLOSE(c, 4.0609, 0.1);
*/
}

BOOST_AUTO_TEST_SUITE_END()
