#include "lib/hub/condition.h"

#include <boost/test/unit_test.hpp>

using namespace hub;
using namespace TooN;

BOOST_AUTO_TEST_SUITE(hub_condition_tests)

BOOST_AUTO_TEST_CASE(condition_test) {

	Matrix<3> matrix_a(Data(1.00000, 0.50000, 0.33333,
		0.50000, 0.33333, 0.25000,
		0.33333, 0.25000, 0.20000));
	double c = condition<3, 3, double>(matrix_a);
	BOOST_CHECK_CLOSE(c , 524.06, 0.1);
	      
	Matrix<2,6> matrix_b(Data(0.438801, 0.235866, 0.589415, 0.024590, 0.062443, 0.780826,
		0.464417, 0.661649, 0.296793, 0.232212, 0.640708, 0.561566));
	c = condition<2, 6, double>(matrix_b);
	BOOST_CHECK_CLOSE(c, 2.6756, 0.1);
}

BOOST_AUTO_TEST_SUITE_END()
