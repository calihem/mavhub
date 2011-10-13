#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#ifdef HAVE_OPENCV_CV_H

#include "application/acc_calibration_app/acc_calibration_app.h"
#include <boost/test/unit_test.hpp>

using namespace mavhub;

BOOST_AUTO_TEST_SUITE(AccCalibrationAppTestSuite)

BOOST_AUTO_TEST_CASE(Test)
{
	BOOST_CHECK(0 == 0);
}
 
BOOST_AUTO_TEST_SUITE_END()

#endif // HAVE_OPENCV_CV_H
#endif // HAVE_MAVLINK_H