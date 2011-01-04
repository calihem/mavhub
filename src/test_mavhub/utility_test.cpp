#include "../utility.h"
#include <boost/test/unit_test.hpp>

using namespace mavhub;

BOOST_AUTO_TEST_SUITE(UtilityTestSuite)

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
