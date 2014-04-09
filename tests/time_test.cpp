#include "lib/hub/time.h"

#include <boost/test/unit_test.hpp>

using namespace hub;

BOOST_AUTO_TEST_SUITE(hub_time_tests)

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
