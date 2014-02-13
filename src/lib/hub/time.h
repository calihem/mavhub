#ifndef _HUB_TIME_H_
#define _HUB_TIME_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#include <sys/time.h>   //gettime
#include <inttypes.h>   //uint64
#include <unistd.h>	//usleep

namespace hub {

/**
 * \brief Calculates the difference between two timevals
 * \param[out] diff The difference diff = t1 - t2.
 * \param[in]  t1   The minuend.
 * \param[in]  t2   The subtrahend.
 */
static timeval& timediff(timeval &diff, const timeval &t1, const timeval &t2);

/**
 * \brief Get the system time in milliseconds.
 * \return Milliseconds since 1970/1/1.
 */
static uint64_t get_time_ms();

/**
 * \brief Get the system time in microseconds.
 * \return Microseconds since 1970/1/1.
 */
static uint64_t get_time_us();

/**
 * Adds a us delta to a timeval struct
 *
 * @param time the timeval struct to which the delta should be added.
 * @param delta_us the delta time in microseconds.
 * @return the timeval struct with the added delta.
 */
struct timeval add_delta_us_to_timeval(const struct timeval &time, uint32_t delta_us);

/**
 * Adds a ns delta to a timespec struct
 *
 * @param time the timespec struct to which the delta should be added.
 * @param delta_ns the delta time in nanoseconds.
 * @return the timespec struct with the added delta.
 */
struct timespec add_delta_ns_to_timespec(const struct timespec &time, uint32_t delta_ns);

/**
 * Converts a timeval to a timespec
 *
 * @param the timeval struct.
 * @return the timespec struct.
 */
struct timespec timeval_to_timespec(const struct timeval &time);

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------
inline timeval& timediff(timeval &diff, const timeval &t1, const timeval &t2) {
	if(t2.tv_usec < t1.tv_usec) {
		diff.tv_usec = 1000000 - t1.tv_usec + t2.tv_usec;
		diff.tv_sec = t2.tv_sec - t1.tv_sec - 1;
	} else {
		diff.tv_usec = t2.tv_usec - t1.tv_usec;
		diff.tv_sec = t2.tv_sec - t1.tv_sec;
	}

	return diff;
}

inline uint64_t get_time_ms() {
	struct timeval tp;
	gettimeofday( &tp, NULL );
	return (tp.tv_sec * 1E6 + tp.tv_usec) / 1000;
}

inline uint64_t get_time_us() {
	struct timeval tp;
	gettimeofday( &tp, NULL );
	return tp.tv_sec * 1E6 + tp.tv_usec;
}

inline struct timeval add_delta_us_to_timeval(const struct timeval &time, uint32_t delta_us) {
	struct timeval ret;

	int time_div_sec = (time.tv_usec + delta_us) / 1000000;

	ret.tv_sec = time.tv_sec + time_div_sec;
	ret.tv_usec = (time.tv_usec + delta_us) % 1000000;

	return ret;
}

inline struct timespec add_delta_ns_to_timespec(const struct timespec &time, uint32_t delta_ns) {
	struct timespec ret;

	int time_div_sec = (time.tv_nsec + delta_ns) / 1000000000;

	ret.tv_sec = time.tv_sec + time_div_sec;
	ret.tv_nsec = (time.tv_nsec + delta_ns) % 1000000000;

	return ret;
}

inline struct timespec timeval_to_timespec(const struct timeval &time) {
	struct timespec ret;

	ret.tv_sec = time.tv_sec;
	ret.tv_nsec = time.tv_usec * 1000;

	return ret;
}



} // namespace hub

#endif // _HUB_TIME_H_
