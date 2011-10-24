/****************************************************************************
** Copyright 2011 Humboldt-Universitaet zu Berlin
**
** This file is part of MAVHUB.
**
** MAVHUB is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** MAVHUB is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with MAVHUB.  If not, see <http://www.gnu.org/licenses/>.
**
*****************************************************************************/
/**
 * \file utility.h
 * \date created at 2010/07/28
 *
 * \brief Collection of useful functions.
 * \todo Move some (longer) inline functions to cpp file.
 */

#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <fcntl.h>
#include <list>
#include <vector>
#include <iomanip>      //setw
#include <sstream>      //stringstream
#include <iterator>     //istream_iterator
#include <sys/time.h>   //gettime
#include <inttypes.h>   //uint64

namespace mavhub {

/**
 * \brief Calculates the difference between two timevals
 * \param[out] diff The difference diff = t1 - t2.
 * \param[in]  t1   The minuend.
 * \param[in]  t2   The subtrahend.
 */
static timeval& timediff(timeval &diff, const timeval &t1, const timeval &t2);

/**
 * \brief Output stream operator for lists.
 * \param[out] os The output stream.
 * \tparam[in] value_list The list of values which should be streamed to output
 * stream os.
 * \return Reference to output stream os.
 */
template<typename T>
std::ostream& operator <<(std::ostream &os, const std::list<T> &value_list);

/**
 * \brief Input stream operator for lists.
 * \param[in,out] is The input stream.
 * \tparam[out] value_list The list of values which should hold the values from
 * input stream is.
 * \return Reference to input stream is.
 *
 * \sa operator >>(std::istream &is, std::list<std::string> &string_list)
 */
template<typename T>
std::istream& operator >>(std::istream &is, std::list<T> &value_list);

/**
 * \brief Input stream operator for lists of strings.
 * \param[in,out] is The input stream.
 * \tparam[out] value_list The list of values which should hold the values from
 * input stream is.
 * \return Reference to input stream is.
 *
 * \sa operator >>(std::istream &is, std::list<T> &value_list)
 */
std::istream& operator >>(std::istream &is, std::list<std::string> &string_list);

/**
 * \brief Output stream operator for pairs.
 * \param[out] os The output stream.
 * \tparam[in] value_pair The pair of values which should be streamed to output
 * stream os.
 * \return Reference to output stream os.
 */
template<typename T1, typename T2>
std::ostream& operator <<(std::ostream &os, const std::pair<T1, T2> &value_pair);

/**
 * \brief Input stream operator for pairs.
 * \param[in,out] is The input stream.
 * \tparam[out] value_list The list of values which should hold the values from
 * input stream is.
 * \return Reference to input stream is.
 */
template<typename T1, typename T2>
std::istream& operator >>(std::istream &is, std::pair<T1, T2> &value_pair);

/**
 * \brief Output stream operator for vectors.
 * \param[out] os The output stream.
 * \tparam[in] value_vector The vector of values which should be streamed to output
 * stream os.
 * \return Reference to output stream os.
 */
template<class T>
std::ostream& operator <<(std::ostream &os, const std::vector<T> &value_vector);

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
 * Check if integer value val is in range (low <= x <= high)
 *
 * @param value x
 * @param low range limit
 * @param high range limit
 * @return true (1) if x is in specified range
 */
int in_range(int x, int low, int high);

/**
 * Check if double value val is in range (low <= x <= high)
 *
 * @param value x
 * @param low range limit
 * @param high range limit
 * @return true (1) if x is in specified range
 */
int in_range(double x, double low, double high);

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

template<typename T>
std::ostream& operator <<(std::ostream &os, const std::list<T> &value_list) {
	typename std::list<T>::const_iterator it;

	for(it = value_list.begin(); it != value_list.end(); ++it) {
		if( it != value_list.begin() )
			os << " ";
		os << *it;
	}
	return os;
}

template<typename T>
std::istream& operator >>(std::istream &is, std::list<T> &value_list) {
	T value;
	char delim;

	std::string line;
	if( std::getline(is, line) ) { //read line
		if( line.empty() )
			return is;

		std::istringstream line_stream(line);
		while( line_stream.good() ) {
			line_stream >> value;
			value_list.push_back(value);
			is >> delim;
		}
	}

	return is;
}

template<typename T1, typename T2>
std::ostream& operator <<(std::ostream &os, const std::pair<T1, T2> &value_pair) {

	os << value_pair.first << ":" << value_pair.second;
	return os;
}

template<typename T1, typename T2>
std::istream& operator >>(std::istream &is, std::pair<T1, T2> &value_pair) {
	char delim;

	is >> value_pair.first;
	is >> delim;
	is >> value_pair.second;

	return is;
}

inline std::istream& operator >>(std::istream &is, std::list<std::string> &string_list) {
	std::istream_iterator<std::string> begin(is);
	std::istream_iterator<std::string> end;
	string_list.assign(begin, end);

	return is;
}

template<class T>
std::ostream& operator <<(std::ostream &os, const std::vector<T> &value_vector) {
	typename std::vector<T>::const_iterator it;

	for(it = value_vector.begin(); it != value_vector.end(); ++it) {
		if( it != value_vector.begin() )
			os << " ";
		os << std::setw(6) << std::setfill(' ') << *it;
	}
	return os;
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

inline int in_range(int x, int low, int high) {
	return x >= low && x <= high;
}

inline int in_range(double x, double low, double high) {
	return x >= low && x <= high;
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

} // namespace mavhub

#endif
