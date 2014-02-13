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
#include <inttypes.h>   //uint64

namespace mavhub {

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

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------
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

} // namespace mavhub

#endif
