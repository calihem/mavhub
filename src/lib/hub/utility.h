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

#ifndef _HUB_UTILITY_H_
#define _HUB_UTILITY_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#include <fcntl.h>
#include <list>
#include <vector>
#include <iomanip>      //setw
#include <sstream>      //stringstream
#include <iterator>     //istream_iterator
#include <inttypes.h>   //uint64

namespace hub {

/**
 * \brief Simple struct combining data item with an index.
 */
template<typename T>
struct indexed_item_t {
	indexed_item_t() {};
	indexed_item_t(const T &item, const uint16_t index) :
		item(item),
		index(index) {};
	friend bool operator<(const indexed_item_t &lhs, const indexed_item_t &rhs) {
		return lhs.item < rhs.item;
	}

	T item;
	uint16_t index;
};

} // namespace hub

namespace std {

/**
 * \brief Output stream operator for lists.
 * \param[out] os The output stream.
 * \tparam[in] value_list The list of values which should be streamed to output
 * stream os.
 * \return Reference to output stream os.
 */
template<typename T>
ostream& operator <<(ostream &os, const list<T> &value_list);

/**
 * \brief Input stream operator for lists.
 * \param[in,out] is The input stream.
 * \tparam[out] value_list The list of values which should hold the values from
 * input stream is.
 * \return Reference to input stream is.
 *
 * \sa operator >>(istream &is, list<string> &string_list)
 */
template<typename T>
istream& operator >>(istream &is, list<T> &value_list);

/**
 * \brief Input stream operator for lists of strings.
 * \param[in,out] is The input stream.
 * \tparam[out] value_list The list of values which should hold the values from
 * input stream is.
 * \return Reference to input stream is.
 *
 * \sa operator >>(istream &is, list<T> &value_list)
 */
istream& operator >>(istream &is, list<string> &string_list);

/**
 * \brief Output stream operator for pairs.
 * \param[out] os The output stream.
 * \tparam[in] value_pair The pair of values which should be streamed to output
 * stream os.
 * \return Reference to output stream os.
 */
template<typename T1, typename T2>
ostream& operator <<(ostream &os, const pair<T1, T2> &value_pair);

/**
 * \brief Input stream operator for pairs.
 * \param[in,out] is The input stream.
 * \tparam[out] value_list The list of values which should hold the values from
 * input stream is.
 * \return Reference to input stream is.
 */
template<typename T1, typename T2>
istream& operator >>(istream &is, pair<T1, T2> &value_pair);

/**
 * \brief Output stream operator for vectors.
 * \param[out] os The output stream.
 * \tparam[in] value_vector The vector of values which should be streamed to output
 * stream os.
 * \return Reference to output stream os.
 */
template<class T>
ostream& operator <<(ostream &os, const vector<T> &value_vector);

// ----------------------------------------------------------------------------
// Implementations
// ----------------------------------------------------------------------------
template<typename T>
ostream& operator <<(ostream &os, const list<T> &value_list) {
	typename list<T>::const_iterator it;

	for(it = value_list.begin(); it != value_list.end(); ++it) {
		if( it != value_list.begin() )
			os << " ";
		os << *it;
	}
	return os;
}

template<typename T>
istream& operator >>(istream &is, list<T> &value_list) {
	T value;
	char delim;

	string line;
	if( getline(is, line) ) { //read line
		if( line.empty() )
			return is;

		istringstream line_stream(line);
		while( line_stream.good() ) {
			line_stream >> value;
			value_list.push_back(value);
			is >> delim;
		}
	}

	return is;
}

template<typename T1, typename T2>
ostream& operator <<(ostream &os, const pair<T1, T2> &value_pair) {

	os << value_pair.first << ":" << value_pair.second;
	return os;
}

template<typename T1, typename T2>
istream& operator >>(istream &is, pair<T1, T2> &value_pair) {
	char delim;

	is >> value_pair.first;
	is >> delim;
	is >> value_pair.second;

	return is;
}

inline istream& operator >>(istream &is, list<string> &string_list) {
	istream_iterator<string> begin(is);
	istream_iterator<string> end;
	string_list.assign(begin, end);

	return is;
}

template<class T>
ostream& operator <<(ostream &os, const vector<T> &value_vector) {
	typename vector<T>::const_iterator it;

	for(it = value_vector.begin(); it != value_vector.end(); ++it) {
		if( it != value_vector.begin() )
			os << " ";
		os << setw(6) << setfill(' ') << *it;
	}
	return os;
}



} // namespace std

#endif // _HUB_UTILITY_H_
