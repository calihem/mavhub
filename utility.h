#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <fcntl.h>
#include <list>
#include <vector>
#include <iomanip> //setw
#include <sstream> //stringstream
#include <iterator> //istream_iterator

namespace mavhub {
	// ----------------------------------------------------------------------------
	// Declaration
	// ----------------------------------------------------------------------------
	static int enable_blocking_mode(int fd, bool enabled);
	static timeval& timediff(timeval &diff, const timeval &t1, const timeval &t2);
	std::istream& operator >>(std::istream &is, std::list<std::string> &string_list);

	// ----------------------------------------------------------------------------
	// Implementation
	// ----------------------------------------------------------------------------
	inline int enable_blocking_mode(int fd, bool enabled) {
		int mode = fcntl(fd, F_GETFL, 0);
		
		if(enabled) {
			mode &= ~O_NONBLOCK;
		} else {
			mode |= O_NONBLOCK;
		}
		return fcntl(fd, F_SETFL, mode);
	}

	inline timeval& timediff(timeval &diff, const timeval &t1, const timeval &t2) {
		if(t2.tv_usec < t1.tv_usec) {
			diff.tv_usec = t1.tv_usec - t2.tv_usec;
			diff.tv_sec = t2.tv_sec - t1.tv_sec - 1;
		} else {
			diff.tv_usec = t2.tv_usec - t1.tv_usec;
			diff.tv_sec = t2.tv_sec - t1.tv_sec;
		}

		return diff;
	}

	template <typename T>
	std::ostream& operator <<(std::ostream &os, const std::list<T> &value_list) {
		typename std::list<T>::const_iterator it;

		for(it=value_list.begin(); it != value_list.end(); ++it) {
			if(it != value_list.begin())
				os << " ";
			os << *it;
		}
		return os;
	}

	template <typename T>
	std::istream& operator >>(std::istream &is, std::list<T> &value_list) {
		T value;
		char delim;

		std::string line;
		if( std::getline(is, line) ) { //read line
			if(line.empty())
				return is;

			std::istringstream line_stream(line);
			while(line_stream.good()) {
				line_stream >> value;
				value_list.push_back(value);
				is >> delim;
			}
		}

		return is;
	}

	inline std::istream& operator >>(std::istream &is, std::list<std::string> &string_list) {
		std::istream_iterator<std::string> begin(is);
		std::istream_iterator<std::string> end;
		string_list.assign(begin, end);

		return is;
	}

	template <class T>
	std::ostream& operator <<(std::ostream &os, const std::vector<T> &value_vector) {
		typename std::vector<T>::const_iterator it;

		for(it=value_vector.begin(); it != value_vector.end(); ++it) {
			if(it != value_vector.begin())
				os << " ";
			os << std::setw(6) << std::setfill(' ') << *it;
		}
		return os;
	}

} // namespace mavhub

#endif
