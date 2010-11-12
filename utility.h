#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <fcntl.h>
#include <vector>

namespace mavhub {
	// ----------------------------------------------------------------------------
	// Declaration
	// ----------------------------------------------------------------------------
	static int enable_blocking_mode(int fd, bool enabled);
	static timeval& timediff(timeval &diff, const timeval &t1, const timeval &t2);

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
