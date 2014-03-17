// Median Filter

#ifndef _FILTER_MEDIAN_H_
#define _FILTER_MEDIAN_H_

//#define WINSIZE 9
//#define WINSIZE_HALF WINSIZE/2


#include <algorithm>
#include <vector>
#include <math.h>

namespace mavhub {
	class MEDIAN {
	public:
		/// set up MEDIAN filter with given order (number of taps) and coefficients
		//MEDIAN(int order_, std::vector<double> &coeff);
		MEDIAN(int order_);
		virtual ~MEDIAN();

		/// calculate next sample
		virtual double calc(double val);
	private:
		/// order
		int order;
		/// order half
		int orderhalf;
		/// order odd/even
		int parity;
		/// index
		int index;
		/// index last
		int indexm1;
		/// input data and copy for sorting
		std::vector<double> x, x1;
		/// coefficients
		// std::vector<double> c;
	};
}

#endif
