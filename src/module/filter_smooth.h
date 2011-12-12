// Smooth Filter
// wrapper for opencv smooth

#ifndef _FILTER_SMOOTH_H_
#define _FILTER_SMOOTH_H_

#include <vector>

namespace mavhub {
	class Smooth {
	public:
		/// set up SMOOTH filter with given order (number of taps) and coefficients
		//SMOOTH(int order_, std::vector<double> &coeff);
		Smooth(int order_, double* coeff);
		virtual ~Smooth();

		/// calculate next sample
		virtual double calc(double val);
	private:
		/// order
		int order;
		/// input data
		std::vector<double> x;
		/// coefficients
		std::vector<double> c;
	};
}

#endif
