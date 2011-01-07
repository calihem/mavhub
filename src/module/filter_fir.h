// FIR Filter

#ifndef _FILTER_FIR_H_
#define _FILTER_FIR_H_

#include <vector>

namespace mavhub {
	class FIR {
	public:
		/// set up FIR filter with given order (number of taps) and coefficients
		//FIR(int order_, std::vector<double> &coeff);
		FIR(int order_, double* coeff);
		virtual ~FIR();

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
