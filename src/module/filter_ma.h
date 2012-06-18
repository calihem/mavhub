// moving average Filter

#ifndef _FILTER_MA_H_
#define _FILTER_MA_H_

#include <vector>

namespace mavhub {
	class MA {
	public:
		/// set up MA filter with given order (number of taps) and coefficients
		//MA(int order_, std::vector<double> &coeff);
		//MA(int order_, double* coeff);
		MA(int order_, int val);
		virtual ~MA();

		/// calculate next sample
		virtual int calc(int val);
	private:
		/// order
		int order;
		/// input data
		std::vector<int> x;
		/// current sum
		int running_sum;
		/// coefficients
		// std::vector<double> c;
	};
}

#endif
