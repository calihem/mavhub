// moving average filter

#include "filter_ma.h"
#include "core/logger.h"


using namespace std;

namespace mavhub {
	// MA::MA(int order_, vector<double> &coeff) {
	MA::MA(int order_, int val_) {
		order = order_;
		running_sum = val_;
		//x.reserve(order);
		//c.reserve(order);
		x.assign(order, 0);
		// c.assign(coeff, coeff+order);
		// for(int i = 0; i < order; i++) {
		// 	//x[i] = 0.0;
		// 	c[i] = coeff[i];
		// }
	}

	MA::~MA() {
		
	}

	int MA::calc(int val) {
		int i;
		// int y;
		// y = 0;
		// values in x: index 0 is current value, 1 last value, 2 last before last ...

		running_sum -= x[order-1];

		// ring buffer
		for(i = order-1; i > 0; i--) {
			x[i] = x[i-1];
		}
		x[0] = val;

		// for(i = 0; i < order; i++) {
		// 	// Logger::log("XXX: filter_ma::calc::x,c", x[i], c[i], Logger::LOGLEVEL_INFO);
		// 	y += x[i] * c[i];
		// }
		
		running_sum += x[0];

		//Logger::log("XXX: filter_ma::calc::y", y, Logger::LOGLEVEL_INFO);
		// return y;
		return(running_sum);
	}
}
