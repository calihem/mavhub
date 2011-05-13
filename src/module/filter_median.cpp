#include "filter_median.h"
#include "core/logger.h"


using namespace std;

namespace mavhub {
	// MEDIAN::MEDIAN(int order_, vector<double> &coeff) {
	MEDIAN::MEDIAN(int order_) : 
		index(0),
		indexm1(0),
		x(order_),
		x1(order_)
	{
		order = order_;
		orderhalf = order / 2;
		parity = order % 2;
		//x.reserve(order);
		//c.reserve(order);
		//x.assign(order, 0.0);
		//c.assign(coeff, coeff+order);
		// for(int i = 0; i < order; i++) {
		// 	//x[i] = 0.0;
		// 	c[i] = coeff[i];
		// }
		Logger::log("MEDIAN: ctor: order, order/2", order, orderhalf, Logger::LOGLEVEL_INFO);
		Logger::log("MEDIAN: ctor: parity", parity, Logger::LOGLEVEL_INFO);
	}

	MEDIAN::~MEDIAN() {
		
	}

	double MEDIAN::calc(double val) {
		// int i;
		double y;
		y = 0.0;

		indexm1 = index; // save index to last value, but what for?
		index = (index + 1) % order;
		x[index] = val; // update with new value
		x1 = x; // copy vector prior to sorting
		sort(x1.begin(), x1.end()); // sorted
		// window size parity check
		if(!parity)
			y = 0.5 * (x1[orderhalf] + x1[orderhalf+1]);
		else
			y = x1[orderhalf];

		// Logger::log("XXX: filter_median::calc::y", y, Logger::LOGLEVEL_INFO);
		return y;
	}
}
