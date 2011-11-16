#include "filter_smooth.h"
#include "core/logger.h"


using namespace std;

namespace mavhub {
	// Smooth::SMOOTH(int order_, vector<double> &coeff) {
	Smooth::Smooth(int order_, double* coeff) {
		order = order_;
		//x.reserve(order);
		//c.reserve(order);
		x.assign(order, 0.0);
		c.assign(coeff, coeff+order);
		// for(int i = 0; i < order; i++) {
		// 	//x[i] = 0.0;
		// 	c[i] = coeff[i];
		// }
	}

	Smooth::~Smooth() {
		
	}

	double Smooth::calc(double val) {
		int i;
		double y;
		y = 0.0;
		// values in x: index 0 is current value, 1 last value, 2 last before last ...
		// ring buffer
		for(i = order-1; i > 0; i--) {
			x[i] = x[i-1];
		}
		x[0] = val;
		for(i = 0; i < order; i++) {
			// Logger::log("XXX: filter_smooth::calc::x,c", x[i], c[i], Logger::LOGLEVEL_INFO);
			y += x[i] * c[i];
		}
		//Logger::log("XXX: filter_smooth::calc::y", y, Logger::LOGLEVEL_INFO);
		return y;
	}
}
