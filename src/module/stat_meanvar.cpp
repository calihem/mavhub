#include "stat_meanvar.h"
#include "core/logger.h"

#include <math.h>

using namespace std;

namespace mavhub {
	Stat_MeanVar::Stat_MeanVar(int order_) : 
		order(order_),
		index(0),
		x(order, 0)
	{
	}

	Stat_MeanVar::~Stat_MeanVar() {
	}

	void Stat_MeanVar::update(double x_) {
		int i;
		double tmp = 0.0;
		x[index] = x_;
		index = (index + 1) % order;
		// calculate mean
		for(i = 0; i < order; i++) {
			tmp += x[i];
		}
		mean = tmp/(double)order;
		// calculate variance
		tmp = 0.0;
		for(i = 0; i < order; i++) {
			tmp += pow(mean - x[i], 2);
		}
		var = tmp/(double)order;
	}

	void Stat_MeanVar::update_heur(double x_) {
		var_e = x_;
	}

	double Stat_MeanVar::get_mean() {
		return mean;
	}

	double Stat_MeanVar::get_var() {
		return var;
	}

	double Stat_MeanVar::get_var_e() {
		return var_e;
	}
}
