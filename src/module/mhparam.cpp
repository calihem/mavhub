#include "mhparam.h"

using namespace std;

namespace mavhub {
	Mhparam::Mhparam(double value_) : value(value_) {
	}
	Mhparam::~Mhparam() {}

	void Mhparam::pset(double value_) {
		value = value_;
	}

	double Mhparam::pget() {
		return value;
	}
}
