#include "pp_ir.h"

namespace mavhub {
	PreProcessorIR::PreProcessorIR(double b_, double m_, double k_) {
		b = b_;
		m = m_;
		k = k_;
	}

	// PreProcessorIR::~PreProcessorIR() {
	// }

	std::pair<double, int> PreProcessorIR::calc(int chan, int s) {
		std::pair<double, int> p;
		// Logger::log("PreProcessorIR::calc", chan, s, Logger::LOGLEVEL_INFO);
		p.first = sens_lin_ir(static_cast<double>(s));
		p.second = 0;
		return p;
	}

	double PreProcessorIR::sens_lin_ir(double raw) {
		return((1.0 / (m * raw + b)) - k);
	}
}
