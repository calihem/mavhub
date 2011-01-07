#include "pp_ir.h"

namespace mavhub {
	PreProcessorIR::PreProcessorIR(double llim_, double ulim_, double b_, double m_, double k_) {
		b = b_;
		m = m_;
		k = k_;
		llim = llim_;
		ulim = ulim_;
		double tmp_c[] = {0.125, 0.25, 0.25, 0.25, 0.125};
		filt_valid = new FIR(5, tmp_c);
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

	void PreProcessorIR::calc(std::vector< std::pair< double, int > > &pre, int chan, int s) {
		// std::pair<double, int> p;
		// Logger::log("PreProcessorUSS::calc", chan, s, Logger::LOGLEVEL_INFO);
		pre[chan].first = sens_lin_ir(static_cast<double>(s));
		pre[chan].second = 0;
		return;
	}

	double PreProcessorIR::sens_lin_ir(double raw) {
		return((1.0 / (m * raw + b)) - k);
	}
}
