#include "pp_baro.h"

namespace mavhub {
	PreProcessorBARO::PreProcessorBARO() {
		//sig.reserve(16);
		double tmp_c[] = {0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.05};
		filt = new FIR(11, tmp_c);
	}

	PreProcessorBARO::~PreProcessorBARO() {
	}

	std::pair<double, int> PreProcessorBARO::calc(int chan, int s) {
		std::pair<double, int> p;
		// Logger::log("PreProcessorBARO::calc", chan, s, Logger::LOGLEVEL_INFO);
		p.first = static_cast<double>(s);
		p.second = 1;
		return p;
	}

	void PreProcessorBARO::calc(std::vector< std::pair< double, int > > &pre, int chan, int s) {
		// std::pair<double, int> p;
		// Logger::log("PreProcessorUSS::calc", chan, s, Logger::LOGLEVEL_INFO);
		pre[chan].first = filt->calc(static_cast<double>(s));
		pre[chan].second = 1;
		return;
	}

}
