#include "pp_acc.h"

namespace mavhub {
	// PreProcessorACC::PreProcessorACC() {
	// 	sig.reserve(16);
	// }

	// PreProcessorACC::~PreProcessorACC() {
	// }

	std::pair<double, int> PreProcessorACC::calc(int chan, int s) {
		std::pair<double, int> p;
		//Logger::log("PreProcessorACC::calc", chan, s, Logger::LOGLEVEL_INFO);
		p.first = static_cast<double>(s) * MKACC2MM;
		p.second = 1;
		return p;
	}

	void PreProcessorACC::calc(std::vector< std::pair< double, int > > &pre, int chan, int s) {
		// std::pair<double, int> p;
		// Logger::log("PreProcessorUSS::calc", chan, s, Logger::LOGLEVEL_INFO);
		pre[chan].first = static_cast<double>(s) * MKACC2MM;
		pre[chan].second = 1;
		return;
	}

}
