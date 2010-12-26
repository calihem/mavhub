#include "pp_uss.h"

namespace mavhub {
	// PreProcessorUSS::PreProcessorUSS() {
	// 	sig.reserve(16);
	// }

	// PreProcessorUSS::~PreProcessorUSS() {
	// }

	std::pair<double, int> PreProcessorUSS::calc(int chan, int s) {
		std::pair<double, int> p;
		// Logger::log("PreProcessorUSS::calc", chan, s, Logger::LOGLEVEL_INFO);
		p.first = US2MM(static_cast<double>(s));
		p.second = 1;
		return p;
	}
}
