#include "pp_baro.h"

namespace mavhub {
	// PreProcessorBARO::PreProcessorBARO() {
	// 	sig.reserve(16);
	// }

	// PreProcessorBARO::~PreProcessorBARO() {
	// }

	std::pair<double, int> PreProcessorBARO::calc(int chan, int s) {
		std::pair<double, int> p;
		// Logger::log("PreProcessorBARO::calc", chan, s, Logger::LOGLEVEL_INFO);
		p.first = static_cast<double>(s);
		p.second = 1;
		return p;
	}
}
