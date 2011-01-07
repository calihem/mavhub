
#include "pp.h"

namespace mavhub {
	PreProcessor::PreProcessor() {
		sig.reserve(16);
	}

	PreProcessor::~PreProcessor() {
	}

	double PreProcessor::mkacc2mm(double acc) {
		return acc * MKACC2MM;
	}
	// is abstract
	// double PreProcessor::calc(int s) {
	// 	// Logger::log("PreProcessor::calc", Logger::LOGLEVEL_INFO);
	// 	return static_cast<double>(s);
	// }
}
