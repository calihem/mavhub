#include "pp_uss.h"

namespace mavhub {
	PreProcessorUSS::PreProcessorUSS() {
		// sig.reserve(16);
		// double tmp_c[] = {0.25, 0.25, 0.25, 0.25};
		//double tmp_c[] = {0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.05};
		// double tmp_c[] = {0.125, 0.25, 0.25, 0.25, 0.125};
		//filt_valid = new FIR(11, tmp_c);
	}

	PreProcessorUSS::~PreProcessorUSS() {
		//delete filt_valid;
	}

	std::pair<double, int> PreProcessorUSS::calc(int chan, int s) {
		std::pair<double, int> p;
		// Logger::log("PreProcessorUSS::calc", chan, s, Logger::LOGLEVEL_INFO);
		p.first = US2MM(static_cast<double>(s));
		// check validity
		if(in_range(p.first, 100.0, 2000.0))
			p.second = 1;
		else
			p.second = 0;
		
		// p.second = 1;
		return p;
	}

	void PreProcessorUSS::calc(std::vector< std::pair< double, int > > &pre, int chan, int s) {
		// std::pair<double, int> p;
		//double tmp_valid;
		//double tmp_valid_filt;
		// Logger::log("PreProcessorUSS::calc", chan, s, Logger::LOGLEVEL_INFO);
		pre[chan].first = US2MM(static_cast<double>(s));
		// valid check gone back to ctlr_hover
		//Logger::log("XXX: filter_fir::calc::y", tmp_valid_filt, pre[chan].second, Logger::LOGLEVEL_INFO);
		// pre[chan].second = tmp_valid_filt * 10;
		// pre[chan].second = tmp_valid;
		return;
	}
}
