#ifndef _PP_USS_H_
#define _PP_USS_H_

#include "pp.h"
#include "filter_fir.h"

namespace mavhub {
	class PreProcessorUSS : public PreProcessor {
	public:
		PreProcessorUSS();
		virtual ~PreProcessorUSS();
		virtual std::pair<double, int> calc(int chan, int s);
		virtual void calc(std::vector< std::pair< double, int > > &pre, int chan, int s);
	private:
		//std::vector<double> filt_valid_coef;
		//FIR* filt_valid;
	};
}
#endif
