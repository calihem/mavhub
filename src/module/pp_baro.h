#ifndef _PP_BARO_H_
#define _PP_BARO_H_

#include "pp.h"
#include "filter_fir.h"

namespace mavhub {
	class PreProcessorBARO : public PreProcessor {
	public:
		PreProcessorBARO();
		virtual ~PreProcessorBARO();
		virtual std::pair<double, int> calc(int chan, int s);
		virtual void calc(std::vector< std::pair< double, int > > &pre, int chan, int s);
	private:
		std::vector<double> filt_c;
		FIR* filt;
	};
}
#endif
