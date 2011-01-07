#ifndef _PP_IR_H_
#define _PP_IR_H_

#include "pp.h"
#include "filter_fir.h"

namespace mavhub {
	class PreProcessorIR : public PreProcessor {
	public:
		PreProcessorIR(double llim_, double ulim_, double b_, double m_, double k_);

		virtual std::pair<double, int> calc(int chan, int s);
		virtual void calc(std::vector< std::pair< double, int > > &pre, int chan, int s);
	private:
		/// linearisation coefficients
		double b;
		double m;
		double k;
		double llim, ulim;
		FIR* filt_valid;
		virtual double sens_lin_ir(double raw);
	};
}
#endif
