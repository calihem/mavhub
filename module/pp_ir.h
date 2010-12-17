#ifndef _PP_IR_H_
#define _PP_IR_H_

#include "pp.h"

namespace mavhub {
	class PreProcessorIR : public PreProcessor {
	public:
		PreProcessorIR(double b_, double m_, double k_);

		virtual std::pair<double, int> calc(int chan, int s);
	private:
		/// linearisation coefficients
		double b;
		double m;
		double k;
		virtual double sens_lin_ir(double raw);
	};
}
#endif
