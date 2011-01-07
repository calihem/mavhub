#ifndef _PP_ACC_H_
#define _PP_ACC_H_

#include "pp.h"

namespace mavhub {
	class PreProcessorACC : public PreProcessor {
	public:
		virtual std::pair<double, int> calc(int chan, int s);
		virtual void calc(std::vector< std::pair< double, int > > &pre, int chan, int s);
	};
}
#endif
