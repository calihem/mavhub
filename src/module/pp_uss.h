#ifndef _PP_USS_H_
#define _PP_USS_H_

#include "pp.h"

namespace mavhub {
	class PreProcessorUSS : public PreProcessor {
	public:
		virtual std::pair<double, int> calc(int chan, int s);
	};
}
#endif
