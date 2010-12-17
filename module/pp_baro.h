#ifndef _PP_BARO_H_
#define _PP_BARO_H_

#include "pp.h"

namespace mavhub {
	class PreProcessorBARO : public PreProcessor {
	public:
		virtual std::pair<double, int> calc(int chan, int s);
	};
}
#endif
