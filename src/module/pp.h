#ifndef _PP_H_
#define _PP_H_


#include "core/logger.h"
/* #include "protocollayer.h" */

#include <vector>

#define MKGYRO2RAD 1.52634e-05
#define MKACC2MM   47.9 // (mm), 0.0479 (m) //0.0048828 (volts) // 0.0009765625
#define US2MM(us) us * 0.1715;

namespace mavhub {
	class PreProcessor {
	public:
		PreProcessor();
		virtual ~PreProcessor();
		virtual std::pair<double, int> calc(int chan, int s) = 0;

	private:
		std::vector<int> sig;
		virtual double mkacc2mm(double acc);
	};
}

#endif
