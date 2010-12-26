#ifndef _TESTCORE_H_
#define _TESTCORE_H_

#include "core/protocollayer.h"

#include <inttypes.h> //uint8_t

namespace mavhub {

	class TestCore : public AppLayer {
		public:
			TestCore();
			virtual ~TestCore();
			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();

	};
	
	// ----------------------------------------------------------------------------
	// TestCore
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif
