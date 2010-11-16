#ifndef _TESTCORE_H_
#define _TESTCORE_H_

#include <inttypes.h> //uint8_t
#include "protocollayer.h"

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
