#ifndef _TESTCORE_H_
#define _TESTCORE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#include "protocol/protocollayer.h"

#include <inttypes.h> //uint8_t

namespace mavhub {

	class TestCore : public AppLayer<mavlink_message_t> {
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

#endif // HAVE_MAVLINK_H

#endif
