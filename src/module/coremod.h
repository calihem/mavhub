#ifndef _COREMOD_H_
#define _COREMOD_H_

#include "protocol/protocollayer.h"

#include <inttypes.h> //uint8_t

namespace mavhub {

	class CoreModule : public AppLayer<mavlink_message_t> {
		public:
			CoreModule();
			virtual ~CoreModule();
			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();

	};
	
	// ----------------------------------------------------------------------------
	// CoreModule
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif
