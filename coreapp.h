#ifndef _COREAPP_H_
#define _COREAPP_H_

#include <inttypes.h> //uint8_t
#include "protocollayer.h"

namespace mavhub {

	class CoreApp : public AppLayer {
		public:
			CoreApp();
			virtual ~CoreApp();
			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();

	};
	
	// ----------------------------------------------------------------------------
	// CoreApp
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif
