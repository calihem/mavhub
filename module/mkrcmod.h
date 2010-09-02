#ifndef _MKRCMOD_H_
#define _MKRCMOD_H_

#include <inttypes.h> //uint8_t
#include "protocollayer.h"

namespace mavhub {

	class MKRCModule : public AppLayer {
		public:
			MKRCModule();
			virtual ~MKRCModule();
			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();

	};
	
	// ----------------------------------------------------------------------------
	// MKRCModule
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif
