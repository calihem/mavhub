#ifndef _MKRCMOD_H_
#define _MKRCMOD_H_

#include <inttypes.h> //uint8_t
#include "protocollayer.h"

namespace mavhub {

	class MKRCModule : public AppLayer {
		public:
			static const int component_id = 25;

			MKRCModule();
			virtual ~MKRCModule();

			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();

		private:
			mavlink_message_t tx_msg;
	};
	
	// ----------------------------------------------------------------------------
	// MKRCModule
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif
