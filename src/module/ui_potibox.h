#ifndef _UI_POTIBOX_H_
#define _UI_POTIBOX_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#include "protocol/protocollayer.h"

#include <inttypes.h> //uint8_t

namespace mavhub {

	class UI_Potibox : public AppLayer<mavlink_message_t> {
		public:
			UI_Potibox();
			virtual ~UI_Potibox();
			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();

	};
	
	// ----------------------------------------------------------------------------
	// UI_Potibox
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif // HAVE_MAVLINK_H

#endif
