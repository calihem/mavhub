#ifndef _UI_POTIBOX_H_
#define _UI_POTIBOX_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#include "modulebase.h"

#include "protocol/protocollayer.h"

#include <inttypes.h> //uint8_t
#include <stdlib.h>

namespace mavhub {

	class UI_Potibox : public ModuleBase {
		public:
			UI_Potibox(const std::map<std::string, std::string> args);
			virtual ~UI_Potibox();
			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();

	private:
			int a[6];
			int d[4];
			int16_t a_new[6];
			int16_t d_new[4];
			mavlink_message_t msg_l;
			mavlink_param_set_t param_set;
			mavlink_huch_action_t action;
			virtual void read_conf(const std::map<std::string, std::string> args);

	};
	
	// ----------------------------------------------------------------------------
	// UI_Potibox
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif // HAVE_MAVLINK_H

#endif
