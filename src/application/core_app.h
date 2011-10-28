#ifndef _CORE_APP_H_
#define _CORE_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#include "protocol/protocollayer.h"

#include <inttypes.h> //uint8_t

namespace mavhub {

	class CoreApp : public AppLayer<mavlink_message_t> {
		public:
			CoreApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel);
			virtual ~CoreApp();
			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();
			
		private:
			enum MAV_TYPE mav_type;
			int component_id;
			enum MAV_AUTOPILOT_TYPE autopilot;

	};
	
	// ----------------------------------------------------------------------------
	// CoreApp
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif // HAVE_MAVLINK_H

#endif
