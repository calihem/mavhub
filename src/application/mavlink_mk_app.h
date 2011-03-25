#ifndef _MAVLINK_MK_APP_H_
#define _MAVLINK_MK_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MKLINK_H

#include "protocol/protocollayer.h"
#include <mklink.h>

#include <inttypes.h> //uint8_t
#include <bitset>

namespace mavhub {

	class MAVLinkMKApp : public AppLayer<mavlink_message_t>, public AppLayer<mk_message_t> {
		public:
			static const int component_id = 26;

			MAVLinkMKApp(const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
			virtual ~MAVLinkMKApp();

			virtual void handle_input(const mavlink_message_t &msg);
			virtual void handle_input(const mk_message_t &msg);

		protected:
			virtual void print(std::ostream &os) const;
			virtual void run();

		private:
			/// Time of last received MK message
			uint64_t message_time;
			/// Time difference since last heartbeat
			uint64_t delta_time;
			/// TX buffer for mavlink messages
			mavlink_message_t tx_mav_msg;
			/// Mutex to protect tx_mav_msg
			pthread_mutex_t tx_mav_mutex;
			/// TX buffer for MK messages
			mavlink_message_t tx_mk_msg;
			/// Mutex to protect tx_mk_msg
			pthread_mutex_t tx_mk_mutex;

			void send_heartbeat();
	};

} // namespace mavhub

#endif // HAVE_MKLINK_H
#endif // _MAVLINK_MK_APP_H_
