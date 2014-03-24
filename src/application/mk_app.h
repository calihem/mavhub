#ifndef _MK_APP_H_
#define _MK_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#ifdef HAVE_MKHUCHLINK_H
#include <mkhuchlink.h>

#include "protocol/protocollayer.h"

#include <inttypes.h> //uint8_t

namespace mavhub {

	class MKApp : public AppLayer<mavlink_message_t>, public AppLayer<mkhuch_message_t> {
		public:
			static const int component_id = 25;

			MKApp(const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
			virtual ~MKApp();

			virtual void handle_input(const mavlink_message_t &msg);
			virtual void handle_input(const mkhuch_message_t &msg);

		protected:
			virtual void print(std::ostream &os) const;
			virtual void run();

		private:
			static const int parameter_count = 104;
			static const int8_t parameter_ids[parameter_count][15];

			/// tx buffer for mavlink messages
// 			mavlink_message_t tx_mav_msg;
			/// Mutex to protect tx_mav_msg
// 			pthread_mutex_t tx_mav_mutex;
			uint8_t parameters[parameter_count];
			/// Serial port to MK
// 			UART mk_dev;
			/// tx buffer for MK messages
// 			mkhuch_message_t tx_mk_msg;
			/// Mutex to protect tx_mk_msg
// 			pthread_mutex_t tx_mk_mutex;
			/// Time of last received mkhuch message
// 			uint64_t message_time;
			/// Last received attitude
// 			mkhuch_attitude_t attitude;
			/// Time of last received attitude
// 			uint64_t attitude_time;
			/// Last requested parameter from MK
// 			uint8_t parameter_request;
			/// Time of last parameter answer from MK
// 			uint64_t parameter_time;

// 			using AppLayer::send;
// 			void send_heartbeat();
// 			void send_mavlink_param_value(const mk_param_type_t param_type);
// 			const uint8_t get_parameter(const mk_param_type_t param_type) const;
// 			const int8_t* get_parameter_id(const mk_param_type_t param_type) const;
// 			const int parameter_id_to_index(const int8_t *parameter_id);
			/// Send mkhuch message over MK device
// 			size_t send(const mkhuch_message_t& msg);
			/// Send mkhuch data over MK device
// 			size_t send(const mkhuch_msg_type_t type, const void *data, const uint8_t size);
			/// Method to handle input from MK
// 			void handle_input(const mkhuch_message_t& msg);
	};

	// ----------------------------------------------------------------------------
	// MKApp
	// ----------------------------------------------------------------------------
// 	inline const uint8_t MKApp::get_parameter(const mk_param_type_t param_type) const {
// 		return parameters[param_type];
// 	}
// 	inline const int8_t* MKApp::get_parameter_id(const mk_param_type_t param_type) const {
// 		return parameter_ids[param_type];
// 	}
} // namespace mavhub

#endif // HAVE_MKHUCHLINK_H
#endif // HAVE_MAVLINK_H
#endif // _MK_APP_H_
