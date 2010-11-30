#ifndef _MK_APP_H_
#define _MK_APP_H_

#include <inttypes.h> //uint8_t
#include "protocollayer.h"

namespace mavhub {

	class MKApp : public AppLayer {
		public:
			static const int component_id = 25;

			MKApp(const Logger::log_level_t loglevel);
			virtual ~MKApp();

			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();

		private:
			typedef enum {
				USERPARAM_1 = 0,
				USERPARAM_2 = 1,
				USERPARAM_3 = 2,
				USERPARAM_4 = 3,
				USERPARAM_5 = 4,
				USERPARAM_6 = 5,
				USERPARAM_7 = 6,
				USERPARAM_8 = 7,
				EXTERN_CONTROL = 8
			} mk_param_type_t;
			static const int parameter_count = 9;
			static const int8_t parameter_ids[parameter_count][15];

			uint32_t state_vector;
			/// tx buffer 
			mavlink_message_t tx_msg;
			/// Mutex to protect tx_msg
			pthread_mutex_t tx_mutex;

			uint8_t parameters[parameter_count];

			void handle_statuses();
			void send_heartbeat();
			const uint8_t get_parameter(const mk_param_type_t param_type) const;
			const int8_t* get_parameter_id(const mk_param_type_t param_type) const;
			void send_mavlink_param_value(const mk_param_type_t param_type);
			const int parameter_id_to_index(const int8_t *parameter_id);
	};
	
	// ----------------------------------------------------------------------------
	// MKApp
	// ----------------------------------------------------------------------------
	inline const uint8_t MKApp::get_parameter(const mk_param_type_t param_type) const {
		return parameters[param_type];
	}
	inline const int8_t* MKApp::get_parameter_id(const mk_param_type_t param_type) const {
		return parameter_ids[param_type];
	}

} // namespace mavhub

#endif
