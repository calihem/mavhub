#ifndef _OPENGL_APP_H_
#define _OPENGL_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_GL_GLUT_H

#include "protocol/protocollayer.h"

#include <inttypes.h> //uint8_t
#include <vector>

namespace mavhub {

	class OpenGLApp : public AppLayer<mavlink_message_t> {
		public:
			static const int component_id = 27;

			OpenGLApp(const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
			virtual ~OpenGLApp();

			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void print(std::ostream &os) const;
			virtual void run();

		private:
			/// Time difference since last heartbeat
// 			uint64_t delta_time;
			/// TX buffer for mavlink messages
			mavlink_message_t tx_mav_msg;
			/// Mutex to protect tx_mav_msg
			pthread_mutex_t tx_mav_mutex;
			/// Vector of texture IDs
			std::vector<unsigned int> textures;

			void send_heartbeat();
	};

} // namespace mavhub

#endif // HAVE_GL_GLUT_H
#endif // _OPENGL_APP_H_
