#ifndef _SLAM_APP_H_
#define _SLAM_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_GSTREAMER
#include "lib/gstreamer/video_server.h"
#include "lib/gstreamer/video_client.h"

#include "protocol/protocollayer.h"

#include <inttypes.h> //uint8_t

namespace mavhub {

	class SLAMApp : public AppLayer<mavlink_message_t>
		, public hub::gstreamer::VideoClient
	{
		public:
			static const int component_id = 28;

			SLAMApp(const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
			virtual ~SLAMApp();

			virtual void handle_input(const mavlink_message_t &msg);
			virtual void handle_video_data(const unsigned char *data, const int width, const int height, const int bpp);

		protected:
			virtual void print(std::ostream &os) const;
			virtual void run();

		private:
			static void new_video_buffer_callback(GstElement *element, GstElement *data);
	};

} // namespace mavhub

#endif // HAVE_GSTREAMER

#endif // _OPENGL_APP_H_
