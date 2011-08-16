#ifndef _HUB_GSTREAMER_VIDEO_CLIENT_H_
#define _HUB_GSTREAMER_VIDEO_CLIENT_H_

#include <gst/gst.h>

#include <string>

namespace hub {
namespace gstreamer {

class VideoServer;

class VideoClient {
	public:
		VideoClient() {};
		/**
		 * \brief Handle 3 channel video data.
		 */
		virtual void handle_video_data(const unsigned char *data, const int width, const int height) = 0;

	protected:
		VideoServer *server;

	private:
};

} // namespace gstreamer
} // namespace hub

#endif
