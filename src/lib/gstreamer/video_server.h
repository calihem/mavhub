#ifndef _HUB_GSTREAMER_VIDEO_SERVER_H_
#define _HUB_GSTREAMER_VIDEO_SERVER_H_

#include <gst/gst.h>

#include <string>

// FIXME: mv thread.h to lib directory
#include "core/thread.h"

namespace hub {
namespace gstreamer {

class VideoServer : public cpp_pthread::PThread {
	public:
		VideoServer(int *argc, char **argv, const std::string &pipeline_description);

	protected:
		/**
		 * \copydoc cpp_pthread::PThread::run()
		 *
		 * The run method ...
		 */
		virtual void run();

	private:
		static GMainLoop *loop;
		GstElement *pipeline;
		
		static gboolean bus_call(GstBus *bus, GstMessage *msg, void *user_data);
};

} // namespace gstreamer
} // namespace hub

#endif
