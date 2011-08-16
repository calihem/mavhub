#ifndef _HUB_GSTREAMER_VIDEO_SERVER_H_
#define _HUB_GSTREAMER_VIDEO_SERVER_H_

#include <gst/gst.h>

#include <string>
#include <map>

// FIXME: mv thread.h to lib directory
#include "core/thread.h"
#include "video_client.h"

namespace hub {
namespace gstreamer {

class VideoServer : public cpp_pthread::PThread {
	public:
		VideoServer(int *argc, char **argv, const std::string &pipeline_description);

		/**
		 * \brief Connect client with the source of the given element.
		 *
		 * If an element with the given name is available, the client will be connected
		 * to the "new_buffer" signal of this element. For every client only one binding
		 * is allowed.
		 * \sa \release
		 */
		int bind2appsink(VideoClient* client, const std::string &element);

		/**
		 * \brief Remove binding between element and client.
		 */
		void release(const VideoClient *client);

		/**
		 * \brief Get element from pipeline by name
		 * 
		 * You have to call function gst_object_unref(gpointer object) afterwards.
		 */
		GstElement* element(const std::string &name) const;

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
		static std::map<VideoClient*, GstElement*> client_sink_map;
		
		static gboolean bus_call(GstBus *bus, GstMessage *msg, void *user_data);
		static void new_video_buffer_callback(GstElement *element, GstElement *data);
};

} // namespace gstreamer
} // namespace hub

#endif
