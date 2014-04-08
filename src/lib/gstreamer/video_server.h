#ifndef _HUB_GSTREAMER_VIDEO_SERVER_H_
#define _HUB_GSTREAMER_VIDEO_SERVER_H_

#include <gst/gst.h>

// FIXME: check dependency
#include <gst/app/gstappsrc.h>

#include <string>
#include <list>
#include <map>
#include <vector>
#include <stdexcept>    // out_of_range exception

#include "lib/hub/thread.h"
#include "video_client.h"

namespace hub {
namespace gstreamer {

class VideoServer : public hub::PThread {
	public:
		VideoServer(int *argc, char **argv, const std::string &pipeline_description);
		VideoServer(int *argc, char **argv, const std::list<std::string> &pipeline_descriptions);

		/**
		 * \return Pipeline ID.
		 * \retval negative Error occured.
		 * \retval positive Successfully created pipeline with this ID.
		 */
		int add_pipeline(const std::string &pipeline_description);

		/**
		 * \brief Connect client with the source of the given element.
		 *
		 * If an element with the given name is available, the client will be connected
		 * to the "new_buffer" signal of this element. For every client only one binding
		 * is supported.
		 * \param client Pointer to client which wants to receive multimedia stream.
		 * \param element Name of the gstreamer element to connect to.
		 * \param pipeline_id ID of pipeline containing the element.
		 * \sa \release
		 */
		int bind2appsink(VideoClient* client, const std::string &element, const int pipeline_id = 0);

		/**
		 * \brief Remove binding between element and client.
		 */
		void release(const VideoClient *client);

		/**
		 * \brief Get element from pipeline by name
		 * 
		 * You have to call function gst_object_unref(gpointer object) afterwards.
		 */
		GstElement* element(const std::string &name, const int pipeline_id = 0) const;

		void print_elements() const;
		
		/**
		 * \brief Push data to an appsrc element.
		 * \param appsrc
		 * \param data
		 * \param width
		 * \param height
		 * \param bpp Bits per pixel.
		 */
		int push(GstAppSrc *appsrc, unsigned char *data, const int width, const int height, const int bpp);

	protected:
		/**
		 * \copydoc hub::PThread::run()
		 *
		 * The run method ...
		 */
		virtual void run();

	private:
		static GMainLoop *loop;
		std::vector<GstElement*> pipeline_vector;
// 		GstElement *pipeline;
		static std::multimap<VideoClient*, GstElement*> client_sink_map;
		
		static gboolean bus_call(GstBus *bus, GstMessage *msg, void *user_data);
#if GST_VERSION_MAJOR == 0 && GST_VERSION_MINOR == 10
		static void new_video_buffer_callback(GstElement *element, GstElement *data);
#elif GST_VERSION_MAJOR == 1
		static GstFlowReturn new_video_buffer_callback(GstElement *element, GstElement *data);
#endif
		/**
		 * \brief Get pipeline with given ID.
		 */
		GstElement* pipeline(const int id = 0) const;
};

inline GstElement* VideoServer::pipeline(const int id) const {
	try {
		return pipeline_vector.at(id);
	}
	catch(std::out_of_range &e) {
		return NULL;
	} 
}

} // namespace gstreamer
} // namespace hub

#endif
