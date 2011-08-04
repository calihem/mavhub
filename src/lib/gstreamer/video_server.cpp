#include "video_server.h"

#include <iostream>	//cout
#include <fstream>	//ifstream
#include <stdexcept>

#include <gst/gst.h>
// #include <gst/app/gstappsink.h>
// #include <gst/app/gstappbuffer.h>

namespace hub {
namespace gstreamer {

#define VERBOSE 1

// little debug makro
#if VERBOSE >= 1
#define dout if(1) cout
#else
#define dout if(0) cout
#endif

using namespace std;

GMainLoop *VideoServer::loop = NULL;

VideoServer::VideoServer(int *argc, char **argv, const std::string &pipeline_description) : 
	pipeline(NULL) {

	gst_init(argc, &argv);
	
	pipeline = gst_parse_launch(pipeline_description.c_str(), NULL);
}

gboolean VideoServer::bus_call(GstBus *bus, GstMessage *msg, void *user_data) {

	switch( GST_MESSAGE_TYPE(msg) ) {
		case GST_MESSAGE_EOS: {
			g_message("End-of-stream");
			g_main_loop_quit(loop);
			break;
		}
		case GST_MESSAGE_ERROR: {
			GError *err;
			gst_message_parse_error(msg, &err, NULL);
			g_error("%s", err->message);
			g_error_free(err);

			g_main_loop_quit(loop);
			break;
		}
		default:
			break;
	}

	return true;
}
         

void VideoServer::run() {
	if(!pipeline) return;
	
	loop = g_main_loop_new(NULL, FALSE);

	GstBus *bus;
	bus = gst_pipeline_get_bus( GST_PIPELINE(pipeline) );
	gst_bus_add_watch(bus, bus_call, NULL);
	gst_object_unref(bus);
	
	gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);

	g_main_loop_run(loop);

	gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_NULL);
	gst_object_unref( GST_OBJECT(pipeline) );
}

} // namespace gstreamer
} // namespace hub
