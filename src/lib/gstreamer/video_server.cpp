#include "video_server.h"

#include <iostream>	//cout
#include <fstream>	//ifstream
#include <stdexcept>
#include <cassert>

#include <gst/gst.h>
//FIXME: check for appsink in autotools
#include <gst/app/gstappsink.h>
#include <gst/app/gstappbuffer.h>

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
std::map<VideoClient*, GstElement*> VideoServer::client_sink_map;

VideoServer::VideoServer(int *argc, char **argv, const std::string &pipeline_description) : 
	pipeline(NULL) {

	gst_init(argc, &argv);
	
	GError *error = NULL;
	pipeline = gst_parse_launch(pipeline_description.c_str(), &error);
	if(!pipeline) {
		g_print("GStreamer parse error: %s\n", error->message);
	}
}

int VideoServer::bind2appsink(VideoClient* client, const std::string &element) {
	GstElement *sink = VideoServer::element(element);
	if(!sink) return -1;

	// sink already connected?
	std::map<VideoClient*, GstElement*>::iterator cli_sink_iter = client_sink_map.begin();
	for( ; cli_sink_iter != client_sink_map.end(); ++cli_sink_iter) {
		if(cli_sink_iter->second == sink) break;
	}
	if( cli_sink_iter == client_sink_map.end() ) { // not connected
		g_object_set( G_OBJECT(sink), "emit_signals", true, "sync", false, NULL);
		g_signal_connect(sink, "new_buffer",
			G_CALLBACK(new_video_buffer_callback), NULL);	
	}

	// put pair to connection map
	client_sink_map.insert( std::make_pair(client, sink) );
	gst_object_unref(sink);

	return 0;
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
		case GST_MESSAGE_WARNING: {
			GError *err;
			gchar *msg_str;
			gst_message_parse_warning(msg, &err, &msg_str);
			g_warning("%s", msg_str);
			g_error_free(err);
			g_free(msg_str);
		}
		case GST_MESSAGE_APPLICATION: {
			const gchar *message_name = gst_structure_get_name( gst_message_get_structure(msg) );
			g_warning("%s", message_name);
		}
		default:
			break;
	}

	return true;
}

GstElement* VideoServer::element(const std::string &name) const {
	return gst_bin_get_by_name( GST_BIN(pipeline), name.c_str() );
}

void VideoServer::new_video_buffer_callback(GstElement *element, GstElement *data) { 
	GstAppSink *sink = GST_APP_SINK(element);
	if(!sink) return;

	GstBuffer *buffer = gst_app_sink_pull_buffer(sink);
	if(!buffer) {
		g_print("ERROR: pulling buffer from appsink failed\n");
		return;
	}

	GstCaps *caps = GST_BUFFER_CAPS(buffer);
	if(!caps) {
		g_print("ERROR: getting caps from buffer failed\n");
		gst_buffer_unref(buffer);
		return;
	}
	if( !gst_caps_is_fixed(caps) ) {
		g_print("ERROR: caps aren't fixed\n");
		gst_buffer_unref(buffer);
		return;
	}
	const GstStructure *struc;
	struc = gst_caps_get_structure(caps, 0);
	gint height, width;
	if( !gst_structure_get_int(struc, "width", &width)
	|| !gst_structure_get_int(struc, "height", &height) ) {
		g_print ("ERROR: width/height not available\n");
		gst_buffer_unref(buffer);
		return;
	}
	assert( GST_BUFFER_SIZE(buffer) == width*height*3);

	unsigned char *buf_data = GST_BUFFER_DATA(buffer);
	std::map<VideoClient*, GstElement*>::iterator cli_sink_iter = client_sink_map.begin();
	for( ; cli_sink_iter != client_sink_map.end(); ++cli_sink_iter) {
		if(cli_sink_iter->second != element) continue;

		cli_sink_iter->first->handle_video_data(buf_data, width, height);
	}

	gst_buffer_unref(buffer);
}

void VideoServer::release(const VideoClient *client) {
	
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
