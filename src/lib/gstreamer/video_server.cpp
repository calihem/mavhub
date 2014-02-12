#include "video_server.h"

#include <iostream>	//cout
#include <fstream>	//ifstream
#include <stdexcept>
#include <cassert>
#include <unistd.h>

#include <gst/app/gstappsink.h>
#if GST_VERSION_MAJOR == 0 && GST_VERSION_MINOR == 10
#include <gst/app/gstappbuffer.h>
#endif

namespace hub {
namespace gstreamer {

#define VERBOSE 0

// little debug makro
#if VERBOSE >= 1
#define dout if(1) cout
#else
#define dout if(0) cout
#endif

using namespace std;

GMainLoop *VideoServer::loop = NULL;
std::multimap<VideoClient*, GstElement*> VideoServer::client_sink_map;

VideoServer::VideoServer(int *argc, char **argv, const std::string &pipeline_description) {

	gst_init(argc, &argv);
	add_pipeline(pipeline_description);
}

VideoServer::VideoServer(int *argc, char **argv, const std::list<std::string> &pipeline_descriptions) {
	gst_init(argc, &argv);
	std::list<std::string>::const_iterator descr_iter;
	for(descr_iter = pipeline_descriptions.begin(); descr_iter != pipeline_descriptions.end(); descr_iter++) {
		add_pipeline(*descr_iter);
	}
}

int VideoServer::add_pipeline(const std::string &pipeline_description) {
	GError *error(NULL);
	GstElement *bin = gst_parse_bin_from_description(pipeline_description.c_str(), false, &error);
	if(error) {
		g_print("GStreamer parse error: %s\n", error->message);
	}
	if(!bin) return -1;
	
	GstElement *pipe = gst_pipeline_new(NULL);
	if(!pipe) {
		g_object_unref(bin);
		return -2;
	}

	gst_bin_add(GST_BIN(pipe), bin);

	// add bus watch
	GstBus *bus = gst_pipeline_get_bus( GST_PIPELINE(pipe) );
	gst_bus_add_watch(bus, bus_call, NULL);
	gst_object_unref(bus);

	gst_element_set_state(GST_ELEMENT(pipe), GST_STATE_PLAYING);

	int id = pipeline_vector.size();
	pipeline_vector.push_back(pipe);
	
	return id;
}

int VideoServer::bind2appsink(VideoClient* client, const std::string &element, const int pipeline_id) {
	GstElement *sink = VideoServer::element(element, pipeline_id);
	if(!sink) return -1;

	// sink already connected?
	std::map<VideoClient*, GstElement*>::iterator cli_sink_iter = client_sink_map.begin();
	for( ; cli_sink_iter != client_sink_map.end(); ++cli_sink_iter) {
		if(cli_sink_iter->second == sink) break;
	}
	if( cli_sink_iter == client_sink_map.end() ) { // not connected
		g_object_set( G_OBJECT(sink), "emit_signals", true, "sync", false, NULL);
#if GST_VERSION_MAJOR == 0 && GST_VERSION_MINOR == 10
		g_signal_connect(sink, "new_buffer",
			G_CALLBACK(new_video_buffer_callback), NULL);
#elif GST_VERSION_MAJOR == 1
		g_signal_connect(sink, "new-sample",
			G_CALLBACK(new_video_buffer_callback), NULL);
#endif
	}

	// put pair to connection map
	client_sink_map.insert( std::make_pair(client, sink) );
	gst_object_unref(sink);

	return 0;
}

gboolean VideoServer::bus_call(GstBus *bus, GstMessage *msg, void *user_data) {

	switch( GST_MESSAGE_TYPE(msg) ) {
		case GST_MESSAGE_EOS: {
			g_message("x End-of-stream");
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

GstElement* VideoServer::element(const std::string &name, const int pipeline_id) const {
	if(pipeline_id >= 0) { // valid ID => search only in this pipeline
		GstElement *pipe = pipeline(pipeline_id);
		return gst_bin_get_by_name( GST_BIN(pipe), name.c_str() );
	} else { //search in every pipeline
		GstElement *elem;
		for(std::vector<GstElement*>::const_iterator pipe_iter = pipeline_vector.begin();
		pipe_iter != pipeline_vector.end();
		pipe_iter++) {
			elem = gst_bin_get_by_name( GST_BIN(*pipe_iter), name.c_str() );
			if(elem) return elem;
		}
	}
	return NULL;
}

#if GST_VERSION_MAJOR == 0 && GST_VERSION_MINOR == 10
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

	gint bpp(8);
	if( !gst_structure_get_int(struc, "bpp", &bpp) ) {
		assert(width != 0 && height != 0);
		bpp = 8*GST_BUFFER_SIZE(buffer)/width/height;
	} else {
		assert(8*GST_BUFFER_SIZE(buffer) >= (unsigned)width*height*bpp);
	}

	unsigned char *buf_data = GST_BUFFER_DATA(buffer);
	std::map<VideoClient*, GstElement*>::iterator cli_sink_iter = client_sink_map.begin();
	for( ; cli_sink_iter != client_sink_map.end(); ++cli_sink_iter) {
		if(cli_sink_iter->second != element) continue;

		cli_sink_iter->first->handle_video_data(buf_data, width, height, bpp);
	}

	gst_buffer_unref(buffer);
}
#elif GST_VERSION_MAJOR == 1
GstFlowReturn VideoServer::new_video_buffer_callback(GstElement *element, GstElement *data) {
	dout << "[VideoServer] new_video_buffer_callback got called" << endl;

	GstFlowReturn rc = GST_FLOW_CUSTOM_ERROR;

	GstAppSink *sink = GST_APP_SINK(element);
	if(!sink) return rc;

	GstSample *sample = gst_app_sink_pull_sample(sink);
	if(!sample) {
		g_print("ERROR: pulling sample from appsink failed\n");
		return rc;
	}
	// define variables here to avoid jumps crossing initialization
	GstBuffer *buffer(NULL);
	gint bpp(8);
	GstMemory *memory(NULL);
	std::map<VideoClient*, GstElement*>::iterator cli_sink_iter;

	GstCaps *caps = gst_sample_get_caps(sample);
	if(!caps) {
		g_print("ERROR: getting caps from sample failed\n");
		goto _return;
	}
	if( !gst_caps_is_fixed(caps) ) {
		g_print("ERROR: caps aren't fixed\n");
		rc = GST_FLOW_NOT_SUPPORTED;
		goto _return;
	}
	const GstStructure *struc;
	struc = gst_caps_get_structure(caps, 0); // structure returned belongs to caps
	gint height, width;
	if( !gst_structure_get_int(struc, "width", &width)
	|| !gst_structure_get_int(struc, "height", &height) ) {
		g_print ("ERROR: width/height not available\n");
		goto _return;
	}

	buffer = gst_sample_get_buffer(sample); // buffer belongs to sample
	if(!buffer) {
		g_print("ERROR: getting buffer from sample failed\n");
		goto _return;
	}

	if( !gst_structure_get_int(struc, "bpp", &bpp) ) {
		assert(width != 0 && height != 0);
		bpp = 8*gst_buffer_get_size(buffer)/width/height;
	} else {
		assert(8*gst_buffer_get_size(buffer) >= (unsigned)width*height*bpp);
	}

	memory = gst_buffer_get_memory(buffer, 0);
	if(!memory) {
		g_print("ERROR: getting memory from buffer");
		goto _return;
	}
	GstMapInfo info;
	if(gst_memory_map(memory, &info, GST_MAP_READ) == FALSE) {
		g_print("ERROR: mapping memory failed");
		goto _return;
	}
	dout << "[VideoServer] filled info struct" << endl;

	cli_sink_iter = client_sink_map.begin();
	for( ; cli_sink_iter != client_sink_map.end(); ++cli_sink_iter) {
		if(cli_sink_iter->second != element) continue;

		dout << "[VideoServer] call handle_video_data" << endl;
		cli_sink_iter->first->handle_video_data((unsigned char*)info.data, width, height, bpp);
	}
	rc = GST_FLOW_OK;

_return:
	if(memory) gst_memory_unref(memory);
	gst_sample_unref(sample);
	dout << "[VideoServer] released all data" << endl;
	return rc;
}
#endif

void VideoServer::print_elements() const {
#if GST_VERSION_MAJOR == 0 && GST_VERSION_MINOR == 10
	std::vector<GstElement*>::const_iterator pipe_iter;
	for(pipe_iter = pipeline_vector.begin(); pipe_iter != pipeline_vector.end(); pipe_iter++) {
		GstIterator *iter = gst_bin_iterate_elements( GST_BIN(*pipe_iter) );
		gpointer item;
		g_print ("Pipeline elements:\n");

		bool done = false;
		while (!done) {
			switch (gst_iterator_next (iter, &item)) {
				case GST_ITERATOR_OK:
					//... use/change item here...
					gchar *name;
					g_object_get( G_OBJECT(item), "name", &name, NULL);
					g_print ("\t '%s'.\n", name);
					g_free (name);
					gst_object_unref (item);
					break;
				case GST_ITERATOR_RESYNC:
					// ...rollback changes to items...
					gst_iterator_resync (iter);
					break;
				case GST_ITERATOR_ERROR:
					// ...wrong parameters were given...
					done = true;
					break;
				case GST_ITERATOR_DONE:
					done = true;
					break;
			}
		}
		gst_iterator_free (iter);
		g_print("\n");
	}
#elif GST_VERSION_MAJOR == 1
	//TODO
#endif
}

int VideoServer::push(GstAppSrc *appsrc, unsigned char *data, const int width, const int height, const int bpp) {
	if(!appsrc) return -1;

	if(!data) { // assume EOS
		gst_app_src_end_of_stream(appsrc);
		return 0;
	}

	int data_size = width*height*bpp/8;
#if GST_VERSION_MAJOR == 0 && GST_VERSION_MINOR == 10
	// copy data (because of different threads)
	gpointer buf_data = g_memdup(data, data_size);
	GstBuffer *buf = gst_app_buffer_new(buf_data, data_size, g_free, buf_data);
	gst_app_src_push_buffer(GST_APP_SRC(appsrc), buf); //this function takes ownership of buf
#elif GST_VERSION_MAJOR == 1
	gpointer buffer_data = g_memdup(data, data_size);
	GstBuffer *buffer = gst_buffer_new_wrapped(buffer_data, data_size);
	gst_app_src_push_buffer(appsrc, buffer); //this function takes ownership of buf
#endif
	return 0;
}

void VideoServer::release(const VideoClient *client) {
	//TODO
}

void VideoServer::run() {
	if( pipeline_vector.empty() ) return;
	

// 	std::vector<GstElement*>::iterator pipe_iter;
// 	for(pipe_iter = pipeline_vector.begin(); pipe_iter != pipeline_vector.end(); pipe_iter++) {
// 		gst_element_set_state(GST_ELEMENT(*pipe_iter), GST_STATE_PLAYING);
// 	}

	loop = g_main_loop_new(NULL, FALSE);
	g_main_loop_run(loop);

	std::vector<GstElement*>::iterator pipe_iter;
	for(pipe_iter = pipeline_vector.begin(); pipe_iter != pipeline_vector.end(); pipe_iter++) {
		gst_element_set_state(*pipe_iter, GST_STATE_NULL);
		gst_object_unref( GST_OBJECT(*pipe_iter) );
	}
}

} // namespace gstreamer
} // namespace hub
