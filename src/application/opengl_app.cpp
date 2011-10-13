#include "opengl_app.h"

#ifdef HAVE_MAVLINK_H

#ifdef HAVE_GL_GLUT_H

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

#include "lib/opengl/map_2d.h"

typedef struct { int x, y; } xy;

using namespace cpp_pthread;
using namespace hub::opengl;

namespace mavhub {

OpenGLApp::OpenGLApp(const Logger::log_level_t loglevel) :
	AppInterface("opengl_app", loglevel),
	AppLayer<mavlink_message_t>("opengl_app", loglevel),
#ifdef HAVE_GSTREAMER
	hub::gstreamer::VideoClient(),
#endif // HAVE_GSTREAMER
	textures(1)
	{
	pthread_mutex_init(&tx_mav_mutex, NULL);
}

OpenGLApp::~OpenGLApp() {}

void OpenGLApp::handle_input(const mavlink_message_t &msg) {
// 	log("OpenGLApp got mavlink_message", static_cast<int>(msg.msgid), Logger::LOGLEVEL_DEBUG);

	switch(msg.msgid) {
		case MAVLINK_MSG_ID_ATTITUDE: {
			log("OpenGLApp got mavlink attitude", Logger::LOGLEVEL_DEBUG);
			mavlink_attitude_t attitude;
			mavlink_msg_attitude_decode(&msg, &attitude);
// 			attitude.roll = 0.0;
// 			attitude.pitch = 0.0;
// 			attitude.yaw = 0.0;
			Map2D::camera_direction(attitude.roll, attitude.pitch, attitude.yaw, false);
			break;
		}
		default: break;
	}
}

#ifdef HAVE_GSTREAMER
void OpenGLApp::handle_video_data(const unsigned char *data, const int width, const int height, const int bpp) {
	//FIXME
	static int num_features;
	static xy features[1280];

	if(!data) return;

	if(bpp == 24) { // assume new image
		log("OpenGLApp new image", bpp, Logger::LOGLEVEL_DEBUG);
		OpenGLApp::width = width;
		OpenGLApp::height = height;

		Lock buf_lock(buf_mutex);
// 		memset(buffer, 0, width*height*3);
		memcpy(buffer, data, width*height*3);
		// set features
// 		buffer[3*(100 * OpenGLApp::width + 100)] = 0;
// 		buffer[3*(100 * OpenGLApp::width + 100) + 1] = 0;
// 		buffer[3*(100 * OpenGLApp::width + 100) + 2] = 255;
		xy *feature_it = features;
		for(int i = 0; i < num_features; i++) {
			buffer[3*(OpenGLApp::width*feature_it->y + feature_it->x)] = 0;
			buffer[3*(OpenGLApp::width*feature_it->y + feature_it->x) + 1] = 0;
			buffer[3*(OpenGLApp::width*feature_it->y + feature_it->x) + 2] = 255;
			feature_it++;
		}
	} else { // assume new features
		Logger::log("OpenGLApp new features", width, height, bpp, Logger::LOGLEVEL_DEBUG);
// 		int num_features;
		width < height ? num_features = height : num_features = width;
		assert(num_features <= 1280);
		memcpy(features, data, num_features*sizeof(xy));
// 		log("num_features:", num_features, Logger::LOGLEVEL_DEBUG);
// 		xy *feature_it = (xy*)(data);
// 		Lock buf_lock(buf_mutex);
// 		for(int i = 0; i < num_features; i++) {
			//FIXME: check dimensions
// 			if(feature_it->y > 640 || feature_it->y < 0
// 			|| feature_it->x > 640 || feature_it->x < 0)
// 				log(feature_it->x, feature_it->y, Logger::LOGLEVEL_DEBUG);
// 			buffer[3*(OpenGLApp::width*feature_it->y + feature_it->x)] = 0;
// 			buffer[3*(OpenGLApp::width*feature_it->y + feature_it->x + 1)] = 255;
// 			buffer[3*(OpenGLApp::width*feature_it->y + feature_it->x + 2)] = 255;
// 			feature_it++;
// 		}
	}
}
#endif // HAVE_GSTREAMER

void OpenGLApp::print(std::ostream &os) const {
	AppLayer<mavlink_message_t>::print(os);
}

void OpenGLApp::run() {
	log("OpenGLApp running", Logger::LOGLEVEL_DEBUG);

	Map2D map(Core::argc, Core::argv);
	Map2D::bind_textures(textures);

#ifdef HAVE_GSTREAMER
	if(Core::video_server) {
		int rc;
		rc = Core::video_server->bind2appsink( dynamic_cast<VideoClient*>(this), "sink0");
		log("OpenGLApp binded to sink0 with status", rc, Logger::LOGLEVEL_DEBUG);
		if(rc < 0) return;
		rc = Core::video_server->bind2appsink( dynamic_cast<VideoClient*>(this), "featuresink0");
		log("OpenGLApp binded to featuresink0 with status", rc, Logger::LOGLEVEL_DEBUG);
		if(rc < 0);
	} else {
		log("video server not running", Logger::LOGLEVEL_WARN);
	}
#endif // HAVE_GSTREAMER

// 	Map2D::load_texture(textures[0], std::string("texture.bmp"), 290, 290);
// 	Map2D::load_texture(textures[0], buffer, 290, 290);

	while( !interrupted() ) {
		{
			Lock buf_lock(buf_mutex);
			Map2D::load_texture(textures[0], buffer, width, height);
		}
		Map2D::display();
		usleep(500);
	}

	log("OpenGLApp stop running", Logger::LOGLEVEL_DEBUG);
}


void OpenGLApp::send_heartbeat() {
	Lock tx_lock(tx_mav_mutex);
	mavlink_msg_heartbeat_pack(system_id(), component_id, &tx_mav_msg, MAV_QUADROTOR, MAV_AUTOPILOT_HUCH);
	AppLayer<mavlink_message_t>::send(tx_mav_msg);
}


} // namespace mavhub

#endif // HAVE_GL_GLUT_H
#endif // HAVE_MAVLINK_H
