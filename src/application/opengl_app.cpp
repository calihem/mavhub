#include "opengl_app.h"

#ifdef HAVE_MAVLINK_H
#ifdef HAVE_GL_GLUT_H

#include "core/logger.h"
#include "core/datacenter.h"

#include "lib/opengl/map_2d.h"

using namespace hub;
using namespace hub::opengl;

namespace mavhub {

OpenGLApp::OpenGLApp(const Logger::log_level_t loglevel) :
	AppInterface("opengl_app", loglevel),
	AppLayer<mavlink_message_t>("opengl_app", loglevel),
#ifdef HAVE_GSTREAMER
	hub::gstreamer::VideoClient(),
#endif // HAVE_GSTREAMER
	textures(1),
	new_data(false)
	{
	pthread_mutex_init(&tx_mav_mutex, NULL);
}

OpenGLApp::~OpenGLApp() {}

void OpenGLApp::handle_input(const mavlink_message_t &msg) {
// 	log("OpenGLApp got mavlink_message", static_cast<int>(msg.msgid), Logger::LOGLEVEL_DEBUG);

	switch(msg.msgid) {
		case MAVLINK_MSG_ID_ATTITUDE: {
			log("got mavlink attitude", Logger::LOGLEVEL_DEBUG);
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
	if(!data) return;

	Logger::log("OpenGLApp got new video data", width, "x", height, "@", bpp,  Logger::LOGLEVEL_DEBUG, _loglevel);
	OpenGLApp::width = width;
	OpenGLApp::height = height;
	OpenGLApp::bpp = bpp;

	if(width*height*bpp/8 > buffer_size) {
		Logger::log(name(), "buffer size too small for video data", Logger::LOGLEVEL_WARN, _loglevel);
		return;
	}

	Lock buf_lock(buf_mutex);
	Logger::log(name(), "handle_video_data: locked buf_mutex", Logger::LOGLEVEL_DEBUG, _loglevel);
	if(bpp == 8) // gray image
		memcpy(buffer, data, width*height);
	else if(bpp == 24) // color image
		memcpy(buffer, data, width*height*3);
	else {
		Logger::log(name(), "bpp of size", bpp, "not supported", Logger::LOGLEVEL_WARN, _loglevel);
		return;
	}
	new_data = true;
	Logger::log(name(), "copied video data to buffer and unlock buf_mutex", Logger::LOGLEVEL_DEBUG, _loglevel);
}
#endif // HAVE_GSTREAMER

void OpenGLApp::print(std::ostream &os) const {
	AppLayer<mavlink_message_t>::print(os);
}

void OpenGLApp::run() {
	log("running", Logger::LOGLEVEL_DEBUG);

	Map2D map(Core::argc, Core::argv);
	Map2D::bind_textures(textures);

#ifdef HAVE_GSTREAMER
	if(Core::video_server) {
		int rc;
		rc = Core::video_server->bind2appsink( dynamic_cast<VideoClient*>(this), "sink0");
		log("binded to sink0 with status", rc, Logger::LOGLEVEL_DEBUG);
		if(rc < 0) return;
		rc = Core::video_server->bind2appsink( dynamic_cast<VideoClient*>(this), "featuresink0");
		log("binded to featuresink0 with status", rc, Logger::LOGLEVEL_DEBUG);
		if(rc < 0);
	} else {
		log("video server not running", Logger::LOGLEVEL_WARN);
	}
#endif // HAVE_GSTREAMER

// 	Map2D::load_texture(textures[0], std::string("texture.bmp"), 290, 290);
// 	Map2D::load_texture(textures[0], buffer, 290, 290);

	while( !interrupted() ) {
		if(new_data) {
			{ // buf_mutex scope
				Lock buf_lock(buf_mutex);
				Logger::log(name(), "loop: lock buf_mutex", Logger::LOGLEVEL_DEBUG, _loglevel);
				Map2D::load_texture(textures[0], buffer, width, height, bpp);
				new_data = false;
			}
			Logger::log(name(), "updated texture and unlocked buf_mutex", Logger::LOGLEVEL_DEBUG, _loglevel);
		}
		Map2D::display();
		usleep(250);
	}

	log("stop running", Logger::LOGLEVEL_DEBUG);
}


void OpenGLApp::send_heartbeat() {
	Lock tx_lock(tx_mav_mutex);
	mavlink_msg_heartbeat_pack(system_id(),
		component_id,
		&tx_mav_msg,
		MAV_TYPE_QUADROTOR,
		MAV_AUTOPILOT_GENERIC,
		MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,	//base mode
		0,	//custom mode
		MAV_STATE_ACTIVE);	//system status
	AppLayer<mavlink_message_t>::send(tx_mav_msg);
}


} // namespace mavhub

#endif // HAVE_GL_GLUT_H
#endif // HAVE_MAVLINK_H
