#include "opengl_app.h"

#ifdef HAVE_GL_GLUT_H

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

#include "lib/opengl/map_2d.h"

using namespace cpp_pthread;
using namespace hub::opengl;

namespace mavhub {

OpenGLApp::OpenGLApp(const Logger::log_level_t loglevel) :
	AppInterface("opengl_app", loglevel),
	AppLayer<mavlink_message_t>("opengl_app", loglevel),
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


void OpenGLApp::print(std::ostream &os) const {
	AppLayer<mavlink_message_t>::print(os);
}

void OpenGLApp::run() {
	log("OpenGLApp running", Logger::LOGLEVEL_DEBUG);

	Map2D map(Core::argc, Core::argv);
	Map2D::bind_textures(textures);

	Map2D::load_texture(textures[0], std::string("texture.bmp"), 290, 290);

	while( !interrupted() ) {
// 		Map2D::load_texture(&textures[0], *capture_image);
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
