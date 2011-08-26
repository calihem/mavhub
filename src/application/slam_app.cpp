#include "slam_app.h"

#ifdef HAVE_GSTREAMER

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

using namespace cpp_pthread;

namespace mavhub {

SLAMApp::SLAMApp(const Logger::log_level_t loglevel) :
	AppInterface("slam_app", loglevel),
	AppLayer<mavlink_message_t>("slam_app", loglevel),
	hub::gstreamer::VideoClient()
	{
// 	pthread_mutex_init(&tx_mav_mutex, NULL);
}

SLAMApp::~SLAMApp() {}

void SLAMApp::handle_input(const mavlink_message_t &msg) {
// 	log("SLAMApp got mavlink_message", static_cast<int>(msg.msgid), Logger::LOGLEVEL_DEBUG);
}


void SLAMApp::handle_video_data(const unsigned char *data, const int width, const int height, const int bpp) {
	if(!data) return;
	log("SLAMApp: got", width, "features", Logger::LOGLEVEL_DEBUG);
}

void SLAMApp::print(std::ostream &os) const {
	AppLayer<mavlink_message_t>::print(os);
}

void SLAMApp::run() {
	log("SLAMApp running", Logger::LOGLEVEL_DEBUG);


	if(Core::video_server) {
		int rc = Core::video_server->bind2appsink( dynamic_cast<VideoClient*>(this), "featuresink0");
		log("SLAMApp binded to fastfeature0", rc, Logger::LOGLEVEL_DEBUG);
	} else {
		log("video server not running", Logger::LOGLEVEL_WARN);
	}

	while( !interrupted() ) {
		usleep(500);
	}

	log("SLAMApp stop running", Logger::LOGLEVEL_DEBUG);
}

} // namespace mavhub

#endif // HAVE_GSTREAMER
