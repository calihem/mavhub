#include "app_store.h"

#include "protocol/protocolstack.h"

#include "module/coremod.h"
#include "module/testcore.h"
#include "module/fc_mpkg.h"
#include "module/ctrl_hover.h"
#include "module/ctrl_lateral.h"
#include "module/ctrl_zrate.h"
#include "module/ctrl_bump.h"
#include "module/ctrl_logfileplayer.h"
#include "module/ctrl_logger.h"
#include "module/ctrl_wifimeter.h"
#include "module/sim_crrcsim.h"
#include "mavlink_mk_app.h"
#include "mavlink_mkhuch_app.h"
#include "mk_app.h"
#include "acc_calibration_app/acc_calibration_app.h"
#include "attitude_filter_app/attitude_filter_app.h"
#include "opengl_app.h"
#include "slam_app.h"

#include <iostream>
#include <sstream> //istringstream
#include <algorithm> //transform

using namespace std;

namespace mavhub {

int AppStore::order(const std::string& app_name, const std::map<std::string, std::string>& args) {
	//transform application name to lower case
	std::string lowercase_name(app_name);
	transform(lowercase_name.begin(), lowercase_name.end(), lowercase_name.begin(), ::tolower);

	//get loglevel
	std::map<std::string,std::string>::const_iterator find_iter = args.find("loglevel");
	Logger::log_level_t loglevel;
	if( find_iter != args.end() ) {
		istringstream istream(find_iter->second);
		istream >> loglevel;
	} else { //no loglevel given, set default
		Logger::log("No loglevel given for", app_name, Logger::LOGLEVEL_DEBUG);
		loglevel = Logger::LOGLEVEL_WARN;
	}

	if(lowercase_name == "test_app") {
#ifdef HAVE_MAVLINK_H
		TestCore *test_app = new TestCore();
		return ProtocolStack<mavlink_message_t>::instance().add_application(test_app);
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "core_app") {
#ifdef HAVE_MAVLINK_H
		CoreModule *core_app = new CoreModule();
		return ProtocolStack<mavlink_message_t>::instance().add_application(core_app);
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "acc_calibration_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_OPENCV_CV_H
		std::string arg;
		int number_of_measurements_for_determine_min_max(50);
		find_iter = args.find("number_of_measurements_for_determine_min_max");
		if(find_iter != args.end()) {
			istringstream istream(find_iter->second);
			istream >> number_of_measurements_for_determine_min_max;
		}
		uint32_t measurement_timeout_in_us(1000000);
		find_iter = args.find("measurement_timeout_in_us");
		if(find_iter != args.end()) {
			istringstream istream(find_iter->second);
			istream >> measurement_timeout_in_us;
		}
		AccCalibrationApp *acc_calib_app = new AccCalibrationApp(loglevel,
			number_of_measurements_for_determine_min_max,
			measurement_timeout_in_us);
		return ProtocolStack<mavlink_message_t>::instance().add_application(acc_calib_app);
#endif // HAVE_OPENCV_CV_H
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "attitude_filter_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_OPENCV_CV_H
		std::string arg;
		int number_of_measurements_for_gyro_bias_mean(50);
		find_iter = args.find("number_of_measurements_for_gyro_bias_mean");
		if(find_iter != args.end()) {
			istringstream istream(find_iter->second);
			istream >> number_of_measurements_for_gyro_bias_mean;
		}
		uint32_t measurement_timeout_in_us(100000);
		find_iter = args.find("measurement_timeout_in_us");
		if(find_iter != args.end()) {
			istringstream istream(find_iter->second);
			istream >> measurement_timeout_in_us;
		}
		AttitudeFilterApp *att_filter_app = new AttitudeFilterApp(loglevel,
			measurement_timeout_in_us,
			number_of_measurements_for_gyro_bias_mean);
		return ProtocolStack<mavlink_message_t>::instance().add_application(att_filter_app);
#endif // HAVE_OPENCV_CV_H
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "fc_mpkg_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MKLINK_H
		int component_id = 0;
		std::map<std::string,std::string>::const_iterator iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}
		FC_Mpkg *fc_mpkg_app = new FC_Mpkg(component_id);
		return ProtocolStack<mavlink_message_t>::instance().add_application(fc_mpkg_app);
#endif // HAVE_MKLINK_H
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "ctrl_hover_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MKLINK_H
#ifdef HAVE_OPENCV_CV_H
		// pass only configuration map into constructor
		Ctrl_Hover *ctrl_hover_app = new Ctrl_Hover(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ctrl_hover_app);
#endif // HAVE_OPENCV_CV_H
#endif // HAVE_MKLINK_H
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "ctrl_lateral_app") {
#ifdef HAVE_MAVLINK_H
		// pass only configuration map into constructor
		Ctrl_Lateral *ctrl_lateral_app = new Ctrl_Lateral(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ctrl_lateral_app);
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "ctrl_zrate_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MKLINK_H
		// pass only configuration map into constructor
		Ctrl_Zrate *ctrl_zrate_app = new Ctrl_Zrate(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ctrl_zrate_app);
#endif // HAVE_MKLINK_H
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "ctrl_logfileplayer_app") {
#ifdef HAVE_MAVLINK_H
		// pass only configuration map into constructor
		Ctrl_LogfilePlayer *ctrl_logfile_app = new Ctrl_LogfilePlayer(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ctrl_logfile_app);
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "ctrl_logger_app") {
#ifdef HAVE_MAVLINK_H
		// pass only configuration map into constructor
		Ctrl_Logger *ctrl_logger_app = new Ctrl_Logger(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ctrl_logger_app);
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "ctrl_bump_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MKLINK_H
		// pass only configuration map into constructor
		Ctrl_Bump *ctrl_bump_app = new Ctrl_Bump(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ctrl_bump_app);
#endif // HAVE_MKLINK_H
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "ctrl_wifimeter_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_LIBGSMM_H
		// pass only configuration map into constructor
		Ctrl_Wifimeter *ctrl_wifimeter_app = new Ctrl_Wifimeter(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ctrl_wifimeter_app);
#endif // HAVE_LIBGSMM_H
		Logger::log("AppFactory: libgps ist not available", Logger::LOGLEVEL_INFO); 
		return -1;
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "opengl_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_GL_GLUT_H
		OpenGLApp *gl_app = new OpenGLApp(loglevel);
		return ProtocolStack<mavlink_message_t>::instance().add_application(gl_app);
#endif // HAVE_GL_GLUT_H
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "slam_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_GSTREAMER
		SLAMApp *slam_app = new SLAMApp(loglevel);
		return ProtocolStack<mavlink_message_t>::instance().add_application(slam_app);
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "sim_crrcsim_app") {
#ifdef HAVE_MAVLINK_H
		// pass only configuration map into constructor
		Sim_Crrcsimule * sim_crrcsim_app = new Sim_Crrcsimule(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(sim_crrcsim_app);
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "mavlink_mk_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MKLINK_H
		std::string dev_name;
		find_iter = args.find("device");
		if(find_iter != args.end()) {
			dev_name = find_iter->second;
		} else {
			dev_name = "/dev/ttyS0";
		}
		unsigned int baudrate(57600);
		find_iter = args.find("baudrate");
		if(find_iter != args.end()) {
			istringstream istream(find_iter->second);
			istream >> baudrate;
		}
		MAVLinkMKApp *mavlink_mk_app = new MAVLinkMKApp(loglevel);

		int rc = ProtocolStack<mavlink_message_t>::instance().add_application(mavlink_mk_app);
		if (!rc)
			return ProtocolStack<mk_message_t>::instance().add_application(mavlink_mk_app);
		return rc;
#endif // HAVE_MKLINK_H
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "mavlink_mkhuch_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MKHUCHLINK_H
		MAVLinkMKHUCHApp *mavlink_mkhuch_app = new MAVLinkMKHUCHApp(loglevel);

		int rc = ProtocolStack<mavlink_message_t>::instance().add_application(mavlink_mkhuch_app);
		if (!rc)
			return ProtocolStack<mkhuch_message_t>::instance().add_application(mavlink_mkhuch_app);
		else
			Logger::log("AppStore: failed to add mavlink_mkhuch_app to ProtocolStack<mavlink_message_t>", Logger::LOGLEVEL_WARN);
		Logger::log("AppStore: added mavlink_mkhuch_app to ProtocolStack<mavlink_message_t>", Logger::LOGLEVEL_DEBUG);
		return rc;
#endif // HAVE_MKHUCHLINK_H
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "mk_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MKHUCHLINK_H
		std::string dev_name;
		find_iter = args.find("device");
		if(find_iter != args.end()) {
			dev_name = find_iter->second;
		} else {
			dev_name = "/dev/ttyS0";
		}
		unsigned int baudrate(57600);
		find_iter = args.find("baudrate");
		if(find_iter != args.end()) {
			istringstream istream(find_iter->second);
			istream >> baudrate;
		}
		MKApp *mk_app = new MKApp(loglevel);

		int rc = ProtocolStack<mavlink_message_t>::instance().add_application(mk_app);
		if (!rc)
			return ProtocolStack<mkhuch_message_t>::instance().add_application(mk_app);
		return rc;
#endif // HAVE_MKHUCHLINK_H
#endif // HAVE_MAVLINK_H
	} else {
		Logger::log("AppStore: no such app", lowercase_name, Logger::LOGLEVEL_WARN);
		return -2;
	}

	Logger::log("AppStore: Can't build app", lowercase_name, "since protocol not available", Logger::LOGLEVEL_INFO); 
	return -1;
}

} // namespace mavhub
