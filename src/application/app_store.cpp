#include "app_store.h"

#include "protocol/protocolstack.h"

#include "module/modulebase.h"
#include "module/testcore.h"
#include "module/fc_mpkg.h"
#include "module/ctrl_alt_simple.h"
#include "module/ctrl_hover.h"
#include "module/ctrl_lateral.h"
#include "module/ctrl_yaw.h"
#include "module/ctrl_zrate.h"
#include "module/ctrl_bump.h"
#include "module/ctrl_lat_bump.h"
#include "module/ctrl_logfileplayer.h"
#include "module/ctrl_logger.h"
#include "module/ctrl_wifimeter.h"
#include "module/plat_link_car.h"
#include "module/plat_link_crrcsim.h"
#include "module/plat_link_mk.h"
#include "module/bridge_ivy.h"
#include "module/bridge_osc.h"
#include "module/ui_potibox.h"
#include "core_app.h"
#include "fiducal_app.h"
#include "fiducal_control_app.h"
#include "mavlink_mk_app.h"
#include "mavlink_mkhuch_app.h"
#include "mk_app.h"
#include "msp_app.h"
#include "acc_calibration_app/acc_calibration_app.h"
#include "attitude_filter_app/attitude_filter_app.h"
#include "opengl_app.h"
#include "slam_app.h"
#include "vision/v_oflow_app.h"
#include "vision/v_oflow_car_app.h"
#include "vision/v_oflow_odca_app.h"
#include "vision/v_camctrl_app.h"

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
		CoreApp *core_app = new CoreApp(args, loglevel);
		return ProtocolStack<mavlink_message_t>::instance().add_application(core_app);
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "acc_calibration_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_OPENCV
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
#endif // HAVE_OPENCV
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "attitude_filter_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_OPENCV
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
#endif // HAVE_OPENCV
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
	} else if(lowercase_name == "fiducal_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_GSTREAMER
#if (defined(HAVE_OPENCV2) && CV_MINOR_VERSION > 2)
		FiducalApp *f_app = new FiducalApp(args, loglevel);
		return ProtocolStack<mavlink_message_t>::instance().add_application(f_app);
#endif // HAVE_OPENCV2 and CV_MINOR_VERSION > 2
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "fiducal_control_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_OPENCV2
		FiducalControlApp *f_app = new FiducalControlApp(args, loglevel);
		return ProtocolStack<mavlink_message_t>::instance().add_application(f_app);
#endif // HAVE_OPENCV2
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "ctrl_hover_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MKLINK_H
#ifdef HAVE_OPENCV
		// pass only configuration map into constructor
		Ctrl_Hover *ctrl_hover_app = new Ctrl_Hover(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ctrl_hover_app);
#endif // HAVE_OPENCV
#endif // HAVE_MKLINK_H
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "ctrl_alt_simple_app") {
#ifdef HAVE_MAVLINK_H
		// pass only configuration map into constructor
		Ctrl_Alt_Simple *ctrl_alt_simple_app = new Ctrl_Alt_Simple(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ctrl_alt_simple_app);
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "ctrl_lateral_app") {
#ifdef HAVE_MAVLINK_H
		// pass only configuration map into constructor
		Ctrl_Lateral *ctrl_lateral_app = new Ctrl_Lateral(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ctrl_lateral_app);
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "ctrl_yaw_app") {
#ifdef HAVE_MAVLINK_H
		// pass only configuration map into constructor
		Ctrl_Yaw *ctrl_yaw_app = new Ctrl_Yaw(args, loglevel);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ctrl_yaw_app);
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
	} else if(lowercase_name == "ctrl_lat_bump_app") {
#ifdef HAVE_MAVLINK_H
		// pass only configuration map into constructor
		Ctrl_Lat_Bump *ctrl_lat_bump_app = new Ctrl_Lat_Bump(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ctrl_lat_bump_app);
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
#ifdef HAVE_OPENCV2
#if CV_MINOR_VERSION >= 3
#if defined HAVE_LIBBLAS && defined HAVE_LIBLAPACK
		SLAMApp *slam_app = new SLAMApp(args, loglevel);
		return ProtocolStack<mavlink_message_t>::instance().add_application(slam_app);
#endif // HAVE_LIBBLAS && HAVE_LIBLAPACK
#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "v_oflow_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_GSTREAMER
#ifdef HAVE_LIBFANN
#ifdef HAVE_OPENCV2
#if CV_MINOR_VERSION >= 2
		V_OFLOWApp *v_oflow_app = new V_OFLOWApp(args, loglevel);
		return ProtocolStack<mavlink_message_t>::instance().add_application(v_oflow_app);
#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // HAVE_LIBFANN
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "v_oflow_car_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_GSTREAMER
#ifdef HAVE_LIBFANN
#ifdef HAVE_OPENCV2
#if CV_MINOR_VERSION >= 2
		V_OFLOWCarApp *v_oflow_car_app = new V_OFLOWCarApp(args, loglevel);
		return ProtocolStack<mavlink_message_t>::instance().add_application(v_oflow_car_app);
#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // HAVE_LIBFANN
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "v_oflow_odca_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_GSTREAMER
#ifdef HAVE_LIBFANN
#ifdef HAVE_OPENCV2
#if CV_MINOR_VERSION >= 2
		V_OFLOWOdcaApp *v_oflow_odca_app = new V_OFLOWOdcaApp(args, loglevel);
		return ProtocolStack<mavlink_message_t>::instance().add_application(v_oflow_odca_app);
#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // HAVE_LIBFANN
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "v_camctrl_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_GSTREAMER
#ifdef HAVE_OPENCV2
#if CV_MINOR_VERSION >= 2
		V_CAMCTRLApp *v_camctrl_app = new V_CAMCTRLApp(args, loglevel);
		return ProtocolStack<mavlink_message_t>::instance().add_application(v_camctrl_app);
#endif // CV_MINOR_VERSION
#endif // HAVE_OPENCV2
#endif // HAVE_GSTREAMER
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "plat_link_car_app") {
#ifdef HAVE_MAVLINK_H
		// pass only configuration map into constructor
		Plat_Link_Car *plat_link_car_app = new Plat_Link_Car(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(plat_link_car_app);
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "plat_link_crrcsim_app") {
#ifdef HAVE_MAVLINK_H
		// pass only configuration map into constructor
		Plat_Link_Crrcsim *plat_link_crrcsim_app = new Plat_Link_Crrcsim(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(plat_link_crrcsim_app);
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "plat_link_mk_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MKHUCHLINK_H
		// pass only configuration map into constructor
		Plat_Link_Mk *plat_link_mk_app = new Plat_Link_Mk(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(plat_link_mk_app);
#endif // HAVE_MKHUCHLINK_H
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "bridge_ivy_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_IVY_IVY_H
		// pass only configuration map into constructor
		Bridge_Ivy * bridge_ivy_app = new Bridge_Ivy(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(bridge_ivy_app);
#endif // HAVE_IVY_IVY_H
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "bridge_osc_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_LIBOSCPACK
		// pass only configuration map into constructor
		Bridge_Osc * bridge_osc_app = new Bridge_Osc(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(bridge_osc_app);
#endif // HAVE_LIBOSCPACK
#endif // HAVE_MAVLINK_H
	} else if(lowercase_name == "ui_potibox_app") {
#ifdef HAVE_MAVLINK_H
		UI_Potibox *ui_potibox_app = new UI_Potibox(args);
		return ProtocolStack<mavlink_message_t>::instance().add_application(ui_potibox_app);
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
	} else if(lowercase_name == "msp_app") {
#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MSPLINK_H
          MSPApp *msp_app = new MSPApp(args, loglevel);

          int rc = ProtocolStack<mavlink_message_t>::instance().add_application(msp_app);
          if (!rc)
            return ProtocolStack<msp_message_t>::instance().add_application(msp_app);
          else
            Logger::log("AppStore: failed to add msp_app to ProtocolStack<mavlink_message_t>", Logger::LOGLEVEL_WARN);
          Logger::log("AppStore: added msp_app to ProtocolStack<mavlink_message_t>", Logger::LOGLEVEL_DEBUG);
          return rc;
#endif // HAVE_MSPLINK_H
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

	Logger::log("AppStore: Can't build app", lowercase_name, "since dependencies are not fulfilled ", Logger::LOGLEVEL_INFO);
	return -1;
}

} // namespace mavhub
