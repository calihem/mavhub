#include "app_factory.h"

#include "core/protocollayer.h"

#include "module/coremod.h"
#include "module/testcore.h"
#include "module/fc_mpkg.h"
#include "module/ctrl_hover.h"
#include "module/ctrl_lateral.h"
#include "module/ctrl_bump.h"
#include "module/ctrl_logfileplayer.h"
#include "module/ctrl_logger.h"
#include "module/sim_crrcsim.h"
#include "mk_app.h"
#include "acc_calibration_app/acc_calibration_app.h"
#include "attitude_filter_app/attitude_filter_app.h"

#include <iostream>
#include <algorithm> //transform

using namespace std;

namespace mavhub {

	// ----------------------------------------------------------------------------
	// AppFactory
	// ----------------------------------------------------------------------------
	AppLayer* AppFactory::build(const std::string& app_name, const std::map<std::string, std::string>& args) {
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
			return new TestCore();
		} else if(lowercase_name == "core_app") {
			return new CoreModule();
		} else if(lowercase_name == "acc_calibration_app") {
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
			return new AccCalibrationApp(loglevel, number_of_measurements_for_determine_min_max, measurement_timeout_in_us);
		} else if(lowercase_name == "attitude_filter_app") {
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
			return new AttitudeFilterApp(loglevel, measurement_timeout_in_us, number_of_measurements_for_gyro_bias_mean);
		} else if(lowercase_name == "fc_mpkg_app") {
			int component_id = 0;
			std::map<std::string,std::string>::const_iterator iter = args.find("component_id");
			if( iter != args.end() ) {
				istringstream s(iter->second);
				s >> component_id;
			}
			return new FC_Mpkg(component_id);
		} else if(lowercase_name == "ctrl_hover_app") {
			// pass only configuration map into constructor
			return new Ctrl_Hover(args);
		} else if(lowercase_name == "ctrl_lateral_app") {
			// pass only configuration map into constructor
			return new Ctrl_Lateral(args);
		} else if(lowercase_name == "ctrl_logfileplayer_app") {
			// pass only configuration map into constructor
			return new Ctrl_LogfilePlayer(args);
		} else if(lowercase_name == "ctrl_logger_app") {
			// pass only configuration map into constructor
			return new Ctrl_Logger(args);
		} else if(lowercase_name == "ctrl_bump_app") {
			// pass only configuration map into constructor
			return new Ctrl_Bump(args);
		} else if(lowercase_name == "sim_crrcsim_app") {
			// pass only configuration map into constructor
			return new Sim_Crrcsimule(args);
		} else if(lowercase_name == "mk_app") {
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
			return new MKApp(loglevel, dev_name, baudrate);
#else
			Logger::log("AppFactory: Can't build mk_app, since mkhuchlink not available", Logger::LOGLEVEL_INFO); 
			return NULL;
#endif // HAVE_MKHUCHLINK_H
		} else {
			Logger::log("factory/app: no such app", lowercase_name, Logger::LOGLEVEL_WARN);
		}
	
		return NULL;
	}
}
