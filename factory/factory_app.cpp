#include "factory_app.h"

#include <iostream>

using namespace std;

namespace mavhub {

	// ----------------------------------------------------------------------------
	// AppFactory
	// ----------------------------------------------------------------------------
	AppLayer* AppFactory::build(const std::string& app_name, const std::map<std::string, std::string> args) {
		//transform application name to lower case
		std::string lowercase_name(app_name);
		transform(lowercase_name.begin(), lowercase_name.end(), lowercase_name.begin(), ::tolower);

		if(lowercase_name == "test_app") {
			return new TestCore();
		} else if(lowercase_name == "core_app") {
			return new CoreModule();
		} else if(lowercase_name == "fc_mpkg_app") {
			int component_id = 0;
			std::map<std::string,std::string>::const_iterator iter = args.find("component_id");
			if( iter != args.end() ) {
				istringstream s(iter->second);
				s >> component_id;
			}
			return new FC_Mpkg(component_id);
		} else if(lowercase_name == "ctrl_hover_app") {
			// as defined in config
			// int component_id = 0;
			// int numchan = 0;
			// list <pair <int, int> > chanmap;
			// int ctl_bias = 0;
			// int ctl_Kc = 0;
			// int ctl_Ti = 0;
			// int ctl_Td = 0;
			// int ctl_sp = 0;
			// int ctl_bref = 0;
			// int ctl_sticksp = 0;

			//std::map<std::string,std::string>::const_iterator iter;

			// iter = args.find("component_id");
			// if( iter != args.end() ) {
			// 	istringstream s(iter->second);
			// 	s >> component_id;
			// }

			// iter = args.find("numsens");
			// if( iter != args.end() ) {
			// 	istringstream s(iter->second);
			// 	s >> numchan;
			// }

			// iter = args.find("inmap");
			// if( iter != args.end() ) {
			// 	istringstream s(iter->second);
			// 	s >> chanmap;
			// }

			// Logger::log(numchan, iter->second, Logger::LOGLEVEL_INFO);
			// Logger::log(chanmap, Logger::LOGLEVEL_INFO);

			// // separate contructor arguments
			// return new Ctrl_Hover(component_id, numchan, chanmap, args);
			// pass only configuration map into constructor
			return new Ctrl_Hover(args);
		}
	
		return NULL;
	}
}
