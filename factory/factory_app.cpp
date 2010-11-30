#include "factory_app.h"

#include "protocollayer.h"

#include "module/coremod.h"
#include "module/testcore.h"
#include "module/fc_mpkg.h"
#include "module/ctrl_hover.h"
#include "app/mk_app.h"

#include <iostream>

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
		} else if(lowercase_name == "fc_mpkg_app") {
			int component_id = 0;
			std::map<std::string,std::string>::const_iterator iter = args.find("component_id");
			if( iter != args.end() ) {
				istringstream s(iter->second);
				s >> component_id;
			}
			return new FC_Mpkg(component_id);
		} else if(lowercase_name == "ctrl_hover_app") {
			return new Ctrl_Hover();
		} else if(lowercase_name == "mk_app") {
			return new MKApp(loglevel);
		}
	
		return NULL;
	}
}
