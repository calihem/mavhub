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
			return new Ctrl_Hover();
		}
	
		return NULL;
	}
}
