#ifndef _FACTORY_APP_H_
#define _FACTORY_APP_H_

#include <string>
#include <map>

namespace mavhub {

class AppLayer;

class AppFactory {
	public:
		static AppLayer* build(const std::string& app_name, const std::map<std::string, std::string>& args);
};

}
#endif
