#ifndef _APP_FACTORY_H_
#define _APP_FACTORY_H_

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
