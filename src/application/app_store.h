#ifndef _APP_STORE_H_
#define _APP_STORE_H_

#include <string>
#include <map>

namespace mavhub {

class AppStore {
	public:
		/**
		 * \brief Creates an instance of AppLayer and register it on protocol stack. 
		 */
		static int order(const std::string& app_name, const std::map<std::string, std::string>& args);
};

} // namespace mavhub
#endif
