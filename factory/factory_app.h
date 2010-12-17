#ifndef _FACTORY_APP_H_
#define _FACTORY_APP_H_

#include "module/coremod.h"
#include "module/testcore.h"
#include "module/fc_mpkg.h"
#include "module/ctrl_hover.h"

namespace mavhub {

	class AppFactory {
	public:
		static AppLayer* build(const std::string& app_name, const std::map<std::string, std::string> args);
	};
}
#endif
