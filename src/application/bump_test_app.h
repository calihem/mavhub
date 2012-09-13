#ifndef _BUMP_TEST_APP_H_
#define _BUMP_TEST_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H

#include "protocol/protocollayer.h"
#include "module/exec_timing.h"

#define BUMP_TEST_LOG 1

namespace mavhub {

class BumpTestApp : public MavlinkAppLayer {
	public:
		BumpTestApp(const std::map<std::string, std::string> &args, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
		virtual ~BumpTestApp();

		virtual void handle_input(const mavlink_message_t &msg);

	protected:
		virtual void run();

	private:
		unsigned int target_system;
		unsigned int target_component;
    double altitude;
    bool do_bump;
    Exec_Timing execTiming;
#ifdef BUMP_TEST_LOG
		std::ofstream logFile;
#endif
};

} // namespace mavhub

#endif // HAVE_MAVLINK_H
#endif // _BUMP_TEST_APP_H_
