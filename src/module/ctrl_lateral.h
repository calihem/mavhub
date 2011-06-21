// control lateral components: nick, roll, yaw

#ifndef _CTRL_LATERAL_H_
#define _CTRL_LATERAL_H_

#include "debug_channels.h"
#include "core/logger.h"
#include "core/protocollayer.h"

namespace mavhub {
	class Ctrl_Lateral : public AppLayer {
	public:
		/// Ctor: lateral controller
		Ctrl_Lateral(const std::map<std::string, std::string>);
		/// Dtor
		virtual ~Ctrl_Lateral();
		/// protocol stack input handling
		virtual void handle_input(const mavlink_message_t &msg);

	protected:
		/// this thread's main method
		virtual void run();

	private:
		uint16_t component_id;
		int16_t nick;
		int16_t roll;
		int16_t yaw;
		mavlink_huch_visual_navigation_t huch_visual_navigation;
		

		// params
		int param_request_list;
		int prm_test_nick;
		double prm_yaw_P;
		//int prm_ni

		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);
	};
}

#endif
