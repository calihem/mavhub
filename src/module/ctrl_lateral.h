// control lateral components: nick, roll, yaw

#ifndef _CTRL_LATERAL_H_
#define _CTRL_LATERAL_H_

#include "debug_channels.h"
#include "core/logger.h"
#include "core/protocollayer.h"
#include "PID.h"

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
		// request
		int param_request_list;
		// container
		std::map<std::string, double>	params;
		int prm_test_nick;
		double prm_yaw_P;
		double prm_nick_P;
		double prm_roll_P;
		//int prm_ni
		// controller instances
		PID* pid_yaw;
		PID* pid_nick;
		PID* pid_roll;

		/// set reasonable config defaults
		virtual void default_conf();
		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);
	};
}

#endif
