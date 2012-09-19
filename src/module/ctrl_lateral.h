// control lateral components: pitch, roll, yaw

#ifndef _CTRL_LATERAL_H_
#define _CTRL_LATERAL_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#include "modulebase.h"
#include "debug_channels.h"
#include "core/logger.h"
#include "protocol/protocollayer.h"
#include "PID.h"
#include "exec_timing.h"

namespace mavhub {
	class Ctrl_Lateral : public ModuleBase {
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
		int16_t pitch;
		int16_t roll;
		int16_t yaw;
#ifdef MAVLINK_ENABLED_HUCH
		mavlink_huch_visual_navigation_t huch_visual_navigation;
		mavlink_huch_visual_flow_t huch_visual_flow;
		mavlink_huch_generic_channel_t chan;
		mavlink_huch_sensor_array_t sensor_array;
#endif // MAVLINK_ENABLED_HUCH	

		// params
		// request
		int param_request_list;
		// container
		std::map<std::string, double>	params;
		int prm_test_pitch;
		double prm_yaw_P;
		double prm_pitch_P;
		double prm_roll_P;
		//int prm_ni
		// controller instances
		PID* pid_yaw;
		PID* pid_pitch;
		PID* pid_roll;
		/// execution timing
		Exec_Timing* exec_tmr;

		/// set reasonable config defaults
		virtual void default_conf();
		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);
	};
}

#endif // HAVE_MAVLINK_H

#endif
