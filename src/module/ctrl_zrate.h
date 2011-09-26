// control zrate components: nick, roll, yaw

#ifndef _CTRL_ZRATE_H_
#define _CTRL_ZRATE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#ifdef HAVE_MKLINK_H
#include <mklink.h>

#include "debug_channels.h"
#include "core/logger.h"
#include "protocol/protocollayer.h"
#include "PID.h"

namespace mavhub {
	class Ctrl_Zrate : public AppLayer<mavlink_message_t>, public AppLayer<mk_message_t> {
	public:
		/// Ctor: zrate controller
		Ctrl_Zrate(const std::map<std::string, std::string>);
		/// Dtor
		virtual ~Ctrl_Zrate();
		/// protocol stack input handling
		virtual void handle_input(const mavlink_message_t &msg);
		virtual void handle_input(const mk_message_t &msg);

	protected:
		/// this thread's main method
		virtual void run();

	private:
		uint16_t component_id;
		/* int16_t nick; */
		/* int16_t roll; */
		/* int16_t yaw; */
		mavlink_huch_visual_navigation_t huch_visual_navigation;
		mavlink_manual_control_t manual_control;

		// params
		int param_request_list;
		std::map<std::string, double>	params;
		// int prm_test_nick;
		// double prm_yaw_P;
		//int prm_ni
		double zrate_sp;
		double zrate_av;
		double zrate_err;

		/// PID instance (z-rate)
		PID* pid_zrate;
		/* int ctl_bias; */
		/* double ctl_Kc; */
		/* double ctl_Ti; */
		/* double ctl_Td; */

		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);
	};
}

#endif // HAVE_MKLINK_H

#endif // HAVE_MAVLINK_H

#endif
