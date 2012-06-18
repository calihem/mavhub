// control altitude, simple version
// IN
//  - measurements: m_0 ... m_n
// OUT
//  - thrust

#ifndef _CTRL_ALT_SIMPLE_H_
#define _CTRL_ALT_SIMPLE_H_

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
	class Ctrl_Alt_Simple : public ModuleBase {
	public:
		/// Ctor: alt_simple controller
		Ctrl_Alt_Simple(const std::map<std::string, std::string>);
		/// Dtor
		virtual ~Ctrl_Alt_Simple();
		/// protocol stack input handling
		virtual void handle_input(const mavlink_message_t &msg);

	protected:
		/// this thread's main method
		virtual void run();

	private:
		/* uint16_t component_id; */
		float thrust;
		
		mavlink_message_t msg;
		mavlink_huch_generic_channel_t chan;

#ifdef MAVLINK_ENABLED_HUCH
#endif // MAVLINK_ENABLED_HUCH	

		// params
		/* /// param request flag */
		/* int param_request_list; */
		/* // param container map */
		/* std::map<std::string, double>	params; */
		/// controller instances
		PID* pid_alt;
		/// execution timer
		Exec_Timing* tmr;
		/// altitude estimate
		double z_hat;

		/// set reasonable config defaults
		virtual void default_conf();
		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);
	};
}

#endif // HAVE_MAVLINK_H

#endif
