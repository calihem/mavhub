#ifndef _PLAT_LINK_CRRCSIM_H_
#define _PLAT_LINK_CRRCSIM_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#include "modulebase.h"
#include "PID.h"
#include "n_ff.h"
#include "Bumper.h"
#include "exec_timing.h"

/* #include "Ivy/ivy.h" */
/* #include "Ivy/ivyloop.h" */
/* #include "Ivy/timer.h" */

#include <inttypes.h> //uint8_t
#include "protocol/protocollayer.h"

namespace mavhub {
	class Plat_Link_Crrcsim : public ModuleBase {
		public:
			Plat_Link_Crrcsim(const std::map<std::string, std::string> args);
			virtual ~Plat_Link_Crrcsim();
			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();

	private:

			/// attitude variables
			double phi, theta, psi;
			/// position variables
			double x,y,z;
			/// position estimates
			double x_hat, y_hat, z_hat;
			/// measurements
			double m1, m2;
			/// altitude controller mode
			int ctl_mode;
			/// lateral controller mode
			int ctl_mode_lat;
			/// PID altitude controller
			PID* pid_alt;
			/* /// parameter dict */
			/* std::map<std::string, double>	params; */
			/* /// params requested for transmission */
			/* bool param_request_list; */
			/// execution timing
			Exec_Timing* exec_tmr;
			/// Bump controller module
			Bumper* bump;
			/// Bump controller module
			Bumper* bump_lat;
			/// direct thrust
			double thrust;
			/// direct roll
			double roll, pitch;
			/// test ffnet
			//N_FF* ffnet;

			// transmissibles
			// base message
			mavlink_message_t msg;
			// control message
			// mavlink_manual_control_t ctl;
			mavlink_huch_attitude_control_t ctl;
			// debug structure
			mavlink_debug_t dbg;


			////////////////////////////////////////////////////////////
			// methods

			/// read data from config
			virtual void conf_defaults();
			/* /// handle parameter list request */
			/* virtual void param_request_respond(); */
			/// ivy timer callback
			virtual void read_conf(const std::map<std::string, std::string> args);
			//virtual void handle_timer (TimerId id, void *data, unsigned long delta);
			/// send debug data
			void send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value);

	};
	
	// ----------------------------------------------------------------------------
	// Plat_Link_Crrcsim
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif // HAVE_MAVLINK_H

#endif
