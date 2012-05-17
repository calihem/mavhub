#ifndef _PLAT_LINK_MK_H_
#define _PLAT_LINK_MK_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#ifdef HAVE_MKHUCHLINK_H
// #include <mkhuchlink.h>

#include "modulebase.h"
// #include "PID.h"
// #include "n_ff.h"
#include "Bumper.h"
#include "exec_timing.h"

/* #include "Ivy/ivy.h" */
/* #include "Ivy/ivyloop.h" */
/* #include "Ivy/timer.h" */

#include <inttypes.h> //uint8_t
#include "protocol/protocollayer.h"

namespace mavhub {
	class Plat_Link_Mk : public ModuleBase, public AppLayer<mkhuch_message_t> {
		public:
			Plat_Link_Mk(const std::map<std::string, std::string> args);
			virtual ~Plat_Link_Mk();
			/// mavlink protocolstack input handler
			virtual void handle_input(const mavlink_message_t &msg);
			/// mkhuch protocolstack input handler
			virtual void handle_input(const mkhuch_message_t &msg);

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
			int ctl_mode_alt;
			/// lateral controller mode
			int ctl_mode_lat;
			/// yaw controller mode
			int ctl_mode_yaw;
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
			float thrust;
			/// direct roll
			float roll, pitch;
			/// direct yaw
			float yaw;
			/// mask
			uint8_t mask;
			/// altitude control active
			uint8_t ac_active;
			/// lateral control active
			uint8_t lc_active;

			// transmissibles
			// base message
			mavlink_message_t msg;
			/// attitude control message
			/* // mavlink_manual_control_t ctl; */
			/* mavlink_huch_attitude_control_t ctl; */
			// debug structure
			mavlink_debug_t dbg;

			/// mkhuch control output
			mkhuch_extern_control_t extern_control;
			/// Mutex to protect tx_mav_msg
			pthread_mutex_t tx_mav_mutex;
			/// tx buffer for MKHUCH messages
			mkhuch_message_t tx_mkhuch_msg;
			/// Mutex to protect tx_mkhuch_msg
			pthread_mutex_t tx_mkhuch_mutex;


			////////////////////////////////////////////////////////////
			// methods

			/// read data from config
			virtual void conf_defaults();
			/* /// handle parameter list request */
			/* virtual void param_request_respond(); */
			/// ivy timer callback
			virtual void read_conf(const std::map<std::string, std::string> args);
			//virtual void handle_timer (TimerId id, void *data, unsigned long delta);
			/* /// send debug data */
			/* void send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value); */

	};
	
	// ----------------------------------------------------------------------------
	// Plat_Link_Mk
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif // HAVE_MKHUCH_H
#endif // HAVE_MAVLINK_H

#endif
