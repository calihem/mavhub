#ifndef _SIM_CRRCSIM_H_
#define _SIM_CRRCSIM_H_

#include "PID.h"
#include "n_ff.h"
#include "Bumper.h"
#include "exec_timing.h"

/* #include "Ivy/ivy.h" */
/* #include "Ivy/ivyloop.h" */
/* #include "Ivy/timer.h" */

#include <inttypes.h> //uint8_t
#include "protocol/protocollayer.h"

// controller modes
enum ctl_mode_t {
	CTL_MODE_NULL, // do nothing
	CTL_MODE_BUMP, // do system bumping
	CTL_MODE_AC,   // dpo altitude control
	CTL_MODE_DIRECT, // directly provide thrust value
	CTL_MODE_NMODES // number of items in enum
};

namespace mavhub {

	class Sim_Crrcsimule : public AppLayer<mavlink_message_t> {
		public:
			Sim_Crrcsimule(const std::map<std::string, std::string> args);
			virtual ~Sim_Crrcsimule();
			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();

	private:
			uint16_t component_id;

			/// attitude variables
			double phi, theta, psi;
			/// position variables
			double x,y,z;
			/// position estimates
			double x_hat, y_hat, z_hat;
			/// measurements
			double m1, m2;
			/// controller mode
			int ctl_mode;
			/// PID altitude controller
			PID* pid_alt;
			/// parameter dict
			std::map<std::string, double>	params;
			/// params requested for transmission
			bool param_request_list;
			/// execution timing
			Exec_Timing* exec_tmr;
			/// Bump controller module
			Bumper* bump;
			/// direct thrust
			double thrust;
			/// test ffnet
			//N_FF* ffnet;

			////////////////////////////////////////////////////////////
			// methods

			/// read data from config
			virtual void conf_defaults();
			/// handle parameter list request
			virtual void param_request_respond(bool param_request);
			/// ivy timer callback
			virtual void read_conf(const std::map<std::string, std::string> args);
			//virtual void handle_timer (TimerId id, void *data, unsigned long delta);
			/// send debug data
			void send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value);

	};
	
	// ----------------------------------------------------------------------------
	// Sim_Crrcsimule
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif
