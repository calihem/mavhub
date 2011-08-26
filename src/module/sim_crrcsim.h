#ifndef _SIM_CRRCSIM_H_
#define _SIM_CRRCSIM_H_

#include "PID.h"

#include "Ivy/ivy.h"
#include "Ivy/ivyloop.h"
#include "Ivy/timer.h"

#include <inttypes.h> //uint8_t
#include "protocol/protocollayer.h"

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

			double phi, theta, psi;
			double x,y,z;
			PID* pid_alt;

			virtual void read_conf(const std::map<std::string, std::string> args);
			//virtual void handle_timer (TimerId id, void *data, unsigned long delta);

	};
	
	// ----------------------------------------------------------------------------
	// Sim_Crrcsimule
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif
