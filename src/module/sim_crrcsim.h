#ifndef _SIM_CRRCSIM_H_
#define _SIM_CRRCSIM_H_

#include <inttypes.h> //uint8_t
#include "core/protocollayer.h"

namespace mavhub {

	class Sim_Crrcsimule : public AppLayer {
		public:
			Sim_Crrcsimule(const std::map<std::string, std::string> args);
			virtual ~Sim_Crrcsimule();
			virtual void handle_input(const mavlink_message_t &msg);

		protected:
			virtual void run();

	private:
			uint16_t component_id;

			virtual void read_conf(const std::map<std::string, std::string> args);
	};
	
	// ----------------------------------------------------------------------------
	// Sim_Crrcsimule
	// ----------------------------------------------------------------------------


} // namespace mavhub

#endif
