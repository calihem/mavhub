// talk to FC with mkpackage

#ifndef _CTRL_BUMP_H_
#define _CTRL_BUMP_H_

#include "debug_channels.h"
#include "core/logger.h"
// #include "thread.h"
#include "protocol/protocollayer.h"
#include "qk_helper.h"

namespace mavhub {
	/// Controller: system bumping (gas)
  class Ctrl_Bump : public AppLayer<mavlink_message_t>, public AppLayer<mk_message_t> {
  public:
		/// Constructor
		Ctrl_Bump(const std::map<std::string, std::string> args);
		virtual ~Ctrl_Bump();
		/// mavhub protocolstack input handler
		virtual void handle_input(const mavlink_message_t &msg);
		virtual void handle_input(const mk_message_t &msg);

  protected:
		/// this thread's main method
		virtual void run();

  private:
		/// component id
		uint16_t component_id;
		/// MK external control
		extern_control_t extctrl;
		// config variables
		int gdt_enable;
		uint64_t gdt_t0;
		int gdt_delay;
		double gdt_gas;
		// output
		int output_enable;
		/// controller bias (hovergas)
		/* int ctl_bias; */
		/* double ctl_Kc; */
		/* double ctl_Ti; */
		/* double ctl_Td; */
		/* int ctl_sp; */
		/* int ctl_bref; */
		/* int ctl_sticksp; */

		// parameters
		int param_count;

		virtual void read_conf(const std::map<std::string, std::string> args);
		virtual void param_list();
		virtual double gdt_eval(uint64_t dt);
  };
}

#endif
