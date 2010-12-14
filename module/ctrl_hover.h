// talk to FC with mkpackage

#ifndef _CTRL_HOVER_H_
#define _CTRL_HOVER_H_

#include "logger.h"
// #include "thread.h"
#include "filter_kalmancv.h"
#include "protocollayer.h"
#include "qk_helper.h"

namespace mavhub {
	/// Controller: hover (altitude)
  class Ctrl_Hover : public AppLayer {
  public:
		/// Constructor
		Ctrl_Hover();
		virtual ~Ctrl_Hover();
		/// mavhub protocolstack input handler
		virtual void handle_input(const mavlink_message_t &msg);

  protected:
		virtual void run();
  private:
		/// huch attitude struct
		mavlink_huch_attitude_t attitude;
		/// huch altitude struct
		mavlink_huch_fc_altitude_t altitude;
		/// huch exp ctrl struct
		mavlink_huch_exp_ctrl_t exp_ctrl;
		/// huch ranger(s) struct
		mavlink_huch_ranger_t ranger;
		/// MK external control
		extern_control_t extctrl;
		/// Kalman instance
		Kalman_CV* kal;
  };
}

#endif
