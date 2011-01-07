// talk to FC with mkpackage

#ifndef _CTRL_HOVER_H_
#define _CTRL_HOVER_H_

#include "debug_channels.h"
#include "logger.h"
#include "filter_kalmancv.h"
#include "PID.h"
#include "protocollayer.h"
#include "qk_helper.h"
#include "pp.h"
#include "pp_uss.h"
#include "pp_acc.h"
#include "pp_baro.h"
#include "pp_ir.h"

namespace mavhub {
	/// Controller: hover (altitude)
  class Ctrl_Hover : public AppLayer {
  public:
		/// Constructor
		// Ctrl_Hover(int component_id_, int numchan, const std::list< std::pair<int, int> > chanmap, const std::map<std::string, std::string> args);
		Ctrl_Hover(const std::map<std::string, std::string> args);
		virtual ~Ctrl_Hover();
		/// mavhub protocolstack input handler
		virtual void handle_input(const mavlink_message_t &msg);
		/// sensor types
		enum sensor_types_t {
			USS,
			BARO,
			ACC,
			IR_SHARP_30_3V,
			IR_SHARP_150_3V,
			STATUS
		};

  protected:
		/// this thread's main method
		virtual void run();
		/// preprocess raw sensor inputs
		virtual void preproc();
		/// reset barometer reference
		virtual void reset_baro_ref(double ref);

  private:
		/// component id
		uint16_t component_id;
		/// huch attitude struct
		mavlink_huch_attitude_t attitude;
		/// mavlink attitude
		mavlink_attitude_t ml_attitude;
		/// huch altitude struct
		mavlink_huch_fc_altitude_t altitude;
		/// huch exp ctrl struct
		mavlink_huch_exp_ctrl_t exp_ctrl;
		/// huch ranger(s) struct
		mavlink_huch_ranger_t ranger;
		/// MK external control
		extern_control_t extctrl;
		/// local state
		mavlink_huch_ctrl_hover_state_t ctrl_hover_state;
		/// manual control input
		mavlink_manual_control_t manual_control;
		/// attitude controller output
		mavlink_attitude_controller_output_t attitude_controller_output;
		/// Kalman instance
		Kalman_CV* kal;
		/// PID instance (altitude)
		PID* pid_alt;
		/// number of sensor inputs processed by the module
		int numchan;
		std::list< std::pair<int, int> > chanmap_pairs;
		std::vector<int> chanmap;
		/// raw sensor input array
		std::vector<int> raw;
		/// pre-processed sensor array: first: value, second: status/valid
		std::vector< std::pair< double, int > > pre;
		/// preprocessor modules
		std::vector<PreProcessor *> premod;
		/// barometer reference
		double baro_ref;
		// config variables
		/// controller bias (hovergas)
		int ctl_bias;
		double ctl_Kc;
		double ctl_Ti;
		double ctl_Td;
		int ctl_sp;
		int ctl_bref;
		int ctl_sticksp;
		// output
		int output_enable;

		// parameters
		int param_request_list;
		int param_count;		

		/// strapdown matrix setter
		int sd_comp_C(mavlink_huch_attitude_t *a, CvMat *C);

		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);
		/// send debug data
		void send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value);
  };

}

#endif
