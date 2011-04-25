// talk to FC with mkpackage

#ifndef _CTRL_HOVER_H_
#define _CTRL_HOVER_H_

#include "debug_channels.h"
#include "core/logger.h"
#include "filter_kalmancv.h"
#include "stat_meanvar.h"
#include "PID.h"
#include "core/protocollayer.h"
#include "qk_helper.h"
#include "pp.h"
#include "pp_uss.h"
#include "pp_acc.h"
#include "pp_baro.h"
#include "pp_ir.h"

#include <math.h>

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
			USS_FC = 0,
			BARO = 1,
			ACC = 2,
			IR_SHARP_30_3V = 3,
			IR_SHARP_150_3V = 4,
			STATUS = 5,
			IR_SHARP_30_5V = 6,
			IR_SHARP_150_5V = 7,
			USS = 8
		};

		/// debugout type to index map
		enum mk_debugout_map_t {
			USSvalue = 0,
			USSlastvalid = 1,
			StickNick = 2, // XXX: changed in FC to sticknick, roll, yaw
			ADval_press = 3,
			ATTabsh = 4,
			StickRoll = 5,
			StickYaw = 6,
			USSstatus = 7,
			ADval_gyrroll = 8,
			ADval_gyrnick = 9,
			ADval_gyryaw = 10,
			ATTrelacctopint = 11,
			ADval_ubat = 12,
			GASmixfrac1 = 13,
			GASmixfrac2 = 14,
			RC_rssi = 15,
			ATTmeanaccnick = 16,
			ATTmeanaccroll = 17,
			ATTmeanacctop = 18,
			ATTintnickl = 19,
			ATTintrolll = 20,
			ATTintyawl = 21,
			FCParam_extctlswitch = 22,
			FCParam_gpsswitch = 23,
			ADval_accnick = 24,
			ADval_accroll = 25,
			ADval_acctop = 26,
			CTL_stickgas = 27,
			ADval_acctopraw = 28,
			ATTintnickh = 29,
			ATTintrollh = 30,
			ATTintyawh = 31
		};

  protected:
		/// this thread's main method
		virtual void run();
		/// preprocess raw sensor inputs
		virtual void preproc();
		/// reset barometer reference
		virtual void reset_baro_ref(double ref);
		/// pre-processing: use attitude for ranger correction
		virtual void att2dist(int chan);

  private:
		/// component id
		uint16_t component_id;

		// HUCH stuff
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
		/// local state
		mavlink_huch_ctrl_hover_state_t ctrl_hover_state;
		/// MK DebugOut structure
		mavlink_mk_debugout_t mk_debugout;
		/// MK FC Status
		mavlink_mk_fc_status_t mk_fc_status;
		/// huch hover ctrl (hc) raw altitude readings
		mavlink_huch_hc_raw_t hc_raw;

		// PIXHAWK stuff
		/// pixhawk raw imu
		mavlink_raw_imu_t raw_imu;
		/// pixhawk attitude
		mavlink_attitude_t ml_attitude;
		/// pixhawk manual control input
		mavlink_manual_control_t manual_control;
		/// pixhawk attitude controller output
		mavlink_attitude_controller_output_t attitude_controller_output;

		/// Kalman instance
		Kalman_CV* kal;
		/// PID instance (altitude)
		PID* pid_alt;
		/// number of sensor inputs processed by the module
		int numchan;
		/// parameters
		std::map<std::string, double>	params;
		std::list< std::pair<int, int> > typemap_pairs;
		std::vector<int> typemap;
		std::list< std::pair<int, int> > chanmap_pairs;
		std::vector<int> chanmap;
		/// raw sensor input array
		std::vector<int> raw;
		/// pre-processed sensor array: first: value, second: status/valid
		std::vector< std::pair< double, int > > pre;
		std::vector<double> premean;
		std::vector<double> precov;
		std::vector<Stat_MeanVar *> stats;
		/// preprocessor modules
		std::vector<PreProcessor *> premod;
		std::vector<double> uss_win, uss_win_sorted;
		int uss_win_idx;
		int uss_win_idx_m1;
		double uss_med, d_uss;
		/// statistics
		//std::list< std::vector <double> > stats;
		/* CvMat* stats; */
		/* CvMat* stats_data; */

		/// barometer reference
		double baro_ref;
		// config variables / parameters
		/// controller bias (hovergas)
		int ctl_bias;
		double ctl_Kc;
		double ctl_Ti;
		double ctl_Td;
		// int ctl_sp;
		// int ctl_bref;
		// int ctl_sticksp;
		/* double ctl_mingas; */
		/* double ctl_maxgas; */
		/* int prm_rst_baro_en; */
		// output
		int output_enable;
		int gs_enable; // groundstation enable generic mavlink data
		int gs_en_dbg; // groundstation enable debug data
		int gs_en_stats; // groundstation enable debug data
		// action requests
		int set_neutral_rq;


		// parameters
		int param_request_list;
		int param_count;		

		// update rate
		int ctl_update_rate;
		
		/// strapdown matrix setter
		int sd_comp_C(mavlink_huch_attitude_t *a, CvMat *C);

		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);
		/// send debug data
		void send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value);
		/// limit gas
		virtual int limit_gas(double gas);

		// MK debug structure conversion
		int mk_debugout_digital_offset;
		// fetch different types from byte based mk_debugout
		uint16_t debugout_getval_u(mavlink_mk_debugout_t* dbgout, int index);
		int16_t debugout_getval_s(mavlink_mk_debugout_t* dbgout, int index);
		int32_t debugout_getval_s32(mavlink_mk_debugout_t* dbgout, int indexl, int indexh);

		void debugout2attitude(mavlink_mk_debugout_t* dbgout);
		void debugout2altitude(mavlink_mk_debugout_t* dbgout);
		void debugout2exp_ctrl(mavlink_mk_debugout_t* dbgout, mavlink_huch_exp_ctrl_t* exp_ctrl);
		void debugout2ranger(mavlink_mk_debugout_t* dbgout, mavlink_huch_ranger_t* ranger);
		void debugout2status(mavlink_mk_debugout_t* dbgout, mavlink_mk_fc_status_t* status);

		void set_pxh_raw_imu();
		void set_pxh_attitude();
		void set_pxh_manual_control();

		double integrate_and_saturate(double offset, double d, double min, double max);
		double calc_var_nonlin(double var_e, double min, double max);
  };

	// limit gas
	inline int Ctrl_Hover::limit_gas(double gas) {
		if(gas < params["ctl_mingas"])
			gas = params["ctl_mingas"]; // mingas
		if (gas > params["ctl_maxgas"])
			gas = params["ctl_maxgas"]; // maxgas
		return gas;
	}

	// integrate and saturate
	inline double Ctrl_Hover::integrate_and_saturate(double offset, double d, double min, double max) {
		double tmp;
		//double max = 10.0;
		d -= 0.5; // shift
		d *= -1.0; // reverse
		tmp = offset + d;
		if(tmp <= min)
			return min;
		else if(tmp > max)
			return max;
		else
			return tmp;
	}

	// calculate nonlinearily scaled variance
	inline double Ctrl_Hover::calc_var_nonlin(double var_e, double min, double max) {
		double tmp;
		//    x1 = (x - lmin) / (lmax - lmin)
		//    return x1 + (np.exp(x1 * 7) - 1.0)
		// tmp = (var_e - min) / (max - min);
		tmp = var_e / (max - min);
		return tmp + (expf(tmp * 7.0) - 1.0);
	}
}

#endif
