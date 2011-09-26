// talk to FC with mkpackage

#ifndef _FC_MPKG_H_
#define _FC_MPKG_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#include "core/logger.h"
// #include "thread.h"
#include "protocol/protocollayer.h"
//#include "qk_datatypes.h"

#define MKGYRO2RAD 1.52634e-05

namespace mavhub {
  // class FC_Mpkg : public cpp_pthread::PThread {
	/// FlightCtrl MKPackage Receiver
  class FC_Mpkg : public AppLayer<mavlink_message_t>, public AppLayer<mk_message_t> {
  public:
		/// Constructor
		FC_Mpkg(int component_id);
		virtual ~FC_Mpkg();
		/// mavhub protocolstack input handler
		virtual void handle_input(const mavlink_message_t &msg);
		virtual void handle_input(const mk_message_t &msg);

		uint16_t component_id;

		/// debugout type to index map
		enum mk_debugout_map_t {
			USSvalue = 0,
			USSlastvalid = 1,
			USScred = 2, // XXX: changed in FC to sticknick, roll, yaw
			ADval_press = 3,
			ATTabsh = 4,
			ATTrelh = 5,
			USSoffset = 6,
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
		virtual void run();
		virtual void publish_data(uint64_t time);

  private:
		// DebugOut_t *mk_debugout_o;
		// mavlink_message_t msg_j;
		/// MK DebugOut structure
		mavlink_mk_debugout_t mk_debugout;
		/// MK FC Status
		mavlink_mk_fc_status_t mk_fc_status;
		/// huch attitude struct
		mavlink_huch_attitude_t huch_attitude;
		mavlink_huch_fc_altitude_t huch_altitude;
		mavlink_huch_exp_ctrl_t huch_exp_ctrl;
		mavlink_huch_ranger_t huch_ranger;
		// pixhawk structs
		mavlink_raw_imu_t raw_imu;
		mavlink_attitude_t attitude;
		mavlink_manual_control_t manual_control;
		// void test();
		int mk_debugout_digital_offset;

		// fetch different types from byte based mk_debugout
		uint16_t debugout_getval_u(mavlink_mk_debugout_t* dbgout, int index);
		int16_t debugout_getval_s(mavlink_mk_debugout_t* dbgout, int index);
		int32_t debugout_getval_s32(mavlink_mk_debugout_t* dbgout, int indexl, int indexh);

		void debugout2attitude(mavlink_mk_debugout_t* dbgout, mavlink_huch_attitude_t* attitude);
		void debugout2altitude(mavlink_mk_debugout_t* dbgout, mavlink_huch_fc_altitude_t* altitude);
		void debugout2exp_ctrl(mavlink_mk_debugout_t* dbgout, mavlink_huch_exp_ctrl_t* exp_ctrl);
		void debugout2ranger(mavlink_mk_debugout_t* dbgout, mavlink_huch_ranger_t* ranger);
		void debugout2status(mavlink_mk_debugout_t* dbgout, mavlink_mk_fc_status_t* status);

		void set_pxh_raw_imu();
		void set_pxh_attitude();
		void set_pxh_manual_control();
  };
}

#endif // HAVE_MAVLINK_H

#endif
