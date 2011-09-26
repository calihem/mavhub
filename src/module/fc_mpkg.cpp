// talk to FC with mkpackage
#include "fc_mpkg.h"

#ifdef HAVE_MAVLINK_H

#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //us

#include <stdio.h>

#include "core/logger.h"
#include "utility.h"
#include "protocol/protocolstack.h"
#include "protocol/mkpackage.h"
#include "core/datacenter.h"

using namespace std;

namespace mavhub {
  FC_Mpkg::FC_Mpkg(int component_id) :
	AppInterface("fc_mpkg"),
	AppLayer<mavlink_message_t>("fc_mpkg"),
	AppLayer<mk_message_t>("fc_mpkg") {
		FC_Mpkg::component_id = component_id;
		FC_Mpkg::mk_debugout_digital_offset = 2;
  }

  FC_Mpkg::~FC_Mpkg() {}

  void FC_Mpkg::handle_input(const mavlink_message_t &msg) {
		// vector<int> v(16);
		//mavlink_message_t msg_j;
		//Logger::log("FC_Mpkg got mavlink_message [len, id]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_DEBUG);

		if(msg.msgid == MAVLINK_MSG_ID_MK_DEBUGOUT) {
			//Logger::log("FC_Mpkg got MK_DEBUGOUT", Logger::LOGLEVEL_INFO);
			mavlink_msg_mk_debugout_decode(&msg, (mavlink_mk_debugout_t *)&mk_debugout);
			// MK FlightCtrl IMU data
			debugout2attitude(&mk_debugout, &huch_attitude);
			// MK FlightCtrl Barometric sensor data
			debugout2altitude(&mk_debugout, &huch_altitude);
			// MK huch-FlightCtrl I2C USS data
			// XXX: this should be in kopter config, e.g. if settings == fc_has_uss
			debugout2ranger(&mk_debugout, &huch_ranger);
			// real debugout data
			debugout2status(&mk_debugout, &mk_fc_status);

			// put into standard pixhawk structs
			// raw IMU
			//set_pxh_raw_imu();
			set_pxh_attitude();
			set_pxh_manual_control();

			publish_data(get_time_us());
			// deadlock problem
			// mavlink_msg_huch_attitude_encode(42, 23, &msg_j, &huch_attitude);
			// send(msg_j);
		}

		// // compare with qk_datatypes
		// mk_debugout_o = (DebugOut_t *)&mk_debugout;
		// v[0] = (int16_t)mk_debugout_o->Analog[ADval_accnick];
		// v[1] = (int16_t)mk_debugout_o->Analog[ADval_accroll];
		// v[2] = (int16_t)mk_debugout_o->Analog[ADval_acctop];
		// v[3] = (int16_t)mk_debugout_o->Analog[ADval_acctopraw];
		// v[4] = (int16_t)mk_debugout_o->Analog[ATTmeanaccnick];
		// v[5] = (int16_t)mk_debugout_o->Analog[ATTmeanaccroll];
		// v[6] = (int16_t)mk_debugout_o->Analog[ATTmeanacctop];
		// v[7] = (int16_t)mk_debugout_o->Analog[ADval_gyrnick];
		// v[8] = (int16_t)mk_debugout_o->Analog[ADval_gyrroll];
		// v[9] = (int16_t)mk_debugout_o->Analog[ADval_gyryaw];
		// Logger::log("FC_Mpkg decoded:  ", v, Logger::LOGLEVEL_INFO);
		// if(msg.sysid == system_id && msg.msgid == 0) {//FIXME: set right msgid
		// 	//TODO
		// }
  }

  void FC_Mpkg::handle_input(const mk_message_t &msg) { }

  void FC_Mpkg::run() {
		int buf[1];
		mavlink_message_t msg_i;

		Logger::log("FC_Mpkg starting", name(), Logger::LOGLEVEL_INFO);

		buf[0] = 10;
		// MKPackage msg_debug_on(1, 'd', 1, buf);
		// call constructor with: numdata (1), <buf(buf), buflen(1)> pairs
		mk_message_t msg_debug_on;
		mklink_msg_pack(&msg_debug_on, MK_FC_ADDRESS, MK_MSG_TYPE_POLL_DEBUG, buf, 1);
		sleep(3);
		AppLayer<mk_message_t>::send(msg_debug_on);
		Logger::log("FC_Mpkg debug request sent to FC", Logger::LOGLEVEL_INFO);
		// MKPackage msg_setneutral(1, 'c');
		while(true) {
			// send(msg_setneutral);
			// Logger::log("FC_Mpkg running", Logger::LOGLEVEL_INFO);
			// XXX: pass on data
			// 1. put into datacenter?
			// 2. retransmit onto protocolstack?

			mavlink_msg_huch_attitude_encode(system_id(), static_cast<uint8_t>(component_id), &msg_i, &huch_attitude);
			AppLayer<mavlink_message_t>::send(msg_i);
			mavlink_msg_huch_fc_altitude_encode(system_id(), static_cast<uint8_t>(component_id), &msg_i, &huch_altitude);
			AppLayer<mavlink_message_t>::send(msg_i);
			// mavlink_msg_huch_ranger_encode(system_id(), static_cast<uint8_t>(component_id), &msg_i, &huch_ranger);
			// send(msg_i);
			mavlink_msg_mk_fc_status_encode(system_id(), static_cast<uint8_t>(component_id), &msg_i, &mk_fc_status);
			AppLayer<mavlink_message_t>::send(msg_i);
			// send pixhawk std struct
			// mavlink_msg_raw_imu_encode(owner->system_id(), static_cast<uint8_t>(component_id), &msg_i, &raw_imu);
			// send(msg_i);
			// mavlink_msg_attitude_encode(owner->system_id(), static_cast<uint8_t>(component_id), &msg_i, &attitude);
			// owner->send(msg_i);
			mavlink_msg_manual_control_encode(system_id(), static_cast<uint8_t>(component_id), &msg_i, &manual_control);
			AppLayer<mavlink_message_t>::send(msg_i);

			// Logger::log("msg len", msg_i.len, Logger::LOGLEVEL_INFO);
			usleep(10000);
		}
		return;
  }

  void FC_Mpkg::debugout2attitude(mavlink_mk_debugout_t* dbgout,
																	mavlink_huch_attitude_t* attitude) {
		// int i;
		vector<int> v(16);
		// for mkcom mapping compatibility:
		// xaccmean, yaccmean, zacc
		attitude->xacc     = v[0]  = FC_Mpkg::debugout_getval_s(dbgout, ADval_accnick);
		attitude->yacc     = v[1]  = FC_Mpkg::debugout_getval_s(dbgout, ADval_accroll);
		attitude->zacc     = v[2]  = FC_Mpkg::debugout_getval_s(dbgout, ADval_acctop);
		attitude->zaccraw  = v[3]  = FC_Mpkg::debugout_getval_s(dbgout, ADval_acctopraw);
		attitude->xaccmean = v[4]  = FC_Mpkg::debugout_getval_s(dbgout, ATTmeanaccnick);
		attitude->yaccmean = v[5]  = FC_Mpkg::debugout_getval_s(dbgout, ATTmeanaccroll);
		attitude->zaccmean = v[6]  = FC_Mpkg::debugout_getval_s(dbgout, ATTmeanacctop);
		attitude->xgyro    = v[7]  = -FC_Mpkg::debugout_getval_s(dbgout, ADval_gyrroll);
		attitude->ygyro    = v[8]  = -FC_Mpkg::debugout_getval_s(dbgout, ADval_gyrnick);
		attitude->zgyro    = v[9]  = -FC_Mpkg::debugout_getval_s(dbgout, ADval_gyryaw);
		attitude->xgyroint = v[10] = -FC_Mpkg::debugout_getval_s32(dbgout, ATTintrolll, ATTintrollh);
		attitude->ygyroint = v[11] = -FC_Mpkg::debugout_getval_s32(dbgout, ATTintnickl, ATTintnickh);
		attitude->zgyroint = v[12] = -FC_Mpkg::debugout_getval_s32(dbgout, ATTintyawl, ATTintyawh);
		attitude->xmag = v[13] = 0.0;
		attitude->ymag = v[14] = 0.0;
		attitude->zmag = v[15] = 0.0;

		// Logger::log("debugout2attitude:", v,
		// 						Logger::LOGLEVEL_INFO);

		// despair debug
		// printf("blub: ");
		// printf("%u,", (uint8_t)dbgout->debugout[0]);
		// printf("%u,", (uint8_t)dbgout->debugout[1]);
		// for(i = 0; i < 64; i++) {
		// 	printf("%u,", (uint8_t)dbgout->debugout[i+2]);
		// }
		// printf("\n");

  }

  void FC_Mpkg::debugout2altitude(mavlink_mk_debugout_t* dbgout, mavlink_huch_fc_altitude_t* altitude) {
		vector<int16_t> v(2);
		// XXX: use ADval_press
		altitude->baro = v[0] = debugout_getval_s(dbgout, ATTabsh);
		// altitude->baro = v[0] = debugout_getval_u(dbgout, ADval_press);
		// Logger::log("debugout2altitude:", v, altitude->baro, Logger::LOGLEVEL_INFO);
  }

  void FC_Mpkg::debugout2ranger(mavlink_mk_debugout_t* dbgout, mavlink_huch_ranger_t* ranger) {
		vector<uint16_t> v(3);
		ranger->ranger1 = v[0] = debugout_getval_u(dbgout, USSvalue);
		// Logger::log("debugout2ranger:", v, Logger::LOGLEVEL_INFO);
  }

  void FC_Mpkg::debugout2status(mavlink_mk_debugout_t* dbgout, mavlink_mk_fc_status_t* status) {
		vector<uint16_t> v(3);
		status->rssi = v[0] = debugout_getval_s(dbgout, RC_rssi);
		status->batt = v[1] = debugout_getval_s(dbgout, ADval_ubat);
		status->gas  = v[2] = debugout_getval_s(dbgout, GASmixfrac2);
		//Logger::log("debugout2status:", v, Logger::LOGLEVEL_INFO);
  }

	// copy huch data into std pixhawk raw_imu
	void FC_Mpkg::set_pxh_raw_imu() {
		raw_imu.usec = 0; // get_time_us(); XXX: qgc bug
		raw_imu.xacc = huch_attitude.xacc;
		raw_imu.yacc = huch_attitude.yacc;
		raw_imu.zacc = huch_attitude.zaccraw;
		raw_imu.xgyro = huch_attitude.xgyro;
		raw_imu.ygyro = huch_attitude.ygyro;
		raw_imu.zgyro = huch_attitude.zgyro;
		raw_imu.xmag = huch_attitude.xmag;
		raw_imu.ymag = huch_attitude.ymag;
		raw_imu.zmag = huch_attitude.zmag;
	}

	// copy huch data into std pixhawk attitude
	void FC_Mpkg::set_pxh_attitude() {
		attitude.usec = 0; // get_time_us(); XXX: qgc bug
		attitude.roll  = huch_attitude.xgyroint * MKGYRO2RAD;
		attitude.pitch = huch_attitude.ygyroint * MKGYRO2RAD;
		attitude.yaw   = huch_attitude.zgyroint * MKGYRO2RAD;
		attitude.rollspeed  = huch_attitude.xgyro * MKGYRO2RAD;
		attitude.pitchspeed = huch_attitude.ygyro * MKGYRO2RAD;
		attitude.yawspeed   = huch_attitude.zgyro * MKGYRO2RAD;
	}

	// copy huch data into std pixhawk attitude
	void FC_Mpkg::set_pxh_manual_control() {
		manual_control.target = system_id();
		manual_control.thrust = (float)debugout_getval_u(&mk_debugout, CTL_stickgas);
	}

  // fetch unsigned int from mk_debugout
  uint16_t FC_Mpkg::debugout_getval_u(mavlink_mk_debugout_t* dbgout, int index) {
		int i;
		i = 2 * index + mk_debugout_digital_offset;
		return (uint16_t)dbgout->debugout[i+1] << 8 |
			(uint8_t)dbgout->debugout[i];
  }

  // fetch signed int from mk_debugout
  int16_t FC_Mpkg::debugout_getval_s(mavlink_mk_debugout_t* dbgout, int index) {
		int i;
		i = 2 * index + mk_debugout_digital_offset;
		return (int16_t)dbgout->debugout[i+1] << 8 |
			(uint8_t)dbgout->debugout[i]; // unsigned part
  }

  // fetch signed int32 from mk_debugout
  int32_t FC_Mpkg::debugout_getval_s32(mavlink_mk_debugout_t* dbgout, int indexl, int indexh) {
		int il, ih;
		uint16_t ul, uh;
		il = 2 * indexl + mk_debugout_digital_offset;
		ih = 2 * indexh + mk_debugout_digital_offset;
		ul = (uint16_t)dbgout->debugout[il+1] << 8 |
			(uint8_t)dbgout->debugout[il];
		uh = (uint16_t)dbgout->debugout[ih+1] << 8 |
			(uint8_t)dbgout->debugout[ih];

		return (int32_t) uh << 16 | ul;
	}

	void FC_Mpkg::publish_data(uint64_t time) {
		//DataCenter::set_huch_attitude(huch_attitude);
		//DataCenter::set_huch_fc_altitude(huch_altitude);
		// XXX: hardware specific mapping
		DataCenter::set_huch_ranger_at(huch_ranger, 0);
		//DataCenter::set_mk_fc_status(mk_fc_status);
	}

}

#endif // HAVE_MAVLINK_H

