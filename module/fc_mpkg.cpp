// talk to FC with mkpackage
#include "fc_mpkg.h"

#include <mavlink.h>
#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //us

#include <stdio.h>

#include "main.h" //system_id
#include "logger.h"
#include "utility.h"
#include "protocolstack.h"
#include "mkpackage.h"

using namespace std;

namespace mavhub {
  FC_Mpkg::FC_Mpkg() {
	 FC_Mpkg::mk_debugout_digital_offset = 2;
  }

  FC_Mpkg::~FC_Mpkg() {}

  void FC_Mpkg::handle_input(const mavlink_message_t &msg) {
	 vector<int> v(16);
	 Logger::log("FC_Mpkg got mavlink_message, len:", (int)msg.len, Logger::LOGLEVEL_INFO);
	 mavlink_msg_mk_debugout_decode(&msg, (mavlink_mk_debugout_t *)&mk_debugout);
	 debugout2attitude(&mk_debugout, &huch_attitude);
	 debugout2altitude(&mk_debugout, &huch_altitude);
	 debugout2ranger(&mk_debugout, &huch_ranger);
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
	 if(msg.sysid == owner->system_id() && msg.msgid == 0) {//FIXME: set right msgid
		//TODO
	 }
  }

  void FC_Mpkg::run() {
	 int buf[1];
	 buf[0] = 100;
	 MKPackage msg_debug_on(1, 'd', 2, buf);
	 owner->send(msg_debug_on);

	 Logger::debug("FC mk-pkg started, debug request sent to FC");
	 MKPackage msg_setneutral(1, 'c');
	 while(true) {
		// owner->send(msg_setneutral);
		// Logger::log("FC mk-pkg sent packet", Logger::LOGLEVEL_DEBUG);
		sleep(5);
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
	 attitude->xgyro    = v[7]  = FC_Mpkg::debugout_getval_s(dbgout, ADval_gyrnick);
	 attitude->ygyro    = v[8]  = FC_Mpkg::debugout_getval_s(dbgout, ADval_gyrroll);
	 attitude->zgyro    = v[9]  = FC_Mpkg::debugout_getval_s(dbgout, ADval_gyryaw);
	 attitude->xgyroint = v[10] = FC_Mpkg::debugout_getval_s32(dbgout, ATTintnickl, ATTintnickh);
	 attitude->ygyroint = v[11] = FC_Mpkg::debugout_getval_s32(dbgout, ATTintrolll, ATTintrollh);
	 attitude->zgyroint = v[12] = FC_Mpkg::debugout_getval_s32(dbgout, ATTintyawl, ATTintyawh);

	 Logger::log("debugout2attitude:", v,
					 Logger::LOGLEVEL_INFO);

	 // despair debug
	 // printf("blub: ");
	 // printf("%u,", (uint8_t)dbgout->debugout[0]);
	 // printf("%u,", (uint8_t)dbgout->debugout[1]);
	 // for(i = 0; i < 64; i++) {
	 // 	printf("%u,", (uint8_t)dbgout->debugout[i+2]);
	 // }
	 // printf("\n");

  }

  void FC_Mpkg::debugout2altitude(mavlink_mk_debugout_t* dbgout,
											 mavlink_huch_altitude_t* altitude) {
	 vector<uint16_t> v(2);
	 // XXX: use ADval_press
	 altitude->baro = v[0] = debugout_getval_u(dbgout, ATTabsh);
	 Logger::log("debugout2altitude:", v, Logger::LOGLEVEL_INFO);
  }

  void FC_Mpkg::debugout2ranger(mavlink_mk_debugout_t* dbgout,
											 mavlink_huch_ranger_t* ranger) {
	 vector<uint16_t> v(3);
	 ranger->ranger1 = v[0] = debugout_getval_u(dbgout, USSvalue);
	 Logger::log("debugout2ranger:", v, Logger::LOGLEVEL_INFO);
  }

  // fetch unsigned int from mk_debugout
  uint16_t FC_Mpkg::debugout_getval_u(mavlink_mk_debugout_t* dbgout,
												 int index) {
	 int i;
	 i = 2 * index + mk_debugout_digital_offset;
	 return (uint16_t)dbgout->debugout[i+1] << 8 |
		(uint8_t)dbgout->debugout[i];
  }

  // fetch signed int from mk_debugout
  int16_t FC_Mpkg::debugout_getval_s(mavlink_mk_debugout_t* dbgout,
												 int index) {
	 int i;
	 i = 2 * index + mk_debugout_digital_offset;
	 return (int16_t)dbgout->debugout[i+1] << 8 |
		(int8_t)dbgout->debugout[i];
  }

  // fetch signed int32 from mk_debugout
  int32_t FC_Mpkg::debugout_getval_s32(mavlink_mk_debugout_t* dbgout,
												 int indexl, int indexh) {
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
}
