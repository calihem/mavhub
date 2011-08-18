/****************************************************************************
** Copyright 2011 Humboldt-Universitaet zu Berlin
**
** This file is part of MAVHUB.
**
** MAVHUB is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** MAVHUB is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with MAVHUB.  If not, see <http://www.gnu.org/licenses/>.
**
*****************************************************************************/
/**
 * \file ctrl_logfileplayer.cpp
 * \date created at 2011/04/20
 * \author Oswald Berthold
 *
 * \brief replay a logfile in qgroundcontrol format to MAVHUB
 *
 * \sa ctrl_logfileplayer.h
 * \sa ctrl_logger.h
 * \sa ctrl_logger.cpp
 */

// play back logfile and simulate kopter inputs

#include "ctrl_logfileplayer.h"

#include <mavlink.h>

#include "core/logger.h"
#include "utility.h"
#include "protocol/protocolstack.h"
#include "protocol/mkpackage.h"
#include "core/datacenter.h"

#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //us
#include <sstream>
#include <stdlib.h>


using namespace std;

namespace mavhub {
  Ctrl_LogfilePlayer::Ctrl_LogfilePlayer(const map<string, string> args) : 
		AppInterface("ctrl_logfileplayer"),
		AppLayer<mavlink_message_t>("ctrl_logfileplayer")
	{
		char header[1024];
		params["replay_mode"] = QGC;
		//params["replay_mode"] = CH;
		conf_defaults();

		read_conf(args);
		logfile_open(logfilename);
		sd.getline(header, 1024);
		param_request_list = 0;
		Logger::log("Ctrl_LogfilePlayer::init with header", header, Logger::LOGLEVEL_INFO);
  }

  Ctrl_LogfilePlayer::~Ctrl_LogfilePlayer() {
	}

	void Ctrl_LogfilePlayer::conf_defaults() {
		params["replay_mode"] = CH;
		params["offset"] = 0.0;
		params["timescale"] = 1.0;
		params["play"] = 1.0;
	}

	void Ctrl_LogfilePlayer::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}

		iter = args.find("replay_mode");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["replay_mode"];
		}

		iter = args.find("offset");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["offset"];
		}

		iter = args.find("logfilename");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> logfilename;
		}

		Logger::log("ctrl_logfileplayer::read_conf: component_id", component_id, Logger::LOGLEVEL_INFO);
		Logger::log("ctrl_logfileplayer::read_conf: logfilename", logfilename, Logger::LOGLEVEL_INFO);
	}

  void Ctrl_LogfilePlayer::handle_input(const mavlink_message_t &msg) {
		static int8_t param_id[15];
		/*
		 * - load logfile
		 * - restart
		 * - change replay_mode
		 */
		switch(msg.msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			Logger::log("Ctrl_Logfileplayer::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
				param_request_list = 1;
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
				Logger::log("Ctrl_Logfileplayer::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Ctrl_Logfileplayer::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log("Ctrl_Logfileplayer::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);

					typedef map<string, double>::const_iterator ci;
					for(ci p = params.begin(); p!=params.end(); ++p) {
						// Logger::log("ctrl_hover param test", p->first, p->second, Logger::LOGLEVEL_INFO);
						if(!strcmp(p->first.data(), (const char *)param_id)) {
							params[p->first] = (int)mavlink_msg_param_set_get_param_value(&msg);
							Logger::log("x Ctrl_Logfileplayer::handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_INFO);
						}
					}
				}
			}
			break;
		default:
			break;
		}
	}	

  void Ctrl_LogfilePlayer::run() {
		// char* logline;
		// char logline[1024];
		//int delay;
		//istringstream is;
		std::string token;
		std::string logline;
		char tmp[1024];
		int tmpi;
		int tabcount;
		int linecount;
		int t, tm1, dt, dtm1;
		int d; //, dtype; // data and datatype
		char dtype_s[64];
		uint8_t ready_to_send = 0;
		
		mavlink_huch_hc_raw_t ch_raw;
		mavlink_huch_ctrl_hover_state_t ch_state;
		mavlink_huch_attitude_t huch_attitude;
		mavlink_mk_fc_status_t fc_state;
		mavlink_mk_debugout_t debugout;
		mavlink_huch_visual_navigation_t huch_vn;
		mavlink_message_t msg;

		int status = 1;

		// heartbeat
		int system_type = MAV_QUADROTOR;
		mavlink_message_t msg_hb;
		mavlink_msg_heartbeat_pack(system_id(), component_id, &msg_hb, system_type, MAV_AUTOPILOT_HUCH);
		// "check in"
		AppLayer<mavlink_message_t>::send(msg_hb);

		t = tm1 = dt = dtm1 = 0;
		linecount = 0;
		d = 0;

		// initialize struct
		bzero(&ch_raw, sizeof(mavlink_huch_hc_raw_t));
		bzero(&ch_state, sizeof(mavlink_huch_ctrl_hover_state_t));
		bzero(&huch_attitude, sizeof(mavlink_huch_attitude_t));
		bzero(&fc_state, sizeof(mavlink_mk_fc_status_t));
		bzero(&debugout, sizeof(mavlink_mk_debugout_t));
		bzero(&huch_vn, sizeof(mavlink_huch_visual_navigation_t));

		while (status != 0) {

			// send data to groundstation
			// parameters if requested: parameter list and data
			if(param_request_list) {
				Logger::log("Ctrl_Logfileplayer::run: param request", Logger::LOGLEVEL_INFO);
				param_request_list = 0;

				typedef map<string, double>::const_iterator ci;
				for(ci p = params.begin(); p!=params.end(); ++p) {
					// Logger::log("ctrl_hover param test", p->first, p->second, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_value_pack(system_id(), component_id, &msg, (const int8_t*) p->first.data(), p->second, 1, 0);
					AppLayer<mavlink_message_t>::send(msg);
				}
			}

			// start reading line by line
			// i know it's more than ugly but wtf ...
			if(sd.is_open()) {
				if(!sd.eof()) {
					//sd >> logline;
					if(params["play"] < 1.0) {
						usleep(10000);
						continue;
					}

					getline(sd, logline);
					linecount++;

					// skip offset lines
					if(linecount < (int)params["offset"])
						continue;

					//Logger::log("Ctrl_LogfilePlayer: count, line", linecount, logline, Logger::LOGLEVEL_INFO);

					typedef string::const_iterator ci;
					//iter = logline.begin();
					//while(iter != logline.end()) {
					tabcount = 0;
					tmpi = 0;
					for(ci p = logline.begin(); p != logline.end(); ++p) {
						// accumulate data
						tmp[tmpi] = *p;
						tmpi++;

						if(*p == '\t') {
							tabcount++;
							tmp[tmpi] = '\0'; // proper c string termination

							// first data entry: accumulate time
							if(tabcount == 0) {
							}
							// time parsing finished
							else if(tabcount == 1 && *p == '\t') {
								tm1 = t;
								dtm1 = dt;
								t = atoi(tmp);
								// sometimes there are more than one package arriving
								// in the same timeslot
								if(tm1 != t)
									dt = t - tm1;
								else
									dt = dtm1;
								//printf("x1 time complete: string: '%s', t: %d, tm1: %d, dt: %d\n", tmp, t, tm1, dt);
								tmpi = 0; // reset counter
							}
							// data begins
							else if(tabcount > 1 && *p != ' ' && tmp[tmpi-2] != ' ') {
								int tmp_offset;
								d = atoi(tmp);
								//printf("x1.1 tmp:'%s'\n", tmp);
								if(in_range(tabcount, 2, 6)) {
									tmp_offset = 2;
									sprintf(dtype_s, "ch_raw");
									if(tabcount == tmp_offset) {
										ch_raw.usec = 0;
										ch_raw.raw0 = d;
									}
									else if(tabcount == tmp_offset+1) {
										ch_raw.raw1 = d;
									}
									else if(tabcount == tmp_offset+2) {
										ch_raw.raw2 = d;
									}
									else if(tabcount == tmp_offset+3) {
										ch_raw.raw3 = d;
									}
									else if(tabcount == tmp_offset+4) {
										ch_raw.raw4 = d;
										// struct finished, encode
										printf("ch_raw: %d, %d, %d, %d, %d\n",
													 ch_raw.raw0,
													 ch_raw.raw1,
													 ch_raw.raw2,
													 ch_raw.raw3,
													 ch_raw.raw4);
										mavlink_msg_huch_hc_raw_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &ch_raw);
										ready_to_send = 1;
										break; // terminate iteration over logline bytes
									}
								}
								else if(in_range(tabcount, 7, 14)) {
									tmp_offset = 7;
									sprintf(dtype_s, "ch_state");
									if(tabcount == tmp_offset) {
										ch_state.uss = atof(tmp);
									}
									else if(tabcount == tmp_offset+1) {
										ch_state.baro = atof(tmp);
									}
									else if(tabcount == tmp_offset+2) {
										ch_state.accz = atof(tmp);
									}
									else if(tabcount == tmp_offset+3) {
										ch_state.ir1 = atof(tmp);
									}
									else if(tabcount == tmp_offset+4) {
										ch_state.ir2 = atof(tmp);
									}
									else if(tabcount == tmp_offset+5) {
										ch_state.kal_s0 = atof(tmp);
									}
									else if(tabcount == tmp_offset+6) {
										ch_state.kal_s1 = atof(tmp);
									}
									else if(tabcount == tmp_offset+7) {
										ch_state.kal_s2 = atof(tmp);
										printf("ch_state: %f, %f, %f, %f, %f, %f, %f, %f\n",
													 ch_state.uss, ch_state.baro, ch_state.accz,
													 ch_state.ir1, ch_state.ir2, ch_state.kal_s0,
													 ch_state.kal_s1, ch_state.kal_s2);
										mavlink_msg_huch_ctrl_hover_state_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &ch_state);
										ready_to_send = 1;
										break;
									}
								}
								else if(in_range(tabcount, 15, 30)) {
									tmp_offset = 15;
									sprintf(dtype_s, "huch_attitude");
									if(tabcount == tmp_offset) {
										huch_attitude.xacc = atoi(tmp);
									}
									else if(tabcount == tmp_offset+1) {
										huch_attitude.yacc = atoi(tmp);
									}
									else if(tabcount == tmp_offset+2) {
										huch_attitude.zacc = atoi(tmp);
									}
									else if(tabcount == tmp_offset+3) {
										huch_attitude.zaccraw = atoi(tmp);
									}
									else if(tabcount == tmp_offset+4) {
										huch_attitude.xaccmean = atoi(tmp);
									}
									else if(tabcount == tmp_offset+5) {
										huch_attitude.yaccmean = atoi(tmp);
									}
									else if(tabcount == tmp_offset+6) {
										huch_attitude.zaccmean = atoi(tmp);
									}
									else if(tabcount == tmp_offset+7) {
										huch_attitude.xgyro = atoi(tmp);
									}
									else if(tabcount == tmp_offset+8) {
										huch_attitude.ygyro = atoi(tmp);
									}
									else if(tabcount == tmp_offset+9) {
										huch_attitude.zgyro = atoi(tmp);
									}
									else if(tabcount == tmp_offset+10) {
										huch_attitude.xgyroint = atoi(tmp);
									}
									else if(tabcount == tmp_offset+11) {
										huch_attitude.ygyroint = atoi(tmp);
									}
									else if(tabcount == tmp_offset+12) {
										huch_attitude.zgyroint = atoi(tmp);
									}
									else if(tabcount == tmp_offset+13) {
										huch_attitude.xmag = atoi(tmp);
									}
									else if(tabcount == tmp_offset+14) {
										huch_attitude.ymag = atoi(tmp);
									}
									else if(tabcount == tmp_offset+15) {
										huch_attitude.zmag = atoi(tmp);
										printf("huch_attitude: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
													 huch_attitude.xacc, huch_attitude.yacc, huch_attitude.zacc,
													 huch_attitude.zaccraw,
													 huch_attitude.xaccmean, huch_attitude.yaccmean, huch_attitude.zaccmean,
													 huch_attitude.xgyro, huch_attitude.ygyro, huch_attitude.zgyro,
													 huch_attitude.xgyroint, huch_attitude.ygyroint, huch_attitude.zgyroint,
													 huch_attitude.xmag, huch_attitude.ymag, huch_attitude.zmag
													 );
										mavlink_msg_huch_attitude_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &huch_attitude);
										ready_to_send = 1;
										break;
									}
								}
								else if(in_range(tabcount, 31, 36)) {
									tmp_offset = 31;
									sprintf(dtype_s, "fc_state");
									if(tabcount == tmp_offset) {
										fc_state.rssi = atoi(tmp);
									}
									else if(tabcount == tmp_offset+1) {
										fc_state.batt = atoi(tmp);
									}
									else if(tabcount == tmp_offset+2) {
										fc_state.nick = atoi(tmp);
									}
									else if(tabcount == tmp_offset+3) {
										fc_state.roll = atoi(tmp);
									}
									else if(tabcount == tmp_offset+4) {
										fc_state.yaw = atoi(tmp);
									}
									else if(tabcount == tmp_offset+5) {
										fc_state.gas = atoi(tmp);
										printf("fc_state: %d, %d, %d, %d, %d, %d\n",
													 fc_state.rssi, fc_state.batt,
													 fc_state.nick, fc_state.roll, fc_state.yaw,
													 fc_state.gas);
										mavlink_msg_mk_fc_status_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &fc_state);
										ready_to_send = 1;
										break;
									}
								}
								else if(in_range(tabcount, 37, 46)) {
									tmp_offset = 37;
									sprintf(dtype_s, "huch_visual_navigation");
									if(tabcount == tmp_offset) {
										huch_vn.alt_velocity = atof(tmp);
									}
									else if(tabcount == tmp_offset+1) {
										huch_vn.alt_absolute = atof(tmp);
									}
									else if(tabcount == tmp_offset+2) {
										huch_vn.home_beta = atof(tmp);
									}
									else if(tabcount == tmp_offset+3) {
										huch_vn.home_distance = atof(tmp);
									}
									else if(tabcount == tmp_offset+4) {
										huch_vn.visual_compass = atof(tmp);
									}
									else if(tabcount == tmp_offset+5) {
										huch_vn.ego_beta = atof(tmp);
									}
									else if(tabcount == tmp_offset+6) {
										huch_vn.ego_speed = atof(tmp);
									}
									else if(tabcount == tmp_offset+7) {
										huch_vn.keypoints = atoi(tmp);
									}
									else if(tabcount == tmp_offset+8) {
										huch_vn.error = atoi(tmp);
									}
									else if(tabcount == tmp_offset+9) {
										huch_vn.debug = atof(tmp);
										printf("%s: %f, %f, %f, %f, %f, %f, %f, %d, %d, %f\n",
													 dtype_s,
													 huch_vn.alt_velocity, huch_vn.alt_absolute,
													 huch_vn.home_beta, huch_vn.home_distance,
													 huch_vn.visual_compass,
													 huch_vn.ego_beta, huch_vn.ego_speed,
													 huch_vn.keypoints, huch_vn.error,
													 huch_vn.debug);
										mavlink_msg_huch_visual_navigation_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &huch_vn);
										ready_to_send = 1;
										break;
									}
								}
								else
									sprintf(dtype_s, "blub");
								tmpi = 0;
							}
						}
						//printf("x2 '%c', %d\n", *p, tabcount);
						

						// time is always first
						// if(t > 1000) // FIXME: 1kHz limit
						// else usleep(1000);

						// position where data begins determines packet type

					 	//Logger::log("Ctrl_LogfilePlayer: logfile_read", Logger::LOGLEVEL_INFO);
						//iter++;
					}

					// printf("x data:'%s': '%s', %d, %d, %d, %d\n", dtype_s, tmp, t, tm1, dt, d);
					//Logger::log("logfileplayer: sleeping", dt * 1000, linecount, Logger::LOGLEVEL_INFO);
					if(dt < 0 || linecount == (int)params["offset"] || linecount == 1) // generic catch
						dt = 1;
					//Logger::log("logfileplayer: sleeping", dt * 1000, linecount, Logger::LOGLEVEL_INFO);
					usleep(dt * 1000 * params["timescale"]); // 100 Hz

					if(params["replay_mode"] == QGC && ready_to_send > 0) {
						AppLayer<mavlink_message_t>::send(msg);
						ready_to_send = 0;
					}
					else if(params["replay_mode"] == CH && ready_to_send > 0) {
						//Logger::log("logfileplayer: preparing raw input simulation", Logger::LOGLEVEL_INFO);
						// 1. mk_debugout zusammenbauen und verschicken
						attitude2debugout(&huch_attitude, &debugout);
						mk_fc_state2debugout(&fc_state, &debugout);
						ch_raw2debugout(&ch_raw, &debugout);
						mavlink_msg_mk_debugout_encode(system_id(), static_cast<uint8_t>(component_id), &msg, &debugout);
						AppLayer<mavlink_message_t>::send(msg);
						// 2. DataCenter befummeln
						if(!strcmp(dtype_s, "ch_raw")) {
							Logger::log("logfileplayer: ch_raw hit", Logger::LOGLEVEL_INFO);
							ch_raw2datacenter(&ch_raw);
						}
					}
					
					// if(linecount > 10000)
					// 	exit(0);

					// std::istringstream is(std::string(logline));
					// is >> token;
					//Logger::log("Ctrl_LogfilePlayer: logfile_read token", token, Logger::LOGLEVEL_INFO);

					// FIXME: determine type of packet depending on start position,
					//        first nonempty after tab
				}
				else {
					Logger::log("Ctrl_LogfilePlayer: logfile_read EOF reached", Logger::LOGLEVEL_INFO);
					status = 0;
				}
			}
			else
				Logger::log("Ctrl_LogfilePlayer: logfile_read not open", Logger::LOGLEVEL_INFO);

		}
	}

	int Ctrl_LogfilePlayer::logfile_open(std::string logfile) {
		// fd = fopen(logfilename.c_str(), "r");
		// if (fd != NULL) return 1;
		// return 0;
		sd.open(logfile.c_str());
		Logger::log("Ctrl_LogfilePlayer::logfile_open", logfile, sd.is_open(), Logger::LOGLEVEL_INFO);
		return 0;
	}

	void Ctrl_LogfilePlayer::logfile_close() {
		Logger::log("Ctrl_LogfilePlayer: logfile_close", Logger::LOGLEVEL_INFO);
		// if(fd > 0)
		// 	fclose(fd);
		sd.close();
	}

	void Ctrl_LogfilePlayer::attitude2debugout(mavlink_huch_attitude_t* attitude, mavlink_mk_debugout_t* debugout) {
		debugout_setval_s(debugout, ADval_accnick, attitude->xacc);
		debugout_setval_s(debugout, ADval_accroll, attitude->yacc);
		debugout_setval_s(debugout, ADval_acctop, attitude->zacc);
		debugout_setval_s(debugout, ADval_acctopraw, attitude->zaccraw);
		debugout_setval_s(debugout, ATTmeanaccnick, attitude->xaccmean);
		debugout_setval_s(debugout, ATTmeanaccroll, attitude->yaccmean);
		debugout_setval_s(debugout, ATTmeanacctop, attitude->zaccmean);
		// watch out for the minus sign
		debugout_setval_s(debugout, ADval_gyrroll, -attitude->xgyro);
		debugout_setval_s(debugout, ADval_gyrnick, -attitude->ygyro);
		debugout_setval_s(debugout, ADval_gyryaw, -attitude->zgyro);
		debugout_setval_s32(debugout, ATTintrolll, ATTintrollh, -attitude->xgyroint);
		debugout_setval_s32(debugout, ATTintnickl, ATTintnickh, -attitude->ygyroint);
		debugout_setval_s32(debugout, ATTintyawl, ATTintyawh, -attitude->zgyroint);
		// debugout->xmag = 0;
		// debugout->ymag = 0;
		// debugout->zmag = 0;
	}
	void Ctrl_LogfilePlayer::mk_fc_state2debugout(mavlink_mk_fc_status_t* fc_status, mavlink_mk_debugout_t* debugout) {
		debugout_setval_s(debugout, RC_rssi, fc_status->rssi);
		debugout_setval_s(debugout, ADval_ubat, fc_status->batt);
		debugout_setval_s(debugout, GASmixfrac2, fc_status->gas);
		debugout_setval_s(debugout, StickNick, fc_status->nick);
		debugout_setval_s(debugout, StickRoll, fc_status->roll);
		debugout_setval_s(debugout, StickYaw, fc_status->yaw);
		// Logger::log("Ctrl_LogfilePlayer: mk_fc_state2debugout", fc_status->rssi, Logger::LOGLEVEL_INFO);
	}
	void Ctrl_LogfilePlayer::ch_raw2debugout(mavlink_huch_hc_raw_t* ch_raw, mavlink_mk_debugout_t* debugout) {
		debugout_setval_s(debugout, ATTabsh, ch_raw->raw1);
		debugout_setval_u(debugout, USSvalue, ch_raw->raw0);
	}
	void Ctrl_LogfilePlayer::ch_raw2datacenter(mavlink_huch_hc_raw_t* ch_raw) {
		// FIXME: chanmap
		// DataCenter::set_sensor(5, (double)ch_raw->raw0);
		DataCenter::set_sensor(1, (double)ch_raw->raw3);
		DataCenter::set_sensor(2, (double)ch_raw->raw4);
	}

  // set unsigned int in mk_debugout
	void Ctrl_LogfilePlayer::debugout_setval_u(mavlink_mk_debugout_t* dbgout, int index, uint16_t val) {
		int i;
		i = 2 * index + 2; //mk_debugout_digital_offset;
		dbgout->debugout[i+1] = (uint8_t)(val >> 8);
		dbgout->debugout[i] = (uint8_t)val;
	}
  // set unsigned int in mk_debugout
	void Ctrl_LogfilePlayer::debugout_setval_s(mavlink_mk_debugout_t* dbgout, int index, int16_t val) {
		int i;
		i = 2 * index + 2; // mk_debugout_digital_offset;
		dbgout->debugout[i+1] = (int8_t)(val >> 8);
		dbgout->debugout[i] = (uint8_t)val;
		//printf("debugout_setval_s: val: %d, dbgout: %d, %d\n", val, dbgout->debugout[i+1], dbgout->debugout[i]);
	}
  // set unsigned int in mk_debugout
	void Ctrl_LogfilePlayer::debugout_setval_s32(mavlink_mk_debugout_t* dbgout, int indexl, int indexh, int32_t val) {
		// FIXME: it's not correct
		int il, ih;
		il = 2 * indexl + 2;
		ih = 2 * indexh + 2;
		dbgout->debugout[ih+1] = (int8_t)(val >> 24);
		dbgout->debugout[ih] = (uint8_t)(val >> 16);
		dbgout->debugout[il+1] = (uint8_t)(val >> 8);
		dbgout->debugout[il] = (uint8_t)(val);
		// printf("setval_s32: %d, %d, %d, %d\n",
		// 			 dbgout->debugout[ih+1], dbgout->debugout[ih],
		// 			 dbgout->debugout[il+1], dbgout->debugout[il]
		// 			 );
	}
}
