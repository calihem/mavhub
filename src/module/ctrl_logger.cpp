// log data prior to controller as csv/tsv text file
//  - FIXME: logfile type settable (atm it is QGC format only),
//    options are raw mavlink (but that incompatible with the
//    use of datacenter), and "one line per packet, with timestamp
//    and type header"
//  - FIXME: generic mechanism to add messages
//  - FIXME: add raw debugout

#include "ctrl_logger.h"

#ifdef HAVE_MAVLINK_H

#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //us

#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <sstream>
#include <cstdlib>      // exit

#include "core/logger.h"
#include "lib/hub/utility.h"
#include "protocol/protocolstack.h"
#include "protocol/mkpackage.h"
#include "core/datacenter.h"

using namespace std;

namespace mavhub {
  Ctrl_Logger::Ctrl_Logger(const map<string, string> args) : 
		AppInterface("ctrl_logger"),
		ModuleBase(args, "ctrl_logger"),
		logging(false)
	{
		// char outstr[200];
		// struct timeval tv;

		read_conf(args);
		Logger::log("Ctrl_Logger: created", Logger::LOGLEVEL_INFO);
  }

  Ctrl_Logger::~Ctrl_Logger() {
		logf_close();
	}

	void Ctrl_Logger::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}

		// create logging param
		params["logging"] = 0.;

		iter = args.find("logmode");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> params["logmode"];
		}

		// FIXME: text or raw / binary
		Logger::log("ctrl_logger::read_conf: component_id", component_id, Logger::LOGLEVEL_INFO);
	}
	
  void Ctrl_Logger::handle_input(const mavlink_message_t &msg) {
		static char param_id[16];
		// char outstr[200];
		// static int8_t param_id[15];
		// int field_index_start;
		// int field_index_end;
		// time_t t;
		// struct tm *tmp;

		switch(msg.msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: // get param list
			Logger::log("Ctrl_Logger::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
			if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
				param_request_list = true;
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET: // handle parameter set message
			if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
				Logger::log("Ctrl_Logger::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
				if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
					Logger::log("Ctrl_Logger::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
					mavlink_msg_param_set_get_param_id(&msg, param_id);
					Logger::log("Ctrl_Logger::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);

					typedef map<string, double>::const_iterator ci;
					for(ci p = params.begin(); p!=params.end(); ++p) {
						if(!strcmp(p->first.data(), (const char *)param_id)) {
							if(p->first == string("logging")) {
								if(params["logging"] == 0. && mavlink_msg_param_set_get_param_value(&msg) > 0)
									log_init();
								else if (params["logging"] > 0. && mavlink_msg_param_set_get_param_value(&msg) == 0) {
									log_deinit();
								}
							}
							params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
							Logger::log("x Ctrl_Logger::handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_INFO);
						}
					}
				}
			}
			break;
		default:
			break;
		}

		handle_logdata(msg);

		// }
	}

	void Ctrl_Logger::handle_logdata(const mavlink_message_t &msg) {
		struct timeval tv;
		int ms; // in milliseconds

		gettimeofday(&tv, NULL);
		ms = ((tv.tv_sec - starttime) * 1000000 + tv.tv_usec) / 1000;

		switch((int)params["logmode"]) {
		case 0:
			handle_logdata_0(ms, msg);
			break;
		case 1:
			handle_logdata_1(ms, msg);
			break;
		default:
			break;
		}
	}

	void Ctrl_Logger::handle_logdata_1(int ms, const mavlink_message_t &msg) {
		if(!logging)
			return;
		// printf("blub\n");
		// int i;
		char outstr[200];

		datavals[0] = (double)ms;

		switch(msg.msgid) {

		case MAVLINK_MSG_ID_DEBUG:
			if(msg.sysid == 39 && msg.compid == 41) { // component 41
				if(mavlink_msg_debug_get_ind(&msg) == 0) {
					datavals[1] = mavlink_msg_debug_get_value(&msg);
				}
			}
			else if(msg.sysid == 39 && msg.compid == 28) { // component 44
				if(mavlink_msg_debug_get_ind(&msg) == 1) {
					datavals[2] = mavlink_msg_debug_get_value(&msg);
				}
			}
			break;

		case MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
			datavals[3] = mavlink_msg_global_vision_position_estimate_get_x(&msg);
			datavals[4] = mavlink_msg_global_vision_position_estimate_get_y(&msg);//"M39:GLOBAL_VISION_POSITION_ESTIMATE.y";
			datavals[5] = mavlink_msg_global_vision_position_estimate_get_z(&msg);//"M39:GLOBAL_VISION_POSITION_ESTIMATE.z";
			break;
		case MAVLINK_MSG_ID_ATTITUDE:
			datavals[15] = mavlink_msg_attitude_get_roll(&msg);
			datavals[16] = mavlink_msg_attitude_get_pitch(&msg);
			datavals[17] = mavlink_msg_attitude_get_yaw(&msg);
			break;
#ifdef MAVLINK_ENABLED_HUCH
		case MAVLINK_MSG_ID_HUCH_ATTITUDE_CONTROL:
			datavals[6] = mavlink_msg_huch_attitude_control_get_pitch(&msg); //"M39:HUCH_ATTITUDE_CONTROL.pitch";
			datavals[7] = mavlink_msg_huch_attitude_control_get_roll(&msg); //"M39:HUCH_ATTITUDE_CONTROL.roll";
			datavals[8] = mavlink_msg_huch_attitude_control_get_yaw(&msg); //"M39:HUCH_ATTITUDE_CONTROL.yaw";
			datavals[9] = mavlink_msg_huch_attitude_control_get_thrust(&msg); // "M39:HUCH_ATTITUDE_CONTROL.thrust";
			break;
		case MAVLINK_MSG_ID_HUCH_CTRL_HOVER_STATE:
			datavals[10] = mavlink_msg_huch_ctrl_hover_state_get_kal_s0(&msg); // "M39:HUCH_CTRL_HOVER_STATE.kal_s0";
			break;
		case MAVLINK_MSG_ID_HUCH_VISUAL_FLOW:
			datavals[11] = mavlink_msg_huch_visual_flow_get_u_i(&msg); //"M39:HUCH_VISUAL_FLOW.u_i";
			datavals[12] = mavlink_msg_huch_visual_flow_get_v_i(&msg); //"M39:HUCH_VISUAL_FLOW.v_i";
			datavals[13] = mavlink_msg_huch_visual_flow_get_u(&msg); //"M39:HUCH_VISUAL_FLOW.u";
			datavals[14] = mavlink_msg_huch_visual_flow_get_v(&msg); //"M39:HUCH_VISUAL_FLOW.v";
			//Logger::log("Ctrl_Logger flow debug:", datavals[13], datavals[14], Logger::LOGLEVEL_INFO);
			break;
#endif
		default:
			break;
		}		

		logline_preamble(1, ms);
		sprintf(outstr, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
						datavals[1],
						datavals[2],
						datavals[3],
						datavals[4],
						datavals[5],
						datavals[6],
						datavals[7],
						datavals[8],
						datavals[9],
						datavals[10],
						datavals[11],
						datavals[12],
						datavals[13],
						datavals[14],
						datavals[15],
						datavals[16],
						datavals[17]
						);
		fwrite(outstr, strlen(outstr), 1, fd);

		// for(i = 0; i < 13; i++) {
		// printf("%f, ", datavals[i]);
		// }
		// printf("\n");
	}

	void Ctrl_Logger::handle_logdata_0(int ms, const mavlink_message_t &msg) {
		if(!logging)
			return;

		char outstr[200];
		switch(msg.msgid) {
#ifdef MAVLINK_ENABLED_HUCH
		case MAVLINK_MSG_ID_HUCH_HC_RAW:
			//Logger::log("Ctrl_Logger got huch_hc_raw msg [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_INFO);

			if(logging) {
				logline_preamble(1, ms);

				sprintf(outstr, "%d\t%d\t%d\t%d\t%d\t",
								mavlink_msg_huch_hc_raw_get_raw0(&msg),
								mavlink_msg_huch_hc_raw_get_raw1(&msg),
								mavlink_msg_huch_hc_raw_get_raw2(&msg),
								mavlink_msg_huch_hc_raw_get_raw3(&msg),
								mavlink_msg_huch_hc_raw_get_raw4(&msg)
								);
				fwrite(outstr, strlen(outstr), 1, fd);

				logline_coda(5);
			}

			break;

		case MAVLINK_MSG_ID_HUCH_CTRL_HOVER_STATE:
			//Logger::log("Ctrl_Logger got huch_ctrl_hover_state msg [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_INFO);

			if(logging) {
				logline_preamble(6, ms);

				sprintf(outstr, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t",
								mavlink_msg_huch_ctrl_hover_state_get_uss(&msg),
								mavlink_msg_huch_ctrl_hover_state_get_baro(&msg),
								mavlink_msg_huch_ctrl_hover_state_get_accz(&msg),
								mavlink_msg_huch_ctrl_hover_state_get_ir1(&msg),
								mavlink_msg_huch_ctrl_hover_state_get_ir2(&msg),
								mavlink_msg_huch_ctrl_hover_state_get_kal_s0(&msg),
								mavlink_msg_huch_ctrl_hover_state_get_kal_s1(&msg),
								mavlink_msg_huch_ctrl_hover_state_get_kal_s2(&msg)
								);
				fwrite(outstr, strlen(outstr), 1, fd);

				logline_coda(13);
			}
			break;
		case MAVLINK_MSG_ID_HUCH_ATTITUDE:
			//Logger::log("Ctrl_Logger got huch_attitude msg [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_INFO);

			if(logging) {
				logline_preamble(14, ms);

				sprintf(outstr, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t",
								mavlink_msg_huch_attitude_get_xacc(&msg),
								mavlink_msg_huch_attitude_get_yacc(&msg),
								mavlink_msg_huch_attitude_get_zacc(&msg),
								mavlink_msg_huch_attitude_get_zaccraw(&msg),
								mavlink_msg_huch_attitude_get_xaccmean(&msg),
								mavlink_msg_huch_attitude_get_yaccmean(&msg),
								mavlink_msg_huch_attitude_get_zaccmean(&msg),
								mavlink_msg_huch_attitude_get_xgyro(&msg),
								mavlink_msg_huch_attitude_get_ygyro(&msg),
								mavlink_msg_huch_attitude_get_zgyro(&msg),
								mavlink_msg_huch_attitude_get_xgyroint(&msg),
								mavlink_msg_huch_attitude_get_ygyroint(&msg),
								mavlink_msg_huch_attitude_get_zgyroint(&msg),
								mavlink_msg_huch_attitude_get_xmag(&msg),
								mavlink_msg_huch_attitude_get_ymag(&msg),
								mavlink_msg_huch_attitude_get_zmag(&msg)
								);
				fwrite(outstr, strlen(outstr), 1, fd);

				logline_coda(29);
			}
			break;
		case MAVLINK_MSG_ID_MK_FC_STATUS:
			//Logger::log("Ctrl_Logger got huch_attitude msg [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_INFO);

			if(logging) {
				logline_preamble(30, ms);

				sprintf(outstr, "%d\t%d\t%d\t%d\t%d\t%d\t",
								mavlink_msg_mk_fc_status_get_rssi(&msg),
								mavlink_msg_mk_fc_status_get_batt(&msg),
								mavlink_msg_mk_fc_status_get_nick(&msg),
								mavlink_msg_mk_fc_status_get_roll(&msg),
								mavlink_msg_mk_fc_status_get_yaw(&msg),
								mavlink_msg_mk_fc_status_get_gas(&msg)
								);
				fwrite(outstr, strlen(outstr), 1, fd);

				logline_coda(35);
			}
			break;
		case MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION:
			// mavlink_huch_visual_navigation_t
			// FIXME: implement
			if(logging) {
				logline_preamble(36, ms);

				sprintf(outstr, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d\t%f\t",
								mavlink_msg_huch_visual_navigation_get_alt_velocity(&msg),
								mavlink_msg_huch_visual_navigation_get_alt_absolute(&msg),
								mavlink_msg_huch_visual_navigation_get_home_beta(&msg),
								mavlink_msg_huch_visual_navigation_get_home_distance(&msg),
								mavlink_msg_huch_visual_navigation_get_visual_compass(&msg),
								mavlink_msg_huch_visual_navigation_get_ego_beta(&msg),
								mavlink_msg_huch_visual_navigation_get_ego_speed(&msg),
								mavlink_msg_huch_visual_navigation_get_keypoints(&msg),
								mavlink_msg_huch_visual_navigation_get_error(&msg),
								mavlink_msg_huch_visual_navigation_get_debug(&msg)
								);
				fwrite(outstr, strlen(outstr), 1, fd);

				logline_coda(45);
			}
			break;
#endif // MAVLINK_ENABLED_HUCH
		default:
			break;
		}
}

  void Ctrl_Logger::run() {
		
		while (true) {
			param_request_respond();

			usleep(1000000); // 1 Hz
			if(logging)
				fflush(fd);
		}
	}

	void Ctrl_Logger::log_init() {
		struct timeval tv;
		char outstr[200];

		logf_genfilename();
		if(logf_open() > 0) {
			logging = true;
		}

		if(!logging)
			return;

		genheader();

		// loop over logged variables
		typedef map<int, string>::const_iterator ci;
		for(ci p = datafields.begin(); p!=datafields.end(); ++p) {
			Logger::log("Ctrl_Logger::log_init: datafield test", p->first, p->second, Logger::LOGLEVEL_INFO);
			sprintf(outstr, "%s", p->second.c_str());
			fwrite(outstr, strlen(outstr), 1, fd);
			sprintf(outstr, "\t");
			fwrite(outstr, strlen(outstr), 1, fd);
		}
		sprintf(outstr, "\n");
		fwrite(outstr, strlen(outstr), 1, fd);

		// sprintf(outstr, "unix_timestamp\t");
		// fwrite(outstr, strlen(outstr), 1, fd);
		// sprintf(outstr, "hc_raw0\t");
		// fwrite(outstr, strlen(outstr), 1, fd);
		// sprintf(outstr, "\n");
		// fwrite(outstr, strlen(outstr), 1, fd);
		fflush(fd);
		
		gettimeofday(&tv, NULL);
		starttime = tv.tv_sec;

		Logger::log(name(), "[datafields.size(), starttime]", datafields.size(), starttime, Logger::LOGLEVEL_INFO);
	}

	void Ctrl_Logger::log_deinit() {
		logging = false;
		logf_close();
	}

	void Ctrl_Logger::genheader() {
		int i;
		switch((int)params["logmode"]) {
		case 0: // original logging mode with separate hardcoded
			//       mavlink structs per line, other fields empty
			datafields[0] = "unix_timestamp";

			// hc raw
			datafields[1] = "ch_raw0";
			datafields[2] = "ch_raw1";
			datafields[3] = "ch_raw2";
			datafields[4] = "ch_raw3";
			datafields[5] = "ch_raw4";

			// hc state
			datafields[6] = "ch_uss";
			datafields[7] = "ch_baro";
			datafields[8] = "ch_accz";
			datafields[9] = "ch_ir1";
			datafields[10] = "ch_ir2";
			datafields[11] = "ch_kal0";
			datafields[12] = "ch_kal1";
			datafields[13] = "ch_kal2";

			// huch attitude
			datafields[14] = "ha_xacc";
			datafields[15] = "ha_yacc";
			datafields[16] = "ha_zacc";
			datafields[17] = "ha_zaccraw";
			datafields[18] = "ha_xaccmean";
			datafields[19] = "ha_yaccmean";
			datafields[20] = "ha_zaccmean";
			datafields[21] = "ha_xgyro";
			datafields[22] = "ha_ygyro";
			datafields[23] = "ha_zgyro";
			datafields[24] = "ha_xgyroint";
			datafields[25] = "ha_ygyroint";
			datafields[26] = "ha_zgyroint";
			datafields[27] = "ha_xmag";
			datafields[28] = "ha_ymag";
			datafields[29] = "ha_zmag";

			// fc state
			datafields[30] = "fcs_rssi";
			datafields[31] = "fcs_batt";
			datafields[32] = "fcs_pitch";
			datafields[33] = "fcs_roll";
			datafields[34] = "fcs_yaw";
			datafields[35] = "fcs_gas";

			// visual navigation
			datafields[36] = "vn_alt_velocity";
			datafields[37] = "vn_alt_absolute";
			datafields[38] = "vn_home_beta";
			datafields[39] = "vn_home_distance";
			datafields[40] = "vn_visual_compass";
			datafields[41] = "vn_ego_beta";
			datafields[42] = "vn_ego_speed";
			datafields[43] = "vn_keypoints";
			datafields[44] = "vn_error";
			datafields[45] = "vn_debug";
			break;
		case 1:	// lateral control experiments
			datafields[0] = "timestamp_ms";
			datafields[1] = "M39:C41:debug.0";
			datafields[2] = "M39:C44:debug.1";
			datafields[3] = "M39:GLOBAL_VISION_POSITION_ESTIMATE.x";
			datafields[4] = "M39:GLOBAL_VISION_POSITION_ESTIMATE.y";
			datafields[5] = "M39:GLOBAL_VISION_POSITION_ESTIMATE.z";
			datafields[6] = "M39:HUCH_ATTITUDE_CONTROL.pitch";
			datafields[7] = "M39:HUCH_ATTITUDE_CONTROL.roll";
			datafields[8] = "M39:HUCH_ATTITUDE_CONTROL.yaw";
			datafields[9] = "M39:HUCH_ATTITUDE_CONTROL.thrust";
			datafields[10] = "M39:HUCH_CTRL_HOVER_STATE.kal_s0";
			datafields[11] = "M39:HUCH_VISUAL_FLOW.u_i";
			datafields[12] = "M39:HUCH_VISUAL_FLOW.v_i";
			datafields[13] = "M39:HUCH_VISUAL_FLOW.u";
			datafields[14] = "M39:HUCH_VISUAL_FLOW.v";
			datafields[15] = "M39:ATTITUDE.roll";
			datafields[16] = "M39:ATTITUDE.pitch";
			datafields[17] = "M39:ATTITUDE.yaw";

			// initialize data to zero
			for(i = 0; i < 18; i++)
				datavals[i] = 0.;

			break;
		default:
			break;
		}
		// sort(datafields.begin(), datafields.end());
	}

	void Ctrl_Logger::logf_genfilename() {
		char outstr[200];
		time_t t;
		struct tm *tmp;
		// struct timeval tv;

		std::string datestr;
		std::string logsuffix;
		std::string logprefix;

		// get time
		t = time(NULL);
		tmp = localtime(&t);
		if (tmp == NULL) {
			perror("localtime");
			// exit(EXIT_FAILURE);
		}

		// format time
		if (strftime(outstr, sizeof(outstr), "%Y%m%d-%H%M%S", tmp) == 0) {
			fprintf(stderr, "strftime returned 0");
			//exit(EXIT_FAILURE);
		}

		// generate filename string
		datestr = std::string(outstr);
		logsuffix = std::string(".csv");
		logprefix = std::string("log/mavhub-log-");
		logfilename = logprefix + datestr + logsuffix;
		Logger::log("logfilename", logfilename, Logger::LOGLEVEL_INFO);
	}

  int Ctrl_Logger::logf_open() {
		struct stat filestat;
    if(stat("log",&filestat) < 0) {
			fprintf(stderr, "No such file or directory, exiting.\n");
			exit(-1);
		}

		fd = fopen(logfilename.c_str(), "w");
		if (fd != NULL) return 1;
		return 0;
	}

  void Ctrl_Logger::logf_close() {
		fclose(fd);
	}

	void Ctrl_Logger::logline_preamble(int start, int usec) {
		char outstr[200];
		sprintf(outstr, "%d\t", usec);
		fwrite(outstr, strlen(outstr), 1, fd);

		//field_index_start = start;
		for(int i = 1; i < start; i++) {
			sprintf(outstr, " \t");
			fwrite(outstr, strlen(outstr), 1, fd);
		}
	}

	void Ctrl_Logger::logline_coda(int start) {
		char outstr[200];
		int field_index_end = start+1;
		for(uint32_t i = field_index_end; i < datafields.size(); i++) {
			sprintf(outstr, " \t");
			fwrite(outstr, strlen(outstr), 1, fd);
		}
		sprintf(outstr, "\n");
		fwrite(outstr, strlen(outstr), 1, fd);
	}

}

#endif // HAVE_MAVLINK_H

