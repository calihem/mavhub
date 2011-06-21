// log data prior to controller as csv/tsv text file
//  - FIXME: logfile type settable (atm it is QGC format only),
//    options are raw mavlink (but that incompatible with the
//    use of datacenter), and "one line per packet, with timestamp
//    and type header"
//  - FIXME: generic mechanism to add messages
//  - FIXME: add raw debugout

#include "ctrl_logger.h"

#include <mavlink.h>
#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //us

#include <sstream>

#include "core/logger.h"
#include "utility.h"
#include "core/protocolstack.h"
#include "protocol/mkpackage.h"
#include "core/datacenter.h"

using namespace std;

namespace mavhub {
  Ctrl_Logger::Ctrl_Logger(const map<string, string> args) : 
		AppLayer("ctrl_logger")
	{
		char outstr[200];
		time_t t;
		struct tm *tmp;
		struct timeval tv;

		read_conf(args);
		Logger::log("Ctrl_Logger: created", Logger::LOGLEVEL_INFO);

		t = time(NULL);
		tmp = localtime(&t);
		if (tmp == NULL) {
			perror("localtime");
			// exit(EXIT_FAILURE);
		}

		if (strftime(outstr, sizeof(outstr), "%Y%m%d-%H%M%S", tmp) == 0) {
			fprintf(stderr, "strftime returned 0");
			//exit(EXIT_FAILURE);
		}

		datestr = std::string(outstr);
		logsuffix = std::string(".csv");
		logprefix = std::string("log/mavhub-log-");
		//logfilename.concatenate(outstr);
		//logfilename.concatenate(".csv");
		logfilename = logprefix + datestr + logsuffix;
		Logger::log("logfilename", logfilename, Logger::LOGLEVEL_INFO);
		logf_open();

		// write header
		// FIXME: from config
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
		datafields[32] = "fcs_nick";
		datafields[33] = "fcs_roll";
		datafields[34] = "fcs_yaw";
		datafields[35] = "fcs_gas";

		// visual navigation
		datafields[36] = "vn_alt_velocity";
		datafields[37] = "vn_psi_warping";
		datafields[38] = "vn_psi_vc";
		datafields[39] = "vn_beta";
		datafields[40] = "vn_distance";
		datafields[41] = "vn_home";
		datafields[42] = "vn_outlier";

		// sort(datafields.begin(), datafields.end());
		// unix timestamp
		
		//std::string logfheader("");
		//Logger::log("logfheader", logfheader, Logger::LOGLEVEL_INFO);
		//fwrite(logfheader.c_str(), logfheader.size(), 1, fd);
		typedef map<int, string>::const_iterator ci;
		for(ci p = datafields.begin(); p!=datafields.end(); ++p) {
			Logger::log("ctrl_logger datafield test", p->first, p->second, Logger::LOGLEVEL_INFO);
			sprintf(outstr, p->second.c_str());
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

		Logger::log("ctrl_logger [datafields.size(), starttime]", datafields.size(), starttime, Logger::LOGLEVEL_INFO);
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
		// FIXME: text or raw / binary
		Logger::log("ctrl_logger::read_conf: component_id", component_id, Logger::LOGLEVEL_INFO);
	}

  void Ctrl_Logger::handle_input(const mavlink_message_t &msg) {
		char outstr[200];
		// static int8_t param_id[15];
		// int field_index_start;
		// int field_index_end;
		// time_t t;
		// struct tm *tmp;
		struct timeval tv;
		int usec; // in milliseconds

		// Logger::log("Ctrl_Logger got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_INFO);

		gettimeofday(&tv, NULL);
		usec = ((tv.tv_sec - starttime) * 1000000 + tv.tv_usec) / 1000;

		switch(msg.msgid) {
			////////////////////// HUCH_HC_RAW
		case MAVLINK_MSG_ID_HUCH_HC_RAW:
			//Logger::log("Ctrl_Logger got huch_hc_raw msg [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_INFO);

			logline_preamble(1, usec);

			sprintf(outstr, "%d\t%d\t%d\t%d\t%d\t",
							mavlink_msg_huch_hc_raw_get_raw0(&msg),
							mavlink_msg_huch_hc_raw_get_raw1(&msg),
							mavlink_msg_huch_hc_raw_get_raw2(&msg),
							mavlink_msg_huch_hc_raw_get_raw3(&msg),
							mavlink_msg_huch_hc_raw_get_raw4(&msg)
							);
			fwrite(outstr, strlen(outstr), 1, fd);

			logline_coda(5);

			break;

		case MAVLINK_MSG_ID_HUCH_CTRL_HOVER_STATE:
			//Logger::log("Ctrl_Logger got huch_ctrl_hover_state msg [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_INFO);

			logline_preamble(6, usec);

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

			break;
		case MAVLINK_MSG_ID_HUCH_ATTITUDE:
			//Logger::log("Ctrl_Logger got huch_attitude msg [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_INFO);

			logline_preamble(14, usec);

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

			break;
		case MAVLINK_MSG_ID_MK_FC_STATUS:
			//Logger::log("Ctrl_Logger got huch_attitude msg [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_INFO);

			logline_preamble(30, usec);

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

			break;
		case MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION:
			// mavlink_huch_visual_navigation_t
			// FIXME: implement
			logline_preamble(36, usec);

			sprintf(outstr, "%f\t%f\t%f\t%f\t%d\t%d\t%d\t",
							mavlink_msg_huch_visual_navigation_get_alt_velocity(&msg),
							mavlink_msg_huch_visual_navigation_get_psi_warping(&msg),
							mavlink_msg_huch_visual_navigation_get_psi_vc(&msg),
							mavlink_msg_huch_visual_navigation_get_beta(&msg),
							mavlink_msg_huch_visual_navigation_get_distance(&msg),
							mavlink_msg_huch_visual_navigation_get_home(&msg),
							mavlink_msg_huch_visual_navigation_get_outlier(&msg)
							);
			fwrite(outstr, strlen(outstr), 1, fd);

			logline_coda(42);

			break;
		default:
			break;
		}
	}

  void Ctrl_Logger::run() {
		
		while (true) {
			usleep(1000000); // 1 Hz
			fflush(fd);
		}
	}

  int Ctrl_Logger::logf_open() {
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
