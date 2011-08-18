// play back log data and simulate kopter inputs

#ifndef _CTRL_LOGFILEPLAYER_H_
#define _CTRL_LOGFILEPLAYER_H_

#include "debug_channels.h"
#include "core/logger.h"
#include "protocol/protocollayer.h"

#include <fstream>

namespace mavhub {
	class Ctrl_LogfilePlayer : public AppLayer<mavlink_message_t> {
	public:
		Ctrl_LogfilePlayer(const std::map<std::string, std::string> args);
		virtual ~Ctrl_LogfilePlayer();
		virtual void handle_input(const mavlink_message_t &msg);
  protected:
		/// this thread's main method
		virtual void run();
	private:
		/// component id
		uint16_t component_id;
		/// logfile handle
		FILE* fd;
		/// logfile handle, c++ style
		std::ifstream sd;
		/// logfile path
		std::string logfilename;
		/* /// replay mode */
		/* int replay_mode; */
		/// replay modes
		enum replay_modes {QGC, CH}; // qgroundcontrol, hover ctrl
		/// parameter list request
		int param_request_list;
		/// parameters
		std::map<std::string, double>	params;
		/// set defaults config
		virtual void conf_defaults();
		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);
		/// open logfile
		virtual int logfile_open(const std::string logfile);
		/// close logfile
		virtual void logfile_close();
		/// copy attitude to debugout structure
		virtual void attitude2debugout(mavlink_huch_attitude_t* attitude, mavlink_mk_debugout_t* debugout);
		/// copy fc state to debugout structure
		virtual void mk_fc_state2debugout(mavlink_mk_fc_status_t* attitude, mavlink_mk_debugout_t* debugout);
		/// copy control hover raw input to debugout structure
		virtual void ch_raw2debugout(mavlink_huch_hc_raw_t* attitude, mavlink_mk_debugout_t* debugout);
		/// copy control hover raw input to datacenter
		virtual void ch_raw2datacenter(mavlink_huch_hc_raw_t* attitude);
		/// setval_u
		void debugout_setval_u(mavlink_mk_debugout_t* dbgout, int index, uint16_t val);
		/// setval_s
		void debugout_setval_s(mavlink_mk_debugout_t* dbgout, int index, int16_t val);
		/// setval_s32
		void debugout_setval_s32(mavlink_mk_debugout_t* dbgout, int indexl, int indexh, int32_t val);
	};
}

#endif
