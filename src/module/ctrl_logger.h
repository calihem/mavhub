// play back log data and simulate kopter inputs

#ifndef _CTRL_LOGGER_H_
#define _CTRL_LOGGER_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#include "modulebase.h"
#include "debug_channels.h"
#include "core/logger.h"
#include "protocol/protocollayer.h"

namespace mavhub {
	class Ctrl_Logger : public ModuleBase {
	public:
		Ctrl_Logger(const std::map<std::string, std::string> args);
		virtual ~Ctrl_Logger();
		virtual void handle_input(const mavlink_message_t &msg);
  protected:
		/// this thread's main method
		virtual void run();
	private:
		/// logfile handle
		FILE* fd;
		/// logfile
		std::string logfilename;
		/// logging on
		bool logging;
		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);
		/// init logging
		virtual void log_init();
		/// de-init logging
		virtual void log_deinit();
		/// generate logfile header
		virtual void genheader();
		/// write data into log, mode dependent
		virtual void handle_logdata(const mavlink_message_t &msg);
		/// write data into log, mode 0
		virtual void handle_logdata_0(int ms, const mavlink_message_t &msg);
		/// write data into log, mode 1
		virtual void handle_logdata_1(int ms, const mavlink_message_t &msg);
		/// generate logfile name with timestamp
		virtual void logf_genfilename();
		/// open logfile
		virtual int logf_open();
		/// close logfile
		virtual void logf_close();
		/// logline preamble
		virtual void  logline_preamble(int start, int usec);
		/// logline coda
		virtual void  logline_coda(int start);

		/// write header
		std::map<int, std::string> datafields;
		/// keep local data array
		std::map<int, double> datavals;
		
		// start time
		uint64_t starttime;
	};
}

#endif // HAVE_MAVLINK_H

#endif

