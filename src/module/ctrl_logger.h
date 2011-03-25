// play back log data and simulate kopter inputs

#ifndef _CTRL_LOGGER_H_
#define _CTRL_LOGGER_H_

#include "debug_channels.h"
#include "core/logger.h"
#include "protocol/protocollayer.h"

namespace mavhub {
	class Ctrl_Logger : public AppLayer<mavlink_message_t> {
	public:
		Ctrl_Logger(const std::map<std::string, std::string> args);
		virtual ~Ctrl_Logger();
		virtual void handle_input(const mavlink_message_t &msg);
  protected:
		/// this thread's main method
		virtual void run();
	private:
		/// component id
		uint16_t component_id;
		/// logfile handle
		FILE* fd;
		/// logfile
		std::string logfilename;
		std::string datestr;
		std::string logsuffix;
		std::string logprefix;
		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);
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

		// start time
		uint64_t starttime;
	};
}

#endif
