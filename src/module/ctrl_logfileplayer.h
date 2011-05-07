// play back log data and simulate kopter inputs

#ifndef _CTRL_LOGFILEPLAYER_H_
#define _CTRL_LOGFILEPLAYER_H_

#include "debug_channels.h"
#include "core/logger.h"
#include "core/protocollayer.h"

#include <fstream>

namespace mavhub {
	class Ctrl_LogfilePlayer : public AppLayer {
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
		/// read data from config
		virtual void read_conf(const std::map<std::string, std::string> args);
		/// open logfile
		virtual int logfile_open(const std::string logfile);
		/// close logfile
		virtual void logfile_close();
	};
}

#endif
