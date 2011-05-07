// play back logfile and simulate kopter inputs
#include "ctrl_logfileplayer.h"

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
  Ctrl_LogfilePlayer::Ctrl_LogfilePlayer(const map<string, string> args) : 
		AppLayer("ctrl_logfileplayer")
	{
		char header[1024];
		read_conf(args);
		logfile_open(logfilename);
		sd.getline(header, 1024);

		Logger::log("Ctrl_LogfilePlayer::init with header", header, Logger::LOGLEVEL_INFO);
  }

  Ctrl_LogfilePlayer::~Ctrl_LogfilePlayer() {
	}

	void Ctrl_LogfilePlayer::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
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
		// static vector<int> v(16);
		//static int8_t param_id[15];
		//Logger::log("Ctrl_LogfilePlayer got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_INFO);
		/*
		 * - load logfile
		 * - restart
		 */
	}	

  void Ctrl_LogfilePlayer::run() {
		// char* logline;
		// char logline[1024];
		//int delay;
		//istringstream is;
		std::string token;
		std::string logline;
		int tabcount;
		while (true) {
			if(sd.is_open()) {
				if(!sd.eof()) {
					//sd >> logline;
					getline(sd, logline);
					Logger::log("Ctrl_LogfilePlayer: logfile_read", logline, Logger::LOGLEVEL_INFO);

					typedef string::const_iterator ci;
					//iter = logline.begin();
					//while(iter != logline.end()) {
					tabcount = 0;
					for(ci p = logline.begin(); p != logline.end(); ++p) {
						if(*p == '\t')
							tabcount++;
						printf("x %c, %d\n", *p, tabcount);
					 	//Logger::log("Ctrl_LogfilePlayer: logfile_read", Logger::LOGLEVEL_INFO);
						//iter++;
					}
					// }
					

					// std::istringstream is(std::string(logline));
					// is >> token;
					//Logger::log("Ctrl_LogfilePlayer: logfile_read token", token, Logger::LOGLEVEL_INFO);

					// FIXME: determine type of packet depending on start position,
					//        first nonempty after tab
				}
				else
					Logger::log("Ctrl_LogfilePlayer: logfile_read EOF", Logger::LOGLEVEL_INFO);
			}
			else
				Logger::log("Ctrl_LogfilePlayer: logfile_read not open", Logger::LOGLEVEL_INFO);

			usleep(10000); // 100 Hz
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
}
