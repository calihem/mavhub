// control lateral components: nick, roll, yaw

#include "ctrl_lateral.h"

#include <mavlink.h>
#include "core/datacenter.h"

#include <sstream>

#define RAND_MAX_TO_M1 1/(double)RAND_MAX

using namespace std;

namespace mavhub {
	Ctrl_Lateral::Ctrl_Lateral(const map<string, string> args) : AppLayer("ctrl_lateral") {
		read_conf(args);
	}

	Ctrl_Lateral::~Ctrl_Lateral() {
	}

  void Ctrl_Lateral::handle_input(const mavlink_message_t &msg) {
		static int8_t param_id[15];
		Logger::log("Ctrl_Lateral got mavlink_message [len, msgid]:", (int)msg.len, (int)msg.msgid, Logger::LOGLEVEL_DEBUG);
		switch(msg.msgid) {
		default:
			break;
		}		
	}

  void Ctrl_Lateral::run() {
		uint64_t dt = 0;
		struct timeval tk, tkm1; // timevals
		// more timing
		int update_rate = 10; // 100 Hz
		int wait_freq = update_rate? 1000000 / update_rate: 0;
		int wait_time = wait_freq;
		uint64_t frequency = wait_time;
		uint64_t start = get_time_us();
		uint64_t usec;

		// body variables
		double tmp;
		int16_t nick, roll, yaw;
		vector<int16_t> v(3);

		gettimeofday(&tk, NULL);
		gettimeofday(&tkm1, NULL);

		nick = roll = yaw = 0;

		Logger::log("Ctrl_Lateral started:", name(), Logger::LOGLEVEL_INFO);
		while(true) {
			/* wait time */
			usec = get_time_us();
			uint64_t end = usec;
			wait_time = wait_freq - (end - start);
			wait_time = (wait_time < 0)? 0: wait_time;
		
			/* wait */
			usleep(wait_time);
			//usleep(10);

			/* calculate frequency */
			end = get_time_us();
			frequency = (15 * frequency + end - start) / 16;
			start = end;

			Logger::log("Ctrl_Lateral slept for", wait_time, component_id, Logger::LOGLEVEL_INFO);

			gettimeofday(&tk, NULL);
			//timediff(tdiff, tkm1, tk);
			dt = (tk.tv_sec - tkm1.tv_sec) * 1000000 + (tk.tv_usec - tkm1.tv_usec);
			tkm1 = tk; // save current time

			// body
			tmp = rand() * RAND_MAX_TO_M1;
			tmp -= 0.5;
			tmp *= 1000;
			nick = v[0] = (int16_t)tmp;
			tmp = rand() * RAND_MAX_TO_M1;
			tmp -= 0.5;
			tmp *= 1000;
			roll = v[1] = (int16_t)tmp;
			tmp = rand() * RAND_MAX_TO_M1;
			tmp *= 1000;
			yaw = v[2] = (int16_t)tmp;
			DataCenter::set_extctrl_nick(nick);
			DataCenter::set_extctrl_roll(roll);
			DataCenter::set_extctrl_yaw(yaw);
			Logger::log("Ctrl_Lateral (n,r,y)", v, Logger::LOGLEVEL_INFO);
		}
	}

	void Ctrl_Lateral::read_conf(const map<string, string> args) {
		map<string,string>::const_iterator iter;

		iter = args.find("component_id");
		if( iter != args.end() ) {
			istringstream s(iter->second);
			s >> component_id;
		}

		Logger::log("ctrl_lateral::read_conf: component_id", component_id, Logger::LOGLEVEL_DEBUG);
	}
}
