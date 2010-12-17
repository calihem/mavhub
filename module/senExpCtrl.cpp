#include "senExpCtrl.h"

#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //gettime
#include <sstream> //stringstream

#include "logger.h" //"printf"
#include "utility.h"
#include "datacenter.h" //i2c-mutex, data

using namespace std;

namespace mavhub {

  // const int SenExpCtrl::wait_oversampling[4] = {4500, 7500, 13500, 25500};
  
  SenExpCtrl::SenExpCtrl(int _fd, int _output) :
		fd(_fd), output(_output)  {

		Logger::log("senExpCtrl init", Logger::LOGLEVEL_INFO);
		/* 
			 running = true;
			 pthread_mutex_lock( &i2c_mutex );
			 i2c_set_adr(fd, EXPCTRL_ADR);
			 pthread_mutex_unlock( &i2c_mutex );
			 running = false;
			 Logger::debug("ExpCtrl: init done");
		*/
  }

  SenExpCtrl::~SenExpCtrl() {
		running = false;
		join( thread );
  }

  void SenExpCtrl::run() {
		char buf[256];
		vector<uint16_t> exprx_value(4);
		//mavlink_message_t msg;
		Logger::log("ExpCtrl: running", Logger::LOGLEVEL_INFO);
		// uint64_t end = getTimeUs() + 1000000;

		running = true;
	
		while(running) {

			pthread_mutex_lock( &i2c_mutex );
			// set device address
			i2c_set_adr(fd, EXPCTRL_ADR);

			// set read register
			buf[0] = 0x00;
			write(fd,buf,1);

			// read data
			if (read(fd,buf,9) != 9) {
				//FIXME
// 				printf("I2C Send %x Failed\n",EXPCTRL_ADR);
			}
			pthread_mutex_unlock(&i2c_mutex);

			// get version
			exp_ctrl_rx_data.version = *buf;
			
			// get values
			memcpy(&exprx_value[0], buf+1, 8);

			// XXX: smart++
			exp_ctrl_rx_data.value0 = exprx_value[0];
			exp_ctrl_rx_data.value1 = exprx_value[1];
			exp_ctrl_rx_data.value2 = exprx_value[2];
			exp_ctrl_rx_data.value3 = exprx_value[3];

			// XXX: kopter specific mapping
			// XXX: 0 is USS
			huch_ranger.ranger2 = exprx_value[2];
			huch_ranger.ranger3 = exprx_value[0];
			
			// Logger::log("ExpCtrl:", (int)exp_ctrl_rx_data.version, exprx_value, Logger::LOGLEVEL_INFO);
			//Logger::log("ExpCtrl rx_t:", (int)exp_ctrl_rx_data.version, exp_ctrl_rx_data.value0, Logger::LOGLEVEL_INFO);

			/* pass data */
			publish_data(get_time_us());

			// pass more data
			// XXX: system_id
			// mavlink_msg_huch_exp_ctrl_rx_encode(39, static_cast<uint8_t>(component_id), &msg, &exp_ctrl_rx_data);

			if (output & DEBUG) print_debug();
		
		}
		Logger::debug("ExpCtrl: stopped");
  }

  void SenExpCtrl::print_debug() {
		ostringstream send_stream;
		send_stream << "ExpCtrl version: " << (int)exp_ctrl_rx_data.version << ", v0:" << exp_ctrl_rx_data.value0 << ", v1:" << exp_ctrl_rx_data.value1 << ", v2:" << exp_ctrl_rx_data.value2 << ", v3:" << exp_ctrl_rx_data.value3;
		Logger::debug(send_stream.str());
  }

	void SenExpCtrl::publish_data(uint64_t time) {
		DataCenter::set_exp_ctrl(exp_ctrl_rx_data);
		// XXX: hardware specific mapping
		DataCenter::set_huch_ranger_at(huch_ranger, 1);
		DataCenter::set_huch_ranger_at(huch_ranger, 2);
	}

} // namespace mavhub
