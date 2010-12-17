#ifndef _SENEXPCTRL_H_
#define _SENEXPCTRL_H_

#include <inttypes.h>
/* #include <linux/i2c.h> */
/* #include <linux/i2c-dev.h> */
#include <sys/ioctl.h>

#include <mavlink.h>

#include "logger.h"
#include "i2csensor.h"


#define	EXPCTRL_ADR	0x50

namespace mavhub {

class SenExpCtrl : public I2cSensor {
	public:
		SenExpCtrl(unsigned short _dev_id, 
			unsigned short _func_id, 
			std::string _port, 
			int _update_rate, 
			int _debug, 
			int _timings )throw(const char *);
		virtual ~SenExpCtrl();
		void print_debug();

	protected:
		virtual void run();
		void publish_data(uint64_t time);
		virtual void* get_data_pointer(unsigned int id) throw(const char *);

	private:
		// void i2c_set_adr(const int fd, const int adr); // throw error
		mavlink_huch_distance_t sensor_data[2];
		mavlink_huch_exp_ctrl_t exp_ctrl_data;
		mavlink_huch_exp_ctrl_rx_t exp_ctrl_rx_data;
		mavlink_huch_ranger_t huch_ranger;

};
// ----------------------------------------------------------------------------
// I2cSensors
// ----------------------------------------------------------------------------	

} // namespace mavhub

#endif
