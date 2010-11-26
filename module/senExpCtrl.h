#ifndef _SENEXPCTRL_H_
#define _SENEXPCTRL_H_

#include <inttypes.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <mavlink.h>

#include "logger.h"
#include "i2csensor.h"


#define	EXPCTRL_ADR	0x50

namespace mavhub {

	class SenExpCtrl : public I2cSensor {
	public:
		SenExpCtrl(int _fd, int _output);
		virtual ~SenExpCtrl();
		void print_debug();

	protected:
		virtual void run();
		virtual void publish_data(uint64_t time);

	private:
		// void i2c_set_adr(const int fd, const int adr); // throw error

		mavlink_huch_exp_ctrl_t exp_ctrl_data;
		mavlink_huch_exp_ctrl_rx_t exp_ctrl_rx_data;
		mavlink_huch_ranger_t huch_ranger;

		int fd;
		int output;
		bool running;

	};
	// ----------------------------------------------------------------------------
	// I2cSensors
	// ----------------------------------------------------------------------------	

} // namespace mavhub

#endif
