#ifndef _SENCMP02_H_
#define _SENCMP02_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>
#endif // HAVE_MAVLINK_H

#include "i2csensor.h"

#include <inttypes.h>
/* #include <linux/i2c.h> */
/* #include <linux/i2c-dev.h> */
#include <sys/ioctl.h>
#include <list>
#include <vector>

#define	CMP02_ADR	0x60

#define CMP02_NUMCHAN 1

namespace mavhub {

class SenCmp02 : public I2cSensor {
	public:
		SenCmp02(unsigned short _dev_id, 
							 unsigned short _func_id, 
							 std::string _port, 
							 int _update_rate,
							 int _debug, 
							 int _timings,
							 std::list< std::pair<int, int> > _chanmap_pairs)
			throw(const char *);
		virtual ~SenCmp02();
		void print_debug();

	protected:
		virtual void run();
		void publish_data(uint64_t time);
		virtual void* get_data_pointer(unsigned int id) throw(const char *);

	private:
		// void i2c_set_adr(const int fd, const int adr); // throw error
		// FIXME: Cmp02 measurement doesnt need to be a "distance", just a voltage
#ifdef HAVE_MAVLINK_H
		mavlink_huch_analog_t sensor_data[CMP02_NUMCHAN];
#endif // HAVE_MAVLINK_H
		// std::vector<int> chanmap;
		//mavlink_huch_exp_ctrl_t exp_ctrl_data;
		//mavlink_huch_exp_ctrl_rx_t exp_ctrl_rx_data;
		//mavlink_huch_ranger_t huch_ranger;
		uint8_t version;
		std::vector<int> chanmap;
};
// ----------------------------------------------------------------------------
// I2cSensors
// ----------------------------------------------------------------------------	

} // namespace mavhub

#endif
