#ifndef _SENSRF02_H_
#define _SENSRF02_H_

#include <inttypes.h>

#include <mavlink.h>

#include "i2csensor.h"

#define	SRF02_ADR	0x70
#define SRF02_NUMCHAN 2

#define SRF02_REG_RD_VERSION 0x00
#define SRF02_REG_RD_LIGHT 0x01 // not valid for srf02 anyway
#define SRF02_REG_RD_ECHO_0 0x02 // not valid for srf02 anyway
#define SRF02_REG_WR_CMD 0x00
#define SRF02_REG_WR_GAIN 0x01
#define SRF02_REG_WR_RNG 0x02
#define SRF02_CMD_TIMING 0xFF //
#define SRF02_CMD_SETGAIN_VAL 0x10 // not valid for srf02 anyway
#define SRF02_CMD_STARTRNG 0x00 // not valid for srf02 anyway
#define SRF02_CMD_STARTRNG_SPEC 0x52 // not valid for srf02 anyway
#define SRF02_CMD_GETRNG 0x02 // not valid for srf02 anyway

/* #define CRA	0x00 */
/* #define CRB	0x01 */
/* #define MR	0x02 */
/* #define DATA	0x03 */
/* #define SR	0x09 */

/* data rates */
/* #define DR05HZ	0 */
/* #define DR1HZ	1 */
/* #define DR2HZ	2 */
/* #define DR5HZ	3 */
#define DR10HZ	4
#define DR20HZ	5
/* #define DR50HZ	6 */
/* #define DR100HZ	7 */
#define DRDEFAULT DR10HZ

/* sensor input range */
/* #define GN07A	0 */
/* #define GN10A	1 */
/* #define GN15A	2 */
/* #define GN20A	3 */
/* #define GN32A	4 */
/* #define GN38A	5 */
/* #define GN45A	6 */
/* #define GN65A	7 */
/* #define GNDEFAULT GN10A */

/* mode */
/* #define CONTINUOUS_CONVERSION_MODE	0 */
/* #define SINGLE_CONVERSION_MODE	1 */
/* #define IDLE_MODE	2 */
/* #define SLEEP_MODE	3 */

#include <list>
#include <vector>

namespace mavhub {

	class SenSrf02 : public I2cSensor {
		public:
		//SenSrf02(unsigned short _dev_id, unsigned short _func_id, std::string _port, int _update_rate, int _debug, int _timings, int _gain, int _mode) throw(const char *);
		SenSrf02(unsigned short _dev_id, unsigned short _func_id, std::string _port, int _update_rate, int _debug, int _timings, std::list< std::pair<int, int> > _chanmap_pairs) throw(const char *);
			virtual ~SenSrf02();
			void print_debug();
			int get_last_measurement();

		protected:
			/// main method
			virtual void run();
			/// well ...
			virtual void* get_data_pointer(unsigned int id) throw(const char *);
			/// return the firmware version from the PIC
			virtual int get_hw_version();
			/// set up the sensor during init: for the SRF02 nothing should happen, for the SRF08 and 10 this should set the gain and range registers
			virtual void set_up();
			/// initialize ranging
			virtual void start_ranging();
			/// read ranging result after it has finished
			virtual int get_range();
			/// copy data into datacenter
			void publish_data(uint64_t time);
			
		private:
			/// array of distance sensor structures
			mavlink_huch_distance_t sensor_data[SRF02_NUMCHAN];
			/// channel mapping: sensor channel to mavhub global logical channels
			std::vector<int> chanmap;

			/* int gain; */
			/* int mode; */

			/* frequenz */
			static const int waitFreq[8];

			/* gain factoy */
			//static const int gainFactor[8];
	};

} // namespace mavhub

#endif
