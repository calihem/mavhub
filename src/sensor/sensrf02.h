#ifndef _SENSRF02_H_
#define _SENSRF02_H_

#include <inttypes.h>

#include <mavlink.h>

#include "i2csensor.h"

#define	SRF02_ADR	0x70
#define SRF02_NUMCHAN 1

#define SRF02_CMD_VERSION 0x00
#define SRF02_CMD_RNG 0x02 // not valid for srf02 anyway
#define SRF02_CMD_TIMING 0xFF //
#define SRF02_CMD_SETGAIN 0x01 // not valid for srf02 anyway
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
			virtual void run();
			virtual void* get_data_pointer(unsigned int id) throw(const char *);
			virtual int get_hw_version();
			virtual void set_up();
			virtual void start_ranging();
			virtual int get_range();
			void publish_data(uint64_t time);
			
		private:
			//mavlink_huch_magnetic_kompass_t kompass_data;
			mavlink_huch_distance_t sensor_data[SRF02_NUMCHAN];
			std::vector<int> chanmap;

			int gain;
			int mode;

			/* frequenz */
			static const int waitFreq[8];

			/* gain factoy */
			//static const int gainFactor[8];
	};

} // namespace mavhub

#endif
