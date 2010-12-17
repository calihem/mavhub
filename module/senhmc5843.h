#ifndef _SENHMC5843_H_
#define _SENHMC5843_H_

#include <inttypes.h>

#include <mavlink.h>

#include "logger.h"
#include "i2csensor.h"

#define	HMC5843_ADR	0x1E

#define CRA	0x00
#define CRB	0x01
#define MR	0x02
#define DATA	0x03
#define SR	0x09

/* data rates */
#define DR05HZ	0
#define DR1HZ	1
#define DR2HZ	2
#define DR5HZ	3
#define DR10HZ	4
#define DR20HZ	5
#define DR50HZ	6
#define DR100HZ	7
#define DRDEFAULT DR10HZ

/* sensor input range */
#define GN07A	0
#define GN10A	1
#define GN15A	2
#define GN20A	3
#define GN32A	4
#define GN38A	5
#define GN45A	6
#define GN65A	7
#define GNDEFAULT GN10A

/* mode */
#define CONTINUOUS_CONVERSION_MODE	0
#define SINGLE_CONVERSION_MODE	1
#define IDLE_MODE	2
#define SLEEP_MODE	3


namespace mavhub {

	class SenHmc5843 : public I2cSensor {
		public:
			SenHmc5843(unsigned short _dev_id, unsigned short _func_id, std::string _port, int _update_rate, int _debug, int _timings, int _gain, int _mode) throw(const char *);
			virtual ~SenHmc5843();
			void print_debug();

		protected:
			virtual void run();
			virtual void* get_data_pointer(unsigned int id) throw(const char *);
			
		private:
			mavlink_huch_magnetic_kompass_t kompass_data;

			int gain;
			int mode;

			/* frequenz */
			static const int waitFreq[8];

			/* gain factoy */
			static const int gainFactor[8];
	};

} // namespace mavhub

#endif
