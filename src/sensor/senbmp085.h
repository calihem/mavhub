#ifndef _SENBMP085_H_
#define _SENBMP085_H_

#include <inttypes.h>

#include <mavlink.h>

#include "i2csensor.h"


#define	BMP085_ADR	0x77

namespace mavhub {

	typedef struct {
		int16_t ac1;
		int16_t ac2;
		int16_t ac3;
		uint16_t ac4;
		uint16_t ac5;
		uint16_t ac6;
		int16_t b1;
		int16_t b2;
		int16_t mb;
		int16_t mc;
		int16_t md;
	} calibration_data_t;


	class SenBmp085 : public I2cSensor {
		public:
			SenBmp085(unsigned short _dev_id, unsigned short _func_id, unsigned short _func_id1, unsigned short _func_id2, std::string _port, int _update_rate, int _debug, int _timings, int _update_rate_temp) throw(const char *);
			virtual ~SenBmp085();
			void print_debug();
						
		protected:
			virtual void run();
			virtual void* get_data_pointer(unsigned int id) throw(const char *);

		private:
			static void read_calibration_data(const int fd, calibration_data_t &cal_data) throw(const char *);
			static void request_temp_data(const int fd); // throw error
			static uint64_t request_pres_data(const int fd, const int oversampling); // throw error
			static int get_temp_data(const int fd); // throw error
			static int get_pres_data(const int fd, const int oversampling); // throw error

			static int calc_temp(const int uncomp_temp, int &b5, const calibration_data_t &cal_data);
			static int calc_pres(const int b5, const int oversampling, const int uncomp_pres, const calibration_data_t &cal_data);
			static float calc_altitude(const int p, const int p0);

			calibration_data_t calibration_data;
			mavlink_huch_raw_pressure_t raw_pressure;
			mavlink_huch_temperature_t temperature;
			mavlink_huch_altitude_t altitude;

			unsigned short func_id1;
			unsigned short func_id2;
			int oversampling;
			int pressure_0;
			int update_rate_temp;

			static const int wait_oversampling[4];
			#define WAITTEMP 4500
	};
	// ----------------------------------------------------------------------------
	// I2cSensors
	// ----------------------------------------------------------------------------	


} // namespace mavhub

#endif
