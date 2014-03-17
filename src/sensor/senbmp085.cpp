#include "senbmp085.h"

#include <math.h> //pow
#include <iostream> //cout
#include <sstream> //stringstream

#include "core/logger.h" //"printf"
#include "lib/hub/time.h"
// #include "datacenter.h"

using namespace std;
using namespace hub;

namespace mavhub {

const int SenBmp085::wait_oversampling[4] = {4500, 7500, 13500, 25500};

SenBmp085::SenBmp085(unsigned short _dev_id, unsigned short _func_id, unsigned short _func_id1, unsigned short _func_id2, std::string _port, int _update_rate, int _debug, int _timings, int _update_rate_temp) throw(const char *):
	update_rate_temp(_update_rate_temp) {
	//FIXME Initialisierung
	dev_id = _dev_id;
	func_id = _func_id;
	func_id1 = _func_id2;
	func_id2 = _func_id2;
	update_rate = _update_rate;
	debug = _debug;
	timings= _timings;

	Logger::debug("bmp085: init...");

	status = RUNNING;	
	try {
		/* check config */
		if ((get_data_pointer(func_id << 16) == NULL) || (get_data_pointer(func_id1 << 16) == NULL) || (get_data_pointer(func_id2 << 16) == NULL)) {
			throw "sensor(software code) doesn't support configured function";
		}

		fd = i2c_init(_port);
		
		i2c_start_conversion(fd, BMP085_ADR);

		/* read calibration data once */
		read_calibration_data(fd, calibration_data);

		/* request temperature data */
		request_temp_data(fd);

		/* wait */
		usleep(WAITTEMP);		

#ifdef MAVLINK_ENABLED_HUCH
		/* get temperature data */
		int uncomp_temp = get_temp_data(fd);		
		/* compute temperature data */
		int b5;
		temperature.temperature = calc_temp(uncomp_temp, b5, calibration_data);
#endif // MAVLINK_ENABLED_HUCH
		/* request pressure data with maximum oversampling */
		oversampling = 3;
		request_pres_data(fd, oversampling);

		/* wait */
		usleep(wait_oversampling[oversampling]);

#ifdef MAVLINK_ENABLED_HUCH
		/* get pressure data */
		int uncomp_pres = get_pres_data(fd, oversampling);

		/* compute pressure at current altitude */
		raw_pressure.pressure = calc_pres(b5, oversampling, uncomp_pres, calibration_data);
		pressure_0 = raw_pressure.pressure;
#endif // MAVLINK_ENABLED_HUCH

		/* calculate oversampling from update rate */
		//TODO 
		oversampling = update_rate < 4? update_rate: 1;

		i2c_end_conversion(fd);
		Logger::debug("done");
		status = STOPPED;
	}
	catch(const char *message) {
		status = UNINITIALIZED;
		i2c_end_conversion(fd);

		string s(message);
		throw ("SenBmp085(): " + s).c_str();
	}
}

SenBmp085::~SenBmp085() {}

void SenBmp085::run() {
	Logger::debug("bmp085: running");
	int count = 0;
	int countTemp = 0;
	uint64_t end = get_time_us() + 1000000;
	int uncomp_temp = 0;
	uint64_t usec;

	status = RUNNING;
	
	try {
		while(status == RUNNING) {
			int b5;

			i2c_start_conversion(fd, BMP085_ADR);
			if (!countTemp--) {
				/* request temperature data */
				request_temp_data(fd);
				i2c_end_conversion(fd);
				/* wait */
				usleep(WAITTEMP);		

				/* get temperature data */
				i2c_start_conversion(fd, BMP085_ADR);
				uncomp_temp = get_temp_data(fd);					
				countTemp = update_rate_temp;

				/* compute temperature data */
#ifdef MAVLINK_ENABLED_HUCH
				{ // begin of data mutex scope
					cpp_pthread::Lock ri_lock(data_mutex);
					temperature.temperature = calc_temp(uncomp_temp, b5, calibration_data);
					temperature.usec = get_time_us();
				} // end of data mutex scope
#endif // MAVLINK_ENABLED_HUCH
			}

			/* request pressure data */
			request_pres_data(fd, oversampling);
			i2c_end_conversion(fd);
			usec = get_time_us();

			/* wait */
			usleep(wait_oversampling[oversampling]);	

			/* get pressure data */
			i2c_start_conversion(fd, BMP085_ADR);
			int uncomp_pres = get_pres_data(fd, oversampling);
			i2c_end_conversion(fd);

#ifdef MAVNLINK_ENABLED_HUCH
			/* compute pressure data */
			int _pres = calc_pres(b5, oversampling, uncomp_pres, calibration_data);

			/* compute altitude */
			float _alt = calc_altitude(_pres, pressure_0);	
			{ // begin of data mutex scope
				cpp_pthread::Lock ri_lock(data_mutex);
				raw_pressure.pressure = _pres;
				raw_pressure.usec = usec;
				altitude.altitude = _alt;
				altitude.usec = usec;
			} // end of data mutex scope
#endif // MAVNLINK_ENABLED_HUCH

			if (debug) print_debug();
		
			/* timings/benchmark output */
			if (timings) {
				if (end <= get_time_us()) {
					Logger::log("bmp frequency: ", count, Logger::LOGLEVEL_DEBUG);
					end += 1000000;
					count = 0;
				} else count++;
			}
		}
	}
	catch(const char *message) {
		i2c_end_conversion(fd);

		string s(message);
		throw ("SenBmp085::run(): " + s).c_str();
	}
	Logger::debug("bmp085: stopped");
}

void SenBmp085::read_calibration_data(const int fd, calibration_data_t &cal_data) throw(const char *) {
	#define CALIBRATION_DATA_SIZE sizeof(calibration_data_t)
	uint8_t buffer[CALIBRATION_DATA_SIZE];
	#define EEPROM_ADR	0xAA
	buffer[0] = EEPROM_ADR;
	
	try {
		i2c_write_bytes(fd, buffer, 1);
		i2c_read_bytes(fd, buffer, CALIBRATION_DATA_SIZE);
		/* assign buffer to calibration data */
		cal_data.ac1 = ((buffer[0] << 8) + buffer[1]);
		cal_data.ac2 = ((buffer[2] << 8) + buffer[3]);
		cal_data.ac3 = ((buffer[4] << 8) + buffer[5]);
		cal_data.ac4 = ((buffer[6] << 8) + buffer[7]);
		cal_data.ac5 = ((buffer[8] << 8) + buffer[9]);
		cal_data.ac6 = ((buffer[10] << 8) + buffer[11]);
		cal_data.b1 = ((buffer[12] << 8) + buffer[13]);
		cal_data.b2 = ((buffer[14] << 8) + buffer[15]);
		cal_data.mb = ((buffer[16] << 8) + buffer[17]);
		cal_data.mc = ((buffer[18] << 8) + buffer[19]);
		cal_data.md = ((buffer[20] << 8) + buffer[21]);
	}
	catch(const char *message) {
		string s(message);
		throw ("read_calibration_data(): " + s).c_str();
	}
}

void SenBmp085::request_temp_data(const int fd) {
	uint8_t buffer[2];
	buffer[0] = 0xF4;	/* cmd reg adr */
	buffer[1] = 0x2E; 	/* cmd for uncompensated temperature */

	i2c_write_bytes(fd, buffer, 2);
}

int SenBmp085::get_temp_data(const int fd) {
	int result = 0;
	uint8_t buffer[2];
	buffer[0] = 0xF6;	/* reg adr for result */

	i2c_write_bytes(fd, buffer, 1);

	i2c_read_bytes(fd, buffer, 2);

	result = ((buffer[0] & 0x00FF) << 8) + buffer[1];

	return result;
}

uint64_t SenBmp085::request_pres_data(const int fd, const int oversampling) {
	uint8_t buffer[2];

	buffer[0] = 0xF4;	/* cmd reg adr */
	buffer[1] = 0x34 + (oversampling << 6); /* cmd for uncompensated pressure */
	i2c_write_bytes(fd, buffer, 2);

	return get_time_us();
}

int SenBmp085::get_pres_data(const int fd, const int oversampling) {
	int result = 0;
	uint8_t buffer[3];

	buffer[0] = 0xF6; /* reg adr for result */
	i2c_write_bytes(fd, buffer, 1);

	i2c_read_bytes(fd, buffer, 3);

	/* return uncompensated pressure value */
	result = ((buffer[0] << 16) + (buffer[1] << 8) + buffer[2]) >> (8 - oversampling);

	return result;
}

int SenBmp085::calc_temp(const int uncomp_temp, int &b5, const calibration_data_t &cal_data) {
	int x1 = ((uncomp_temp - cal_data.ac6) * cal_data.ac5) >> 15;
	int x2 = (cal_data.mc << 11) / (x1 + cal_data.md);
	b5 = x1 + x2;
	return (b5 + 8) >> 4;
}

int SenBmp085::calc_pres(const int b5, const int oversampling, const int uncomp_pres, const calibration_data_t &cal_data) {
	int b6 = b5 - 4000;
	int x1 = (cal_data.b2 * ((b6 * b6) >> 12)) >> 11;
	int x2 = (cal_data.ac2 * b6) >> 11;
	int x3 = x1 + x2;
	int b3 = (((cal_data.ac1 * 4 + x3) << oversampling) + 2) / 4;
	x1 = (cal_data.ac3 * b6) >> 13;
	x2 = (cal_data.b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) / 4;
	uint32_t b4 = cal_data.ac4 *(uint32_t)(x3 + 32768) >> 15;
	uint32_t b7 = ((uint32_t)uncomp_pres - b3) * (50000 >> oversampling);
	int p = (b7 < 0x80000000)? (b7 * 2) / b4 : (int32_t)((b7 / b4) * 2);
	x1 = pow(p >> 8,2);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	return p + ((x1 + x2 + 3791) >> 4);
}

float SenBmp085::calc_altitude(const int p, const int p0) {
	//return 44330 * (1 - pow((float)P/101325,0.190295));
	return 44330 * (1 - pow((float)p/p0,0.190295));
}

void SenBmp085::print_debug() {
	//FIXME: remove variable send_stream (no extra allocation for debug messages)
	ostringstream send_stream;
#ifdef MAVNLINK_ENABLED_HUCH
	send_stream << "bmp085;" << temperature.temperature << ";" << raw_pressure.pressure << ";" << altitude.altitude;
#else
	send_stream << "bmp085: mavlink missing";
#endif // MAVNLINK_ENABLED_HUCH
	Logger::debug(send_stream.str());
}

void* SenBmp085::get_data_pointer(unsigned int id) throw(const char *) {
	if (status == RUNNING) {
		switch ((0xFFFF0000 & id) >> 16) {
#ifdef MAVNLINK_ENABLED_HUCH
			case ALTITUDE_SENSOR: // altitude 
				return &altitude;
			case TEMPERATURE_SENSOR: // temperature
				return &temperature;
			case PRESSURE_SENSOR: // raw pressure
				return &raw_pressure;
#endif // MAVNLINK_ENABLED_HUCH
			default: throw "sensor bmp085 doesn't support this sensor type";
		}
	} throw "sensor bmp085 isn't running";
}

} // namespace mavhub
