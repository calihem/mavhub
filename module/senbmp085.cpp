#include "senbmp085.h"

#include <math.h> //pow
#include <iostream> //cout
#include <sstream> //stringstream

#include "logger.h" //"printf"
#include "utility.h"
#include "datacenter.h" //i2c-mutex, data

using namespace std;

namespace mavhub {

const int SenBmp085::wait_oversampling[4] = {4500, 7500, 13500, 25500};

SenBmp085::SenBmp085(int _fd, int _update_temp, int _oversampling, int _output) throw(exception):
	fd(_fd), update_temp(_update_temp), output(_output) {

	Logger::debug("bmp085: init...");
	running = true;
	pthread_mutex_lock( &i2c_mutex );
	i2c_set_adr(fd, BMP085_ADR);

	/* read calibration data once */
	read_calibration_data(fd, calibration_data);

	/* compute pressure at current altitude */
	/* request temperature data */
	request_temp_data(fd);

	/* wait */
	usleep(WAITTEMP);		

	/* get temperature data */
	int uncomp_temp = get_temp_data(fd);		
	/* compute temperature data */
	int b5;
	bmp085_data.temperature = calc_temp(uncomp_temp, b5, calibration_data);
	/* request pressure data  with maximum oversampling */
	oversampling = 3;
	request_pres_data(fd, oversampling);

	/* wait */
	usleep(wait_oversampling[oversampling]);

	/* get pressure data */
	int uncomp_pres = get_pres_data(fd, oversampling);

	/* compute pressure data */
	bmp085_data.pressure = calc_pres(b5, oversampling, uncomp_pres, calibration_data);
	pressure_0 = bmp085_data.pressure;
	oversampling = _oversampling < 4? _oversampling: 1;

	pthread_mutex_unlock( &i2c_mutex );
	running = false;
	Logger::debug("done");
}

SenBmp085::~SenBmp085() {
	running = false;
	join( thread );
}

void SenBmp085::run() {
	Logger::debug("bmp085: running");
	int count = 0;
	int countTemp = 0;
	uint64_t end = get_time_us() + 1000000;
	int uncomp_temp = 0;

	running = true;
	
	while(running) {
		int b5;

		pthread_mutex_lock( &i2c_mutex );
		i2c_set_adr(fd, BMP085_ADR);
		if (!countTemp--) {
			/* request temperature data */
			request_temp_data(fd);
			pthread_mutex_unlock( &i2c_mutex );

			/* wait */
			usleep(WAITTEMP);		

			/* get temperature data */
			pthread_mutex_lock( &i2c_mutex );
			i2c_set_adr(fd, BMP085_ADR);
			uncomp_temp = get_temp_data(fd);					
			countTemp = update_temp;

			/* compute temperature data */
			bmp085_data.temperature = calc_temp(uncomp_temp, b5, calibration_data);
		}

		/* request pressure data */
		request_pres_data(fd, oversampling);
		pthread_mutex_unlock( &i2c_mutex );
		bmp085_data.timestamp = get_time_us();

		/* wait */
		usleep(wait_oversampling[oversampling]);	

		/* get pressure data */
		pthread_mutex_lock( &i2c_mutex );
		i2c_set_adr(fd, BMP085_ADR);
		int uncomp_pres = get_pres_data(fd, oversampling);
		pthread_mutex_unlock( &i2c_mutex );

		/* compute pressure data */
		bmp085_data.pressure = calc_pres(b5, oversampling, uncomp_pres, calibration_data);

		/* compute altitude */
		bmp085_data.height = calc_altitude(bmp085_data.pressure, pressure_0);	

		/* pass data */
		publish_data(get_time_us());

		if (output & DEBUG) print_debug();
		
		/* timings/benchmark output */
		if (output & TIMINGS) {
			if (end <= get_time_us()) {
				Logger::log("bmp frequency: ", count, Logger::LOGLEVEL_DEBUG);
				end += 1000000;
				count = 0;
			} else count++;
		}
	}
	Logger::debug("bmp085: stopped");
}

void SenBmp085::read_calibration_data(const int fd, calibration_data_t &cal_data) throw(exception) {
	#define CALIBRATION_DATA_SIZE sizeof(calibration_data_t)
	uint8_t buffer[CALIBRATION_DATA_SIZE];
	#define EEPROM_ADR	0xAA
	buffer[0] = EEPROM_ADR;
	
	if (write(fd, buffer, 1) != 1) {
		Logger::warn("read_calibration_data(): Failed to write to slave!");
		throw exception();
	}
	if (read(fd, buffer, CALIBRATION_DATA_SIZE) != CALIBRATION_DATA_SIZE) {
		Logger::warn("read_calibration_data(): Failed to read from slave!");
		throw exception();
	} else {
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
}

void SenBmp085::request_temp_data(const int fd) {
	uint8_t buffer[2];
	buffer[0] = 0xF4;	/* cmd reg adr */
	buffer[1] = 0x2E; 	/* cmd for uncompensated temperature */
	if (write(fd, buffer, 2) != 2) {
		Logger::warn("request_temp_data(): Failed to write to slave!");
	}
}

int SenBmp085::get_temp_data(const int fd) {
	int result = 0;
	uint8_t buffer[2];
	buffer[0] = 0xF6; /* reg adr for result */
	if (write(fd, buffer, 1) != 1) {
		Logger::warn("get_temp_data(): Failed to write to slave!");
	}
	if (read(fd, buffer, 2) != 2) {
		Logger::warn("get_temp_data(): Failed to read from slave!");
	} else {
		/* return uncompensated temperature value */
		result = ((buffer[0] & 0x00FF) << 8) + buffer[1];
	}

	return result;
}

uint64_t SenBmp085::request_pres_data(const int fd, const int oversampling) {
	uint8_t buffer[2];

	buffer[0] = 0xF4;	/* cmd reg adr */
	buffer[1] = 0x34 + (oversampling << 6); /* cmd for uncompensated pressure */
	if (write(fd, buffer, 2) != 2) {
		Logger::warn("request_pres_data(): Failed to write to slave!");
	}
	return get_time_us();
}

int SenBmp085::get_pres_data(const int fd, const int oversampling) {
	int result = 0;
	uint8_t buffer[3];

	buffer[0] = 0xF6; /* reg adr for result */
	if (write(fd, buffer, 1) != 1) {
		Logger::warn("get_pres_data(): Failed to write to slave!");
	}
	if (read(fd, buffer, 3) != 3) {
		Logger::warn("get_pres_data(): Failed to read from slave!");
	} else {
		/* return uncompensated pressure value */
		result = ((buffer[0] << 16) + (buffer[1] << 8) + buffer[2]) >> (8 - oversampling);
	}

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
	ostringstream send_stream;
	send_stream << "bmp085;" << bmp085_data.temperature << ";" << bmp085_data.pressure << ";" << bmp085_data.height;
	Logger::debug(send_stream.str());
}

void SenBmp085::publish_data(uint64_t time) {
	bmp085_data.timestamp = time;
	DataCenter::set_bmp085(bmp085_data);
}

} // namespace mavhub
