#include "senhmc5843.h"

#include <math.h> //pow
#include <iostream> //cout
#include <sstream> //stringstream

#include "main.h" //system_id
#include "logger.h" //"printf"
#include "utility.h"
#include "datacenter.h" //i2c-mutex, data

using namespace std;

namespace mavhub {

const int SenHmc5843::waitFreq[] = {2000000, 1000000, 500000, 200000, 100000, 50000, 20000, 10000};
const int SenHmc5843::gainFactor[] =  {1620,1300,970,780,530,460,390,280};

SenHmc5843::SenHmc5843(int _fd, int _update_rate, int _gain, int _mode, int _output):
	fd(_fd), output(_output)  {

	uint8_t buffer[2];

	Logger::debug("hmc5843: init...");

	running = true;
	pthread_mutex_lock( &i2c_mutex );
	i2c_set_adr(fd,HMC5843_ADR);

	/* mode */
	buffer[0] = MR;
	mode = _mode < 4? _mode: 1; // default: single-conversion mode
	buffer[1] = mode;

	if (write(fd, buffer, 2) != 2) {
	    Logger::warn("hmc5843_init(): Failed to write to slave.");
	}

	/* setup gain */
	buffer[0] = CRB;
	gain = _gain < 8? _gain: 1; // default value
	buffer[1] = gain;

	if (write(fd, buffer, 2) != 2) {
		Logger::warn("hmc5843_init(): Failed to write to slave.");
	}

	/* setup data rate */
	buffer[0] = CRA;
		/* limit to valid data rates */
	update_rate = _update_rate < 8? _update_rate: DRDEFAULT;
		/* in single conversion mode data rate should be 0,3,4,7 - why ever */
	buffer[1] = (mode == SINGLE_CONVERSION_MODE)? DR100HZ: update_rate;

	if (write(fd, buffer, 2) != 2) {
		Logger::warn("hmc5843_init(): Failed to write to slave.");
	}
	output = _output;

	pthread_mutex_unlock( &i2c_mutex );
	running = false;
	Logger::debug("done\n");
}

SenHmc5843::~SenHmc5843() {
	running = false;
	join( thread );
}

void SenHmc5843::run() {
	uint8_t buffer[6];
	int wait_time = waitFreq[update_rate];

	uint64_t frequency = wait_time;
	uint64_t start = get_time_us();
	uint64_t time_output = start + 1000000;

	Logger::debug("hmc5843: running");

	pthread_mutex_lock( &i2c_mutex );
	i2c_set_adr(fd, HMC5843_ADR);
	running = true;

	while(running) {
		/* mode - start measurement */
		if (mode == SINGLE_CONVERSION_MODE) {
			buffer[0] = MR;
			buffer[1] = mode;

			if (write(fd, buffer, 2) != 2) {
				Logger::warn("SenHmc5843::run(): Failed to write to slave.");
			}
			/* device auto-increments its reg-pointer to data-adress */
		} else {
			/* CONTINUOUS_CONVERSION_MODE - set reg-pointer to data-adress */
			buffer[0] = DATA;
			if (write(fd, buffer, 2) != 2) {
				Logger::warn("SenHmc5843::run(): Failed to write to slave.");
			}				
		}
		pthread_mutex_unlock( &i2c_mutex);

		/* wait */
		usleep(wait_time);

		/* get data */
		pthread_mutex_lock( &i2c_mutex );
		i2c_set_adr(fd, HMC5843_ADR);
		if (read(fd, buffer, 6) != 6) {
			Logger::warn("SenHmc5843::run(): Failed to read from slave.");
		} else {
			/* assign buffer to calibration data */
			hmc5843_data.data_x = (int16_t)((buffer[0] << 8) + buffer[1]);
			hmc5843_data.data_y = (int16_t)((buffer[2] << 8) + buffer[3]);
			hmc5843_data.data_z = (int16_t)((buffer[4] << 8) + buffer[5]);
			hmc5843_data.data_x /= gainFactor[gain];
			hmc5843_data.data_y /= gainFactor[gain];
			hmc5843_data.data_z /= gainFactor[gain];

			/* pass data */
			publish_data(start);

			/* debug data */
			if (output & DEBUG) print_debug();
		}

		/* calculate frequency and wait time */
		uint64_t end = get_time_us();
		frequency = (15 * frequency + end - start) / 16;
		wait_time = wait_time + waitFreq[update_rate] - frequency;
		wait_time = (wait_time < 0)? 0: wait_time;
		start = end;
		//Logger::log("hmc5843 frequency: ", frequency, Logger::LOGLEVEL_DEBUG);
		//Logger::log("hmc5843 wait_time: ", wait_time, Logger::LOGLEVEL_DEBUG);
		
		/* timings/benchmark output */
		if (output & TIMINGS) {
			if (time_output <= end) {
				Logger::log("hmc5843 frequency: ", (float)1000000/frequency, Logger::LOGLEVEL_DEBUG);
				//Logger::log("hmc5843 wait_time: ", wait_time, Logger::LOGLEVEL_DEBUG);
				time_output += 1000000;
			}
		}
	}
	/* unlock i2c */
	pthread_mutex_unlock( &i2c_mutex);

	Logger::debug("hmc5843: stopped");
}

void SenHmc5843::print_debug() {
	ostringstream send_stream;
	send_stream << "hmc5843;" << hmc5843_data.data_x << ";" << hmc5843_data.data_y << ";" << hmc5843_data.data_z;
	Logger::debug(send_stream.str());
}

void SenHmc5843::publish_data(uint64_t time) {
	hmc5843_data.timestamp = time;
	DataCenter::set_hmc5843(hmc5843_data);
}

} // namespace mavh
