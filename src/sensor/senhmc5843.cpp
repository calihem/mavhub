#include "senhmc5843.h"

#include <math.h> //pow
#include <iostream> //cout

#include "core/logger.h" //"printf"
#include "utility.h"
#include "lib/hub/time.h"
#include "core/datacenter.h"

using namespace std;
using namespace hub;

namespace mavhub {

	const int SenHmc5843::waitFreq[] = {2000000, 1000000, 500000, 200000, 100000, 50000, 20000, 10000};
	const int SenHmc5843::gainFactor[] =  {1620,1300,970,780,530,460,390,280};

	SenHmc5843::SenHmc5843(unsigned short _dev_id, unsigned short _func_id, string _port, int _update_rate, int _debug, int _timings, int _gain, int _mode) throw(const char *) {
		//FIXME Initialisierung
		dev_id = _dev_id;
		func_id = _func_id;
		update_rate = _update_rate;
		debug = _debug;
		timings = _timings; 

		// limit parameters to valid values
		mode = _mode < 4? _mode: 1; // default: single-conversion mode
		gain = _gain < 8? _gain: 1; // default: +-1Gs
		update_rate = _update_rate < 8? _update_rate: DRDEFAULT; // default: 10Hz

		uint8_t buffer[2];

		Logger::debug("hmc5843: init...");

		status = RUNNING;	
		try {
			/* check config */
			if (get_data_pointer(func_id << 16) == NULL) {
				throw "sensor doesn't support configured function";
			}

			Logger::log("hmc5843: init", _port, Logger::LOGLEVEL_DEBUG);

			fd = i2c_init(_port);
			i2c_start_conversion(fd, HMC5843_ADR);

			/* mode */
			buffer[0] = MR;
			buffer[1] = mode;

			i2c_write_bytes(fd, buffer, 2);

			/* setup gain */
			buffer[0] = CRB;
			buffer[1] = gain << 5;

			i2c_write_bytes(fd, buffer, 2);

			/* setup data rate */
			buffer[0] = CRA;
			/* in single conversion mode data rate should be 0,3,4,7 - why ever */
			buffer[1] = (((mode == SINGLE_CONVERSION_MODE)? DR100HZ: update_rate) << 2);

			i2c_write_bytes(fd, buffer, 2);
			Logger::debug("done");
			status = STOPPED;
		}
		catch(const char *message) {
			status = UNINITIALIZED;
			i2c_end_conversion(fd);

			string s(message);
			throw ("SenHmc5843(): " + s).c_str();
		}

		i2c_end_conversion(fd);
	}

	SenHmc5843::~SenHmc5843() {
		status = STOPPED;
		join();
	}

	void SenHmc5843::run() {
		uint8_t buffer[6];
		int wait_time = waitFreq[update_rate];

		uint64_t frequency = wait_time;
		uint64_t start = get_time_us();
		uint64_t time_output = start + 1000000;
		uint64_t usec;

		Logger::debug("hmc5843: running");
		i2c_start_conversion(fd, HMC5843_ADR);
		try {
			status = RUNNING;

			while(status == RUNNING) {
				/* mode - start measurement */
				if (mode == SINGLE_CONVERSION_MODE) {
					buffer[0] = MR;
					buffer[1] = mode;

					i2c_write_bytes(fd, buffer, 2);
					/* device auto-increments its reg-pointer to data-adress */
				} else {
					/* CONTINUOUS_CONVERSION_MODE - set reg-pointer to data-adress */
					buffer[0] = DATA;
					i2c_write_bytes(fd, buffer, 2);	
				}
				i2c_end_conversion(fd);

				/* wait time */
				usec = get_time_us();
				uint64_t end = usec;
				wait_time = waitFreq[update_rate] - (end - start);
				wait_time = (wait_time < 0)? 0: wait_time;
				//Logger::log("hmc5843 frequency: ", frequency, Logger::LOGLEVEL_DEBUG);
				//Logger::log("hmc5843 wait_time: ", wait_time, Logger::LOGLEVEL_DEBUG);
		
				/* wait */
				usleep(wait_time);

				/* calculate frequency */
				end = get_time_us();
				frequency = (15 * frequency + end - start) / 16;
				start = end;

				/* get data */
				i2c_start_conversion(fd, HMC5843_ADR);
				i2c_read_bytes(fd, buffer, 6);

#ifdef MAVLINK_ENABLED_HUCH
				/* assign buffer to data */
				{ // begin of data mutex scope
					cpp_pthread::Lock ri_lock(data_mutex);
					kompass_data.data_x = (int16_t)((buffer[0] << 8) + buffer[1]);
					kompass_data.data_y = (int16_t)((buffer[2] << 8) + buffer[3]);
					kompass_data.data_z = (int16_t)((buffer[4] << 8) + buffer[5]);
					kompass_data.data_x /= gainFactor[gain];
					kompass_data.data_y /= gainFactor[gain];
					kompass_data.data_z /= gainFactor[gain];
					kompass_data.usec = usec;
				} // end of data mutex scope
				publish_data(end);
#endif // MAVLINK_ENABLED_HUCH

				/* debug data */
				if (debug) print_debug();

				/* timings/benchmark output */
				if (timings) {
					if (time_output <= end) {
						Logger::log("hmc5843 frequency: ", (float)1000000/frequency, Logger::LOGLEVEL_DEBUG);
						//Logger::log("hmc5843 wait_time: ", wait_time, Logger::LOGLEVEL_DEBUG);
						time_output += 1000000;
					}
				}
			}
		}
		catch(const char *message) {
			i2c_end_conversion(fd);
			status = STRANGE;

			string s(message);
			throw ("SenHmc5843::run(): " + s).c_str();
		}
		/* unlock i2c */
		i2c_end_conversion(fd);

		Logger::debug("hmc5843: stopped");
	}

	void SenHmc5843::print_debug() {
		//FIXME: remove variable send_stream (no extra allocation for debug messages)
		ostringstream send_stream;
#ifdef MAVLINK_ENABLED_HUCH
		send_stream << "hmc5843: " << kompass_data.data_x << ";" << kompass_data.data_y << ";" << kompass_data.data_z;
#else
		send_stream << "hmc5843: mavlink missing";
#endif // MAVLINK_ENABLED_HUCH

		Logger::debug(send_stream.str());
	}

	void SenHmc5843::publish_data(uint64_t time) {
#ifdef MAVLINK_ENABLED_HUCH
		// Logger::log("Cmp02 sensor:", i, chanmap[i], sensor_data[i].analog, Logger::LOGLEVEL_INFO);
		DataCenter::set_huch_magnetic_kompass(kompass_data);
#endif // MAVLINK_ENABLED_HUCH
	}

	void* SenHmc5843::get_data_pointer(unsigned int id) throw(const char *) {
		if (status == RUNNING) {
			switch ((0xFFFF0000 & id) >> 16) {
#ifdef MAVLINK_ENABLED_HUCH
			case KOMPASS_SENSOR: // kompass 
				return &kompass_data;
#endif // MAVLINK_ENABLED_HUCH
			default: throw "sensor hmc5843 doesn't support this sensor type";
			}
		} throw "sensor hmc5843 isn't running";
	}

} // namespace mavh
