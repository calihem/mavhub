// read devantech / robot-electronics SRF* type sonar sensors

#include "sensrf02.h"

#include "core/logger.h" //"printf"
#include "utility.h"
#include "../core/datacenter.h"
#include "../module/filter_median.h"

#include <list>
#include <math.h> //pow
#include <iostream> //cout

using namespace std;

namespace mavhub {

	const int SenSrf02::waitFreq[] = {2000000, 1000000, 500000, 200000, 100000, 50000, 20000, 10000};

	//SenSrf02::SenSrf02(unsigned short _dev_id, unsigned short _func_id, string _port, int _update_rate, int _debug, int _timings, int _gain, int _mode) throw(const char *) {
	SenSrf02::SenSrf02(unsigned short _dev_id, unsigned short _func_id, string _port, int _update_rate, int _debug, int _timings, std::list< std::pair<int, int> > _chanmap_pairs) throw(const char *) :
		chanmap(1)
	{	
		//FIXME Initialisierung
		dev_id = _dev_id;
		func_id = _func_id;
		update_rate = _update_rate;
		debug = _debug;
		timings = _timings; 
		//Logger::log("update_rate in ctor", update_rate, Logger::LOGLEVEL_INFO);

		// limit parameters to valid values
		// mode = _mode < 4? _mode: 1; // default: single-conversion mode
		// gain = _gain < 8? _gain: 1; // default: +-1Gs
		update_rate = _update_rate < 8? _update_rate: DRDEFAULT; // default: 10Hz

		//uint8_t buffer[2];

		Logger::log("sensrf02: init, port", _port, Logger::LOGLEVEL_INFO);

		//chanmap.reserve(1);
		list<pair<int, int> >::const_iterator iter;
		iter = _chanmap_pairs.begin();
		for(int i = 0; i < 1; i++) {
			// FIXME: rather two concurrent iterators?
			chanmap[i] = iter->second;
			Logger::log("sensrf02: init, chantype", chanmap[i], Logger::LOGLEVEL_INFO);
			iter++;
		}


		status = RUNNING;	
		try {
			// /* check config */
			// if (get_data_pointer(func_id << 16) == NULL) {
			// 	throw "sensor doesn't support configured function";
			// }

			fd = i2c_init(_port);

			i2c_start_conversion(fd, SRF02_ADR);
			get_hw_version();

			/* mode */
			//buffer[0] = MR;
			//buffer[1] = mode;

			//i2c_write_bytes(fd, buffer, 2);

			/* setup gain */
			//buffer[0] = CRB;
			//buffer[1] = gain;

			//i2c_write_bytes(fd, buffer, 2);

			set_up();

			/* setup data rate */
			//buffer[0] = CRA;
			/* in single conversion mode data rate should be 0,3,4,7 - why ever */
			//buffer[1] = (mode == SINGLE_CONVERSION_MODE)? DR100HZ: update_rate;

			//i2c_write_bytes(fd, buffer, 2);
			Logger::log("sensrf02: init done", Logger::LOGLEVEL_INFO);
			status = STOPPED;
		}
		catch(const char *message) {
			status = UNINITIALIZED;
			i2c_end_conversion(fd);

			string s(message);
			throw ("SenSrf02(): " + s).c_str();
		}

		i2c_end_conversion(fd);
	}

	SenSrf02::~SenSrf02() {
		status = STOPPED;
		join();
	}

	void SenSrf02::run() {
		//uint8_t buffer[6];
		int wait_time = waitFreq[update_rate];

		uint64_t frequency = wait_time;
		uint64_t start = get_time_us();
		uint64_t time_output = start + 1000000;
		uint64_t usec;

		MEDIAN filt_med(5);

		Logger::log("sensrf02: running (Hz)", 1e6 / wait_time, Logger::LOGLEVEL_INFO);
		i2c_start_conversion(fd, SRF02_ADR);
		try {
			status = RUNNING;

			while(status == RUNNING) {
				/* mode - start measurement */
				// if (mode == SINGLE_CONVERSION_MODE) {
				// 	buffer[0] = MR;
				// 	buffer[1] = mode;

				// 	i2c_write_bytes(fd, buffer, 2);
				// 	/* device auto-increments its reg-pointer to data-adress */
				// } else {
				/* CONTINUOUS_CONVERSION_MODE - set reg-pointer to data-adress */
				// buffer[0] = DATA;
				// i2c_write_bytes(fd, buffer, 2);	
				start_ranging();
				// }
				i2c_end_conversion(fd);

				/* wait time */
				usec = get_time_us();
				uint64_t end = usec;
				wait_time = waitFreq[update_rate] - (end - start);
				wait_time = (wait_time < 0)? 0: wait_time;
				//Logger::log("sensrf02 frequency: ", frequency, Logger::LOGLEVEL_INFO);
				//Logger::log("sensrf02 wait_time: ", wait_time, Logger::LOGLEVEL_DEBUG);
		
				/* wait */
				usleep(wait_time);

				/* calculate frequency */
				end = get_time_us();
				frequency = (15 * frequency + end - start) / 16;
				start = end;

				/* get data */
				i2c_start_conversion(fd, SRF02_ADR);
				//i2c_read_bytes(fd, buffer, 6);


				/* assign buffer to data */
				{ // begin of data mutex scope
					cpp_pthread::Lock ri_lock(data_mutex);
					//Logger::log("sensrf02 data locked", Logger::LOGLEVEL_INFO);
					// use median filter
					// sensor_data[0].distance = (int)filt_med.calc((double)get_range());
					// use raw value
#ifdef MAVLINK_ENABLED_HUCH
					sensor_data[0].distance = get_range();
#endif // MAVLINK_ENABLED_HUCH
					//Logger::log("sensrf02 data acquired", Logger::LOGLEVEL_INFO);
					publish_data(start);
					//Logger::log("sensrf02 data published", Logger::LOGLEVEL_INFO);
				} // end of data mutex scope

				// Logger::log("sensrf02: read", sensor_data[0].distance, Logger::LOGLEVEL_INFO);

				/* debug data */
				if (debug) print_debug();

				/* timings/benchmark output */
				if (timings) {
					if (time_output <= end) {
						Logger::log("sensrf02 frequency: ", (float)1000000/frequency, Logger::LOGLEVEL_DEBUG);
						//Logger::log("srf02 wait_time: ", wait_time, Logger::LOGLEVEL_DEBUG);
						time_output += 1000000;
					}
				}
			}
		}
		catch(const char *message) {
			//i2c_end_conversion(fd);
			status = STRANGE;

			string s(message);
			throw ("SenSrf02::run(): " + s).c_str();
		}
		/* unlock i2c */
		//i2c_end_conversion(fd);

		Logger::log("sensrf02: stopped", Logger::LOGLEVEL_INFO);
	}

	void SenSrf02::print_debug() {
		ostringstream send_stream;
#ifdef MAVLINK_ENABLED_HUCH
		//send_stream << "srf02;" << sensor_data[0] << ";" << kompass_data.data_y << ";" << kompass_data.data_z;
		send_stream << "sensrf02;" << sensor_data[0].distance << ";";
#else
	send_stream << "sensrf02: mavlink missing";
#endif // MAVLINK_ENABLED_HUCH
		Logger::debug(send_stream.str());
	}

	void* SenSrf02::get_data_pointer(unsigned int id) throw(const char *) {
		if (status == RUNNING) {
			switch ((0xFFFF0000 & id) >> 16) {
#ifdef MAVLINK_ENABLED_HUCH
			case KOMPASS_SENSOR: // kompass 
				return &sensor_data;
#endif // MAVLINK_ENABLED_HUCH
			default: throw "sensor srf02 doesn't support this sensor type";
			}
		} throw "sensor srf02 isn't running";
	}

	int SenSrf02::get_hw_version() {
		uint8_t buffer[2];
		buffer[0] = SRF02_REG_RD_VERSION; // read from version register
		i2c_write_bytes(fd, (uint8_t *)buffer, 1);
	
		i2c_read_bytes(fd, (uint8_t *)buffer, 1);
		Logger::log("sensrf02: version", (int)buffer[0], Logger::LOGLEVEL_INFO);
	
		return buffer[0];
	}

	void SenSrf02::set_up() {
		uint8_t buffer[2];
		// limit max gain
		buffer[0] = SRF02_REG_WR_GAIN;
		buffer[1] = 0x0A; // half the default max gain
		i2c_write_bytes(fd, (uint8_t *)buffer, 2);

		// limit max range
		buffer[0] = SRF02_REG_WR_RNG;
		buffer[1] = 0x8C; // 6m - 140 * 43mm
		i2c_write_bytes(fd, (uint8_t *)buffer, 2);
	
		// i2c_read_bytes(fd, (uint8_t *)buffer, 1);
		Logger::log("sensrf02: set_up", Logger::LOGLEVEL_INFO);
	
		// return buffer[0];
		return;
	}

	void SenSrf02::start_ranging() {
		uint8_t buffer[2];
		buffer[0] = SRF02_REG_WR_CMD; // command register
		buffer[1] = SRF02_CMD_STARTRNG_SPEC; // in us
		i2c_write_bytes(fd, (uint8_t *)buffer, 2);
		return;
	}

	int SenSrf02::get_range() {
		int reading;
		double reading_mm;
		uint8_t buffer[2];
		buffer[0] = SRF02_REG_RD_ECHO_0; // command register
		i2c_write_bytes(fd, (uint8_t *)buffer, 1);

		i2c_read_bytes(fd, (uint8_t *)buffer, 2);

		reading = (int)(buffer[0] << 8) | buffer[1];
		reading_mm = reading * 0.17;
		return reading;
	}

	int SenSrf02::get_last_measurement() {
		int reading;
#ifdef MAVLINK_ENABLED_HUCH
		{ // begin of data mutex scope
			cpp_pthread::Lock ri_lock(data_mutex);
			reading = sensor_data[0].distance;
		}
#endif // MAVLINK_ENABLED_HUCH
		return reading;
	}

	void SenSrf02::publish_data(uint64_t time) {
		//DataCenter::set_exp_ctrl(exp_ctrl_rx_data);
		// FIXME: hardware specific mapping
		//DataCenter::set_huch_ranger_at(huch_ranger, 1);
#ifdef MAVLINK_ENABLED_HUCH
		DataCenter::set_sensor(chanmap[0], (double)sensor_data[0].distance);
#endif // MAVLINK_ENABLED_HUCH
	}
} // namespace mavh
