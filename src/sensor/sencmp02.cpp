#include "sencmp02.h"

#include <cstring> //memcpy
#include <math.h> //pow
#include <iostream> //cout
#include <sys/time.h> //gettime
#include <sstream> //stringstream

#include "core/logger.h" //"printf"
#include "lib/hub/time.h"
#include "core/datacenter.h"

using namespace std;
using namespace hub;

namespace mavhub {

	SenCmp02::SenCmp02(unsigned short _dev_id, 
										 unsigned short _func_id, 
										 std::string _port, 
										 int _update_rate, 
										 int _debug, 
										 int _timings,
										 std::list< std::pair<int, int> > _chanmap_pairs
										 )
		throw(const char *) :
		version(254),
		chanmap(CMP02_NUMCHAN)
	{
	//FIXME Initialisierung
	dev_id = _dev_id;
	func_id = _func_id;
	update_rate = _update_rate;
	debug = _debug;
	timings = _timings; 

	Logger::log("Cmp02: init...", timings, Logger::LOGLEVEL_INFO);

	// channel mapping
	list<pair<int, int> >::const_iterator iter;
	iter = _chanmap_pairs.begin();
	for(int i = 0; i < CMP02_NUMCHAN; i++) {
		// FIXME: rather two concurrent iterators?
		chanmap[i] = iter->second;
		Logger::log("Cmp02: chantype", chanmap[i], Logger::LOGLEVEL_DEBUG);
		iter++;
	}

	status = RUNNING;	
	try {
		/* check config */
		if (get_data_pointer(func_id << 16) == NULL) {
			throw "sensor doesn't support configured function";
		}

		fd = i2c_init(_port);
		status = STOPPED;
	}
	catch(const char *message) {
		status = UNINITIALIZED;

		string s(message);
		throw ("Cmp02(): " + s).c_str();
	}
}

SenCmp02::~SenCmp02() {
	status = STOPPED;
	join();
}

void SenCmp02::run() {

	char buffer[256];
	int wait_freq = update_rate? 1000000 / update_rate: 0;
	int wait_time = wait_freq;

	uint64_t frequency = wait_time;
	uint64_t start = get_time_us();
	uint64_t time_output = start + 1000000;
	uint64_t usec;
	vector<uint16_t> cmp_value(2);
	//mavlink_message_t msg;

	Logger::log("Cmp02: running (Hz)", update_rate, Logger::LOGLEVEL_INFO);
	// uint64_t end = getTimeUs() + 1000000;

	status = RUNNING;

	while((status == RUNNING) && update_rate) {
		try {

			/* wait time */
			usec = get_time_us();
			uint64_t end = usec;
			wait_time = wait_freq - (end - start);
			wait_time = (wait_time < 0)? 0: wait_time;
		
			/* wait */
			usleep(wait_time);
			//usleep(10);

			/* calculate frequency */
			end = get_time_us();
			frequency = (15 * frequency + end - start) / 16;
			start = end;

			// Logger::log("sencmp2: pre i2c_start_conversion", Logger::LOGLEVEL_DEBUG);
			i2c_start_conversion(fd, CMP02_ADR);

			// set read register
			buffer[0] = 0x00;
			// Logger::log("sencmp2: pre i2c_write_bytes", Logger::LOGLEVEL_DEBUG);
			i2c_write_bytes(fd, (uint8_t*)buffer, 1);

			// read data
			// Logger::log("sencmp2: pre i2c_read_bytes", Logger::LOGLEVEL_DEBUG);
			i2c_read_bytes(fd, (uint8_t*)buffer, 1);

			// get version
			version = *buffer;

			// Logger::log("Cmp02: version", (int)version, Logger::LOGLEVEL_INFO);

			buffer[0] = 0x01;
			// Logger::log("sencmp2: pre i2c_write_bytes 2", Logger::LOGLEVEL_DEBUG);
			i2c_write_bytes(fd, (uint8_t*)buffer, 1);
			// Logger::log("sencmp2: pre i2c_read_bytes 2", Logger::LOGLEVEL_DEBUG);
			i2c_read_bytes(fd, (uint8_t*)buffer, 1);
			
			// Logger::log("sencmp2: pre i2c_end_conversion", Logger::LOGLEVEL_DEBUG);
			i2c_end_conversion(fd);

			// get values
			memcpy(&cmp_value[0], buffer, 1);

			// Logger::log("Cmp02: value", (int)cmp_value[0], Logger::LOGLEVEL_INFO);

			// FIXME: smart++
			// exp_ctrl_rx_data.value0 = cmp_value[0];
			// exp_ctrl_rx_data.value1 = cmp_value[1];
			// exp_ctrl_rx_data.value2 = cmp_value[2];
			// exp_ctrl_rx_data.value3 = cmp_value[3];

			// FIXME: kopter specific mapping
			// FIXME: 0 is USS
			// huch_ranger.ranger2 = cmp_value[2];
			// huch_ranger.ranger3 = cmp_value[0];

			// home / qk01
			// huch_ranger.ranger2 = cmp_value[0];
			// huch_ranger.ranger3 = cmp_value[1];

			/* assign buffer to data */
#ifdef MAVLINK_ENABLED_HUCH
			{ // begin of data mutex scope
				int i;
				Lock ri_lock(data_mutex);
				for(i=0; i < CMP02_NUMCHAN; i++) {
					sensor_data[i].analog = cmp_value[i];
					sensor_data[i].usec = start;
					// Logger::log("Cmp02 sensor:", i, sensor_data[i].analog, Logger::LOGLEVEL_INFO);
				}
			} // end of data mutex scope
#endif // MAVLINK_ENABLED_HUCH
	
			// FIXME: if(publish) else poll or whatever
			publish_data(start);

			//Logger::log("Cmp02:", (int)exp_ctrl_rx_data.version, cmp_value, Logger::LOGLEVEL_INFO);
			//Logger::log("Cmp02:", DataCenter::get_sensor(chanmap[0]), Logger::LOGLEVEL_INFO);

			// pass more data
			// FIXME: system_id
			// mavlink_msg_huch_exp_ctrl_rx_encode(39, static_cast<uint8_t>(component_id), &msg, &exp_ctrl_rx_data);

			/* debug data */
			if (debug) print_debug();

			/* timings/benchmark output */
			if (timings) {
				if (time_output <= end) {
					Logger::log("Cmp02 frequency: ", (float)1000000/frequency, Logger::LOGLEVEL_DEBUG);
					time_output += 1000000;
				}
			}
		} // end try
		catch(const char *message) {
			i2c_end_conversion(fd);
			status = STRANGE;

			string s(message);
			Logger::log("sencmp2: would-be exception:", s, Logger::LOGLEVEL_DEBUG);
			// fallback termination and free bus
			i2c_end_conversion(fd);
			Logger::log("sencmp2: would-be exception:", s, Logger::LOGLEVEL_DEBUG);

			// FIXME: throw and exit on "write_bytes 2"
			// throw ("Cmp02::run(): " + s).c_str();
			status = RUNNING;
		}
	} // end while

	Logger::debug("sencmp02: stopped, exiting run()");
	return;
}

void SenCmp02::print_debug() {
	ostringstream send_stream;
	// send_stream << "Cmp02 version: " << (uint8_t)version << ", cmpval:" << sensor_data[0];
	// Logger::debug(send_stream.str());
}

void SenCmp02::publish_data(uint64_t time) {
#ifdef MAVLINK_ENABLED_HUCH
	int i;
	for(i=0; i < CMP02_NUMCHAN; i++) {
		// Logger::log("Cmp02 sensor:", i, chanmap[i], sensor_data[i].analog, Logger::LOGLEVEL_INFO);
		DataCenter::set_sensor(chanmap[i], (double)sensor_data[i].analog);
	}
#endif // MAVLINK_ENABLED_HUCH
}

void* SenCmp02::get_data_pointer(unsigned int id) throw(const char *) {
	// if (status == RUNNING) {
	// 	switch ((0xFFFF0000 & id) >> 16) {
	// 		case (DISTANCE_SENSOR): // sensor 0
	// 			return &sensor_data[0];
	// 		case (DISTANCE_SENSOR | 0x1000): // sensor 1
	// 			return &sensor_data[1];
	// 		default: throw "sensor Cmp02 doesn't support this sensor type";
	// 	}
	// } throw "sensor exp_ctrl isn't running";
#ifdef MAVLINK_ENABLED_HUCH
	return &sensor_data[0];
#else
	throw "SenCmp02: mavlink missing";
#endif // MAVLINK_ENABLED_HUCH
}


} // namespace mavhub
