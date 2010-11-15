#ifndef _FACTORY_H_
#define _FACTORY_H_

#include <inttypes.h> //uint8_t
#include <string>
#include <algorithm>
#include <sstream> //stringstream
#include <iterator> //istream_iterator
#include <vector>

#include "protocollayer.h"
#include "lib/setting.h"
#include "utility.h"
#include "module/i2csensor.h"
#include "module/senbmp085.h"

namespace mavhub {

class SensorFactory {
	public:
		static void build( std::list<I2cSensor*>& i2cSensors, const std::string& filename);
};

class LinkFactory {
	public:
		/// Enumeration of link types
		enum link_type_t {
			SerialLink = 0,
			UDPLink = 1,
			UnsupportedLink = 255
		};
		friend std::ostream& operator <<(std::ostream &os, const link_type_t &link_type);
		friend std::istream& operator >>(std::istream &is, link_type_t &link_type);

		static MediaLayer* build(const link_type_t type, const std::string& devicename, const unsigned int baudrate);
		static MediaLayer* build(const link_type_t type, const uint16_t port);
		static MediaLayer* build(const std::string& type, const std::string& devicename);
};

inline std::ostream& operator <<(std::ostream &os, const LinkFactory::link_type_t &link_type) {
	os << static_cast<int>(link_type);

	return os;
}

inline std::istream& operator >>(std::istream &is, LinkFactory::link_type_t &link_type) {
	int num_link_type;
	is >> num_link_type;
	if(num_link_type >= 0 && num_link_type <= 1)
		link_type = static_cast<LinkFactory::link_type_t>(num_link_type);
	else
		link_type = LinkFactory::UnsupportedLink;

	return is;
}

inline void SensorFactory::build(std::list<I2cSensor*>& i2cSensors, const std::string& filename) {
	//open config file
	cpp_io::Setting settings(filename, std::ios_base::in);

	/* i2c port config */
	std::string i2c_port;
	if( settings.value("port", i2c_port) ) {
		Logger::log("Port is missing in config file:", filename, "for i2c", Logger::LOGLEVEL_WARN);
		return;
	}
	int file;
	if ((file = open(i2c_port.c_str(), O_RDWR)) < 0) {
		Logger::warn("Failed to open the i2c bus");	
		return;
	}

	/* bmp085 sensor config */
	if( settings.begin_group("bmp085") == 0) {
		int oversampling = 0;
		int temp_update_rate = 0;
//		std::list<std::string> output_list;
		std::string output_string;
		int output = 0;
		if( settings.value("oversampling", oversampling) ) {
			Logger::log("bmp085 oversampling is missing in config file: ", filename, Logger::LOGLEVEL_WARN);
		}
		if( settings.value("temp_update_rate", temp_update_rate) ) {
			Logger::log("bmp085 temp_update_rate is missing in config file: ", filename, Logger::LOGLEVEL_WARN);
		}

		bool value;
		if( settings.value("debug_data", value) ) {
		} else if (value) output |= DEBUG;

		if( settings.value("show_timings", value) ) {
		} else if (value) output |= TIMINGS;

		/* create instance */
		i2cSensors.push_back(new SenBmp085(file, temp_update_rate, oversampling, output));

		settings.end_group();
	}
}

inline MediaLayer* LinkFactory::build(const LinkFactory::link_type_t type, const std::string& devicename, const unsigned int baudrate) {

	try{
		switch(type) {
			case SerialLink:
				return new UARTLayer(devicename, UART::baudrate_to_speed(baudrate) | CS8 | CLOCAL | CREAD);
				break;
			default:
				break;
		}
	}
	catch(const char *message) {
		Logger::error(message);
	}

	return NULL;
}

inline MediaLayer* LinkFactory::build(const LinkFactory::link_type_t type, uint16_t port) {
	
	try{
		switch(type) {
			case UDPLink:
				return new UDPLayer(port);
				break;
			default:
				break;
		}
	}
	catch(const char *message) {
		Logger::error(message);
	}

	return NULL;
}

inline MediaLayer* LinkFactory::build(const std::string& type, const std::string& devicename) {
	//transform type to lower case
	std::string lowercase_type(type);
	transform(lowercase_type.begin(), lowercase_type.end(), lowercase_type.begin(), ::tolower);
	
	if(lowercase_type == "seriallink"
	|| lowercase_type == "0"
	|| lowercase_type == "serial"
	|| lowercase_type == "uart") {
		return build(SerialLink, devicename, 57600); //FIXME
	} else if(lowercase_type == "udp"
	|| lowercase_type == "1"
	|| lowercase_type == "udpport"
	|| lowercase_type == "udplayer") {
		std::istringstream istream(devicename);
		uint16_t port;
		istream >> port;
		return build(UDPLink, port);
	}
	
	return NULL;
}

} // namespace mavhub

#endif
