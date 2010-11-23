#ifndef _FACTORY_H_
#define _FACTORY_H_

#include <inttypes.h> //uint8_t
#include <string>
#include <algorithm>
#include <sstream> //stringstream
#include <iterator> //istream_iterator
#include <vector>

#include "protocollayer.h"
#include "protocolstack.h"
#include "lib/setting.h"
#include "utility.h"
// Sensors
#include "module/i2csensor.h"
#include "module/senbmp085.h"
#include "module/senhmc5843.h"


#include "module/coremod.h"
#include "module/testcore.h"
#include "module/fc_mpkg.h"

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

		struct link_construction_plan_t {
			link_construction_plan_t();

			/* generic */
			link_type_t link_type;
			ProtocolStack::packageformat_t package_format;
			/* serial link */
			std::string dev_name;
			unsigned int baudrate;
			/* UDP */
			uint16_t port;
			std::list<string_addr_pair_t> groupmember_list;
		};

		static MediaLayer* build(const link_construction_plan_t &plan);
		static MediaLayer* build(const std::string& type, const std::string& devicename);
};

class AppFactory {
	public:
		static AppLayer* build(const std::string& app_name, const std::map<std::string, std::string> args);
};
	
// ----------------------------------------------------------------------------
// SensorFactory
// ----------------------------------------------------------------------------
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
	/* hmc5843 sensor config */
	if( settings.begin_group("hmc5843") == 0) {
		int update_rate = 0;
		int gain = 0;
		int mode = 0;
		int output = 0;
		if( settings.value("update_rate", update_rate) ) {
			Logger::log("hmc5843 update rate is missing in config file: ", filename, Logger::LOGLEVEL_WARN);
		}
		if( settings.value("gain", gain) ) {
			Logger::log("hmc5843 gain is missing in config file: ", filename, Logger::LOGLEVEL_WARN);
		}
		if( settings.value("mode", mode) ) {
			Logger::log("hmc5843 mode is missing in config file: ", filename, Logger::LOGLEVEL_WARN);
		}

		bool value;
		if( settings.value("debug_data", value) ) {
		} else if (value) output |= DEBUG;

		if( settings.value("show_timings", value) ) {
		} else if (value) output |= TIMINGS;

		/* create instance */
		i2cSensors.push_back(new SenHmc5843(file, update_rate, gain, mode, output));

		settings.end_group();
	}
}

// ----------------------------------------------------------------------------
// LinkFactory
// ----------------------------------------------------------------------------
inline LinkFactory::link_construction_plan_t::link_construction_plan_t() :
		link_type(UnsupportedLink),
		package_format(ProtocolStack::MAVLINKPACKAGE),
		dev_name(),
		baudrate(57600),
		port(0),
		groupmember_list()
{}

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

inline MediaLayer* LinkFactory::build(const link_construction_plan_t &plan) {
	MediaLayer *layer(NULL);

	switch(plan.link_type) {
		case SerialLink:
			try{
				layer = new UARTLayer(plan.dev_name, UART::baudrate_to_speed(plan.baudrate) | CS8 | CLOCAL | CREAD);
			}
			catch(const char *message) {
				Logger::error(message);
			}
			break;
		case UDPLink:
			try{
				if( (layer = new UDPLayer(plan.port)) ) {
					UDPLayer *udp_layer = dynamic_cast<UDPLayer*>(layer);
					if(udp_layer) {
						// add udp group members
						try{
							udp_layer->add_groupmembers(plan.groupmember_list);
						}
						catch(const char *message) {
							Logger::log(message, Logger::LOGLEVEL_DEBUG);
						}
					}
				}
			}
			catch(const char *message) {
				Logger::error(message);
			}
			break;
		default:
			break;
	}

	return layer;
}

inline MediaLayer* LinkFactory::build(const std::string& type, const std::string& devicename) {
	link_construction_plan_t construction_plan;
	
	//transform type to lower case
	std::string lowercase_type(type);
	transform(lowercase_type.begin(), lowercase_type.end(), lowercase_type.begin(), ::tolower);
	
	if(lowercase_type == "seriallink"
	|| lowercase_type == "0"
	|| lowercase_type == "serial"
	|| lowercase_type == "uart") {
		construction_plan.link_type = SerialLink;
		construction_plan.dev_name = devicename;
	} else if(lowercase_type == "udp"
	|| lowercase_type == "1"
	|| lowercase_type == "udpport"
	|| lowercase_type == "udplayer") {
		construction_plan.link_type = UDPLink;
		std::istringstream istream(devicename);
		istream >> construction_plan.port;
	}

	return build(construction_plan);
}

// ----------------------------------------------------------------------------
// AppFactory
// ----------------------------------------------------------------------------
inline AppLayer* AppFactory::build(const std::string& app_name, const std::map<std::string, std::string> args) {
	//transform application name to lower case
	std::string lowercase_name(app_name);
	transform(lowercase_name.begin(), lowercase_name.end(), lowercase_name.begin(), ::tolower);

	if(lowercase_name == "test_app") {
		return new TestCore();
	} else if(lowercase_name == "fc_mpkg_app") {
		return new FC_Mpkg();
	}
	
	return NULL;
}

} // namespace mavhub

#endif
