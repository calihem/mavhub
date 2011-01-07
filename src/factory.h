#ifndef _FACTORY_H_
#define _FACTORY_H_

#include <inttypes.h> //uint8_t
#include <string>
#include <algorithm>
#include <sstream> //stringstream
#include <iterator> //istream_iterator
#include <vector>
#include <map>

#include "core/protocollayer.h"
#include "core/protocolstack.h"
#include "core/setting.h"
#include "utility.h"

// sub-factories
#include "application/factory_app.h"
#include "sensor/sensor_factory.h"

// Sensors
#include "sensor/i2csensor.h"
#include "sensor/senbmp085.h"
#include "sensor/senhmc5843.h"
#include "sensor/senExpCtrl.h"

#include "core/logger.h"

namespace mavhub {

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

		static cpp_io::IOInterface* build(const link_construction_plan_t &plan);
		static cpp_io::IOInterface* build(const std::string& type, const std::string& devicename);
};

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

inline cpp_io::IOInterface* LinkFactory::build(const link_construction_plan_t &plan) {
	cpp_io::IOInterface *layer(NULL);

	switch(plan.link_type) {
		case SerialLink:
			try{
				layer = new UART(plan.dev_name, UART::baudrate_to_speed(plan.baudrate) | CS8 | CLOCAL | CREAD);
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

inline cpp_io::IOInterface* LinkFactory::build(const std::string& type, const std::string& devicename) {
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


} // namespace mavhub

#endif
