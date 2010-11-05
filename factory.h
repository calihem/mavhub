#ifndef _FACTORY_H_
#define _FACTORY_H_

#include <inttypes.h> //uint8_t
#include <string>
#include <algorithm>
#include "protocollayer.h"
#include <sstream> //stringstream

namespace mavhub {

class LinkFactory {
	public:
		/// Enumeration of supported link types
		enum link_type_t {
			SerialLink,
			UDPLink
		};
	
		static MediaLayer* build(link_type_t type,
										 const std::string& devicename,
										 dev_rate_t devicerate);
		static MediaLayer* build(link_type_t type, uint16_t port);
		static MediaLayer* build(const std::string& type, const std::string& devicename);

};

inline MediaLayer* LinkFactory::build(LinkFactory::link_type_t type,
												  const std::string& devicename,
												  dev_rate_t devicerate) {
  
	try{
		switch(type) {
			case SerialLink:
			  // Logger::log("rate: ", devicerate, Logger::LOGLEVEL_DEBUG);
			  return new UARTLayer(devicename, (devicerate | CS8 | CLOCAL | CREAD));
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

inline MediaLayer* LinkFactory::build(LinkFactory::link_type_t type, uint16_t port) {
	
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
	  return build(SerialLink, devicename, 57600);
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
