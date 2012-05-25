#include "link_factory.h"

#include <algorithm> //transform
#include <sstream> //stringstream
#include <iterator> //istream_iterator

#include "core/logger.h"

namespace mavhub {

LinkFactory::link_construction_plan_t::link_construction_plan_t() :
		link_type(UnknownLink),
		protocol_type(UnknownProtocol),
		dev_name(),
		baudrate(57600),
		port(0),
		groupmember_list()
{}

	void LinkFactory::link_construction_plan_t::clear() {
		link_type = UnknownLink;
		protocol_type = UnknownProtocol;
		dev_name.clear();
		baudrate = 57600;
		port = 0;
		groupmember_list.clear();
	}

std::ostream& operator <<(std::ostream &os, const LinkFactory::link_type_t &link_type) {
	switch(link_type) {
		case LinkFactory::SerialLink:
			os << "serial";
			break;
		case LinkFactory::UDPLink:
			os << "udp";
			break;
		default:
			os << "unknown";
			break;
	}

	return os;
}

std::istream& operator >>(std::istream &is, LinkFactory::link_type_t &link_type) {
	std::string link_type_str;
	is >> link_type_str;

	//transform type to lower case
	std::transform(link_type_str.begin(), link_type_str.end(), link_type_str.begin(), ::tolower);

	if(link_type_str == "serial"
	|| link_type_str == "0"
	|| link_type_str == "seriallink"
	|| link_type_str == "uart") {
		link_type = LinkFactory::SerialLink;
	} else if(link_type_str == "udp"
	|| link_type_str == "1"
	|| link_type_str == "udpport"
	|| link_type_str == "udplayer") {
		link_type = LinkFactory::UDPLink;
	} else {
		link_type = LinkFactory::UnknownLink;
	}

	return is;
}

cpp_io::IOInterface* LinkFactory::build(const link_construction_plan_t &plan) {
	cpp_io::IOInterface *layer(NULL);

	switch(plan.link_type) {
		case SerialLink:
			try{
				layer = new UART(plan.dev_name, UART::baudrate_to_speed(plan.baudrate) | CS8 | CLOCAL | CREAD);
			}
			catch(const std::exception& e) {
				Logger::error(e.what());
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
						catch(const std::exception& e) {
							Logger::log(e.what(), Logger::LOGLEVEL_DEBUG);
						}
					}
				}
			}
			catch(const std::exception& e) {
				Logger::error(e.what());
			}
			break;
		default:
			break;
	}

	return layer;
}

// cpp_io::IOInterface* LinkFactory::build(const std::string& type, const std::string& devicename) {
// 	link_construction_plan_t construction_plan;
// 	
// 	//transform type to lower case
// 	std::string lowercase_type(type);
// 	transform(lowercase_type.begin(), lowercase_type.end(), lowercase_type.begin(), ::tolower);
// 	
// 	if(lowercase_type == "seriallink"
// 	|| lowercase_type == "0"
// 	|| lowercase_type == "serial"
// 	|| lowercase_type == "uart") {
// 		construction_plan.link_type = SerialLink;
// 		construction_plan.dev_name = devicename;
// 	} else if(lowercase_type == "udp"
// 	|| lowercase_type == "1"
// 	|| lowercase_type == "udpport"
// 	|| lowercase_type == "udplayer") {
// 		construction_plan.link_type = UDPLink;
// 		std::istringstream istream(devicename);
// 		istream >> construction_plan.port;
// 	}
// 
// 	return build(construction_plan);
// }


} // namespace mavhub

