#ifndef _LINK_FACTORY_H_
#define _LINK_FACTORY_H_

#include "io/network.h"
#include "protocol/protocol.h"
#include "protocol/protocollayer.h"

#include <inttypes.h> //uint8_t
#include <string>
#include <list>

namespace mavhub {

class LinkFactory {
	public:
		/// Enumeration of link types
		enum link_type_t {
			SerialLink = 0,
			UDPLink = 1,
			UnknownLink = 255
		};
		friend std::ostream& operator <<(std::ostream &os, const link_type_t &link_type);
		friend std::istream& operator >>(std::istream &is, link_type_t &link_type);

		struct link_construction_plan_t {
			link_construction_plan_t();

			void clear();

			/* generic */
			link_type_t link_type;
			protocol_type_t protocol_type;
			/* serial link */
			std::string dev_name;
			unsigned int baudrate;
			/* UDP */
			uint16_t port;
			std::list<string_addr_pair_t> groupmember_list;
		};

		static cpp_io::IOInterface* build(const link_construction_plan_t &plan);
// 		static cpp_io::IOInterface* build(const std::string& type, const std::string& devicename);
};

} // namespace mavhub

#endif
