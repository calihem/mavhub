#ifndef _PROTOCOLSTACK_H_
#define _PROTOCOLSTACK_H_

#include "thread.h"
#include <inttypes.h> //uint8_t
#include "protocollayer.h"
#include <list>
#include <vector>

namespace mavhub {

	class ProtocolStack : public PThread {
		public:
			enum packageformat_t {
				MAVLINKPACKAGE,	//MAVLINK packet
				MKPACKAGE	//MikroKopter packet
			};
			typedef std::pair<MediaLayer*, packageformat_t> interface_packet_pair_t; 
			typedef std::list<interface_packet_pair_t> interface_packet_list_t;
			typedef std::list< std::vector<uint8_t> > buffer_list_t;

			ProtocolStack(uint8_t system_id);
			~ProtocolStack();

			void addInterface(MediaLayer *interface, const packageformat_t format);
// 			void addLayer(ProtocolEntity *layer);
// // 			void addApplication(ApplicationLayer &app, int port);
// 			void join();
// 			int localhost() const;

		protected:
			virtual void run();


		private:
			static const int BUFFERLENGTH = 512;
			uint8_t system_id;
			interface_packet_list_t interface_list;
			buffer_list_t rx_buffer_list;

	};
	// ----------------------------------------------------------------------------
	// ProtocolStack
	// ----------------------------------------------------------------------------
// 	inline int ProtocolStack::localhost() const { return hostID; }


} // namespace mavhub

#endif
