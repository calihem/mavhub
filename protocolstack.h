#ifndef _PROTOCOLSTACK_H_
#define _PROTOCOLSTACK_H_

#include "thread.h"
#include <inttypes.h> //uint8_t
#include "protocollayer.h"
#include <list>

namespace mavhub {

	class ProtocolStack : public PThread {
		public:
			enum packageformat_t {
				MAVLINKPACKAGE,	//MAVLINK packet
				MKPACKAGE	//MikroKopter packet
			};
			typedef std::pair<MediaLayer*, packageformat_t> interface_packet_pair_t; 
			typedef std::list<interface_packet_pair_t> interface_packet_list_t;
			typedef std::list<MediaLayer*> interface_list_t;
			
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
			uint8_t system_id;
			interface_packet_list_t interface_list;

	};
	// ----------------------------------------------------------------------------
	// ProtocolStack
	// ----------------------------------------------------------------------------
// 	inline int ProtocolStack::localhost() const { return hostID; }


} // namespace mavhub

#endif
