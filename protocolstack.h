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
			void addApplication(AppLayer *app);

			void send(const mavlink_message_t &msg) const;
// 			void join();
// 			int localhost() const;

		protected:
			virtual void run();


		private:
			/// size of rx buffer
			static const int BUFFERLENGTH = 512;
			/// polling interval through all interfaces in us
			static const int POLLINTERVAL = 1000000;//FIXME
			uint8_t system_id;
			interface_packet_list_t interface_list;
			buffer_list_t rx_buffer_list;
			std::list<AppLayer*> app_list;
			/// transmit buffer
			mutable uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];
			/// transmit msg to every app in app_list
			void transmit_to_apps(const mavlink_message_t &msg);
	};
	// ----------------------------------------------------------------------------
	// ProtocolStack
	// ----------------------------------------------------------------------------
	inline void ProtocolStack::transmit_to_apps(const mavlink_message_t &msg) {
		std::list<AppLayer*>::iterator app_iter;
		for(app_iter = app_list.begin(); app_iter != app_list.end(); ++app_iter) {
			(*app_iter)->handle_input(msg);
		}
	}

// 	inline int ProtocolStack::localhost() const { return hostID; }


} // namespace mavhub

#endif
