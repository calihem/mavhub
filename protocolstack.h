#ifndef _PROTOCOLSTACK_H_
#define _PROTOCOLSTACK_H_

#include "thread.h"
#include <inttypes.h> //uint8_t
#include "protocollayer.h"
#include <list>
#include <vector>
#include <ostream> 

namespace mavhub {

	class ProtocolStack : public PThread {
		public:
			/// Enumeration of supported packet formats
			enum packageformat_t {
				MAVLINKPACKAGE,	//MAVLINK packet
				MKPACKAGE	//MikroKopter packet
			};
			typedef std::pair<MediaLayer*, packageformat_t> interface_packet_pair_t; 
			typedef std::list<interface_packet_pair_t> interface_packet_list_t;
			typedef std::list< std::vector<uint8_t> > buffer_list_t;

			static ProtocolStack& instance();
			friend std::ostream& operator <<(std::ostream &os, const ProtocolStack &proto_stack);

			/// Set system ID
			void system_id(uint8_t system_id);
			/// Get system ID
			uint8_t system_id() const;
			
			int add_link(MediaLayer *interface, const packageformat_t format);
			int remove_link(unsigned int link_id);
			
			void add_application(AppLayer *app);

			void send(const mavlink_message_t &msg) const;
// 			void join();

		protected:
			virtual void run();


		private:
			/// Private singleton constructor
			ProtocolStack(uint8_t system_id = -1);
			~ProtocolStack();
			ProtocolStack(const ProtocolStack &); // intentionally undefined
			ProtocolStack& operator=(const ProtocolStack &); // intentionally undefined
	
			/// Size of rx buffer
			static const int BUFFERLENGTH = 512;
			/// Polling interval through all interfaces in us
			static const int POLLINTERVAL = 1000000;//FIXME

			/// System ID
			uint8_t sys_id;
			/// List of all links/ interfaces
			interface_packet_list_t interface_list;
			/// Mutex to protect link/ interface management structures
			mutable pthread_mutex_t link_mutex;
			/// List of all registered applications
			std::list<AppLayer*> app_list;
			/// receive buffers
			buffer_list_t rx_buffer_list;
			/// transmit buffer
			mutable uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];
			/// mutex to protect tx_buffer
			mutable pthread_mutex_t tx_mutex;

			/// transmit msg to every app in app_list
			void transmit_to_apps(const mavlink_message_t &msg) const;
			/// transmit msg on every MediaLayer except src_iface
			void retransmit(const mavlink_message_t &msg, const MediaLayer *src_iface) const;
	};
	// ----------------------------------------------------------------------------
	// ProtocolStack
	// ----------------------------------------------------------------------------
	inline void ProtocolStack::system_id(uint8_t system_id) {
		sys_id = system_id;
	}
	inline uint8_t ProtocolStack::system_id() const {
		return sys_id;
	}
	inline void ProtocolStack::transmit_to_apps(const mavlink_message_t &msg) const {
		std::list<AppLayer*>::const_iterator app_iter;
		for(app_iter = app_list.begin(); app_iter != app_list.end(); ++app_iter) {
			(*app_iter)->handle_input(msg);
		}
	}


} // namespace mavhub

#endif
