#ifndef _PROTOCOLSTACK_H_
#define _PROTOCOLSTACK_H_

#include "thread.h"
#include <inttypes.h> //uint8_t
#include "protocollayer.h"
#include <list>
#include <vector>
#include <ostream> 

namespace mavhub {
	class MKPackage;

	class ProtocolStack : public cpp_pthread::PThread {
		public:
			/// Enumeration of supported packet formats
			enum packageformat_t {
				MAVLINKPACKAGE,	//MAVLINK packet
				MKPACKAGE	//MikroKopter packet
			};
			friend std::ostream& operator <<(std::ostream &os, const packageformat_t &format);
			friend std::istream& operator >>(std::istream &is, packageformat_t &format);

			typedef std::pair<cpp_io::IOInterface*, packageformat_t> interface_packet_pair_t; 
			typedef std::list<interface_packet_pair_t> interface_packet_list_t;
			typedef std::list< std::vector<uint8_t> > buffer_list_t;
			typedef std::pair<uint16_t, AppLayer*> id_app_pair_t;

			static ProtocolStack& instance();
			friend std::ostream& operator <<(std::ostream &os, const ProtocolStack &proto_stack);

			/// Set system ID
			void system_id(const uint8_t system_id);
			/// Get system ID
			const uint8_t system_id() const;
			
			int add_link(cpp_io::IOInterface *interface, const packageformat_t format);
			cpp_io::IOInterface *link(unsigned int link_id);
			int remove_link(unsigned int link_id);

			void add_application(AppLayer *app);
			void send(const mavlink_message_t &msg, const AppLayer *app) const;
			void send(const MKPackage &msg, const AppLayer *app) const;

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

			/// System ID
			uint8_t sys_id;
			/// List of all links/ interfaces
			interface_packet_list_t interface_list;
			/// Highest file descriptor
			int highest_fd;
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

			///
			void links_to_file_set(fd_set& fds) const;
			void read(const fd_set& fds);
			/// transmit msg to every app in app_list
			void transmit_to_apps(const mavlink_message_t &msg) const;
			/// transmit msg on every cpp_io::IOInterface except src_iface
			void retransmit(const mavlink_message_t &msg, const cpp_io::IOInterface *src_iface) const;
			void retransmit(const MKPackage &msg, const cpp_io::IOInterface *src_iface) const;
			void retransmit_to_apps(const mavlink_message_t &msg, const AppLayer *app) const;
			/// convert MikroKopter packet to MAVLINK packet
			int mk2mavlink(const MKPackage &mk_msg, mavlink_message_t &mav_msg);
	};
	// ----------------------------------------------------------------------------
	// ProtocolStack
	// ----------------------------------------------------------------------------
	inline std::ostream& operator <<(std::ostream &os, const ProtocolStack::packageformat_t &format) {
		os << static_cast<int>(format);

		return os;
	}
	inline std::istream& operator >>(std::istream &is, ProtocolStack::packageformat_t &format) {
		int num_format;
		is >> num_format;
		switch(num_format) {
			case 1:
				format = ProtocolStack::MKPACKAGE;
				break;
			default:
				format = ProtocolStack::MAVLINKPACKAGE;
				break;
		}

		return is;
	}

	inline void ProtocolStack::system_id(const uint8_t system_id) {
		sys_id = system_id;
	}
	inline const uint8_t ProtocolStack::system_id() const {
		return sys_id;
	}
	inline void ProtocolStack::transmit_to_apps(const mavlink_message_t &msg) const {
		std::list<AppLayer*>::const_iterator app_iter;
		for(app_iter = app_list.begin(); app_iter != app_list.end(); ++app_iter) {
			(*app_iter)->handle_input(msg);
		}
	}
	inline void ProtocolStack::retransmit_to_apps(const mavlink_message_t &msg, const AppLayer *app) const {
		std::list<AppLayer*>::const_iterator app_iter;
		for(app_iter = app_list.begin(); app_iter != app_list.end(); ++app_iter) {
			if(*app_iter != app)
				(*app_iter)->handle_input(msg);
		}
	}

} // namespace mavhub

#endif
