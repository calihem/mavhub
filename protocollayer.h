#ifndef _PROTOCOLLAYER_H_
#define _PROTOCOLLAYER_H_

#include <inttypes.h> //uint8_t
#include <list>
#include <errno.h>
#include <mavlink.h>
#include "thread.h"
#include "lib/io.h"
#include "uart.h"
#include "network.h"

namespace mavhub {
	class ProtocolStack;
	class MKPackage;

	// ----------------------------------------------------------------------------
	// Application Layers
	// ----------------------------------------------------------------------------
	class AppLayer : public cpp_pthread::PThread {
		public:
			AppLayer();
			virtual ~AppLayer() {};
			virtual void handle_input(const mavlink_message_t &msg) = 0;

		protected:
			friend class ProtocolStack;
			const ProtocolStack *owner;

			virtual void run() = 0;
			void set_owner(const ProtocolStack *stack);
			void send(const mavlink_message_t &msg) const;
			void send(const MKPackage &msg) const;
	};

	class UDPLayer : public UDPSocket {
		public:
			static const int DefaultPort = 32000;

			UDPLayer(int port) throw(const char*);
			virtual ~UDPLayer();

			virtual ssize_t read(void *buf, size_t nbyte) const;
			/**
			 * @brief send data of buffer to every group member
			 * @param buf data to send
			 * @param nbyte length of buffer
			 * @return number of bytes actually sent
			 */
			virtual ssize_t write(const void *buf, size_t nbyte) const;
			void add_groupmember(const std::string& addr, uint16_t port) throw(const char*);
			void add_groupmembers(const std::list<string_addr_pair_t>& member_list) throw(const char*);

		protected:
			virtual void print(std::ostream &os) const;

		private:
			/// list of groupmembers with numeric ip addr and port
			std::list<num_addr_pair_t> groupmember_list;
	};

	// ----------------------------------------------------------------------------
	// AppLayer
	// ----------------------------------------------------------------------------
	inline void AppLayer::set_owner(const ProtocolStack *stack) {
		owner = stack;
	}

	// ----------------------------------------------------------------------------
	// UDPLayer
	// ----------------------------------------------------------------------------
	inline ssize_t UDPLayer::read(void *buf, size_t nbyte) const {
		return UDPSocket::recv_any( static_cast<char*>(buf), nbyte);
	}


} // namespace mavhub

#endif
