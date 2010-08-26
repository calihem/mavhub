#ifndef _PROTOCOLLAYER_H_
#define _PROTOCOLLAYER_H_

#include <inttypes.h> //uint8_t
#include <list>
#include <errno.h>
#include "thread.h"
#include "uart.h"
#include "network.h"

namespace mavhub {
	class ProtocolStack;

	// ----------------------------------------------------------------------------
	// Application Layers
	// ----------------------------------------------------------------------------
	class AppLayer : public PThread {
		public:
			AppLayer();
			virtual ~AppLayer() {};
			void set_owner(const ProtocolStack *stack);
			virtual void handle_input(const mavlink_message_t &msg) = 0;

		protected:
			const ProtocolStack *owner;
			virtual void run() = 0;
	};

	// ----------------------------------------------------------------------------
	// Media Layers
	// ----------------------------------------------------------------------------
	class MediaLayer {
		public:
			virtual int read(uint8_t *buffer, int length) const = 0;
			virtual int write(const uint8_t *buffer, int length) const = 0;
			/// Get the human readable name of this device
			virtual const std::string& name() const;
			/// Get the system name of this device
			virtual const std::string& system_name() const;
			
		protected:
			/// Human readable name of this device, e.g. "Serial Link"
			std::string dev_name;
			/// Name of the device in the system context, e.g. "/dev/ttyS0"
			std::string sys_name;
	};

	class UARTLayer : public UART, public MediaLayer {
		public:
			UARTLayer(const std::string& devicename, tcflag_t control_modes = B57600 | CS8 | CLOCAL | CREAD) throw(const char*);
			virtual ~UARTLayer();

			virtual int read(uint8_t *buffer, int length) const;
			virtual int write(const uint8_t *buffer, int length) const;
	};

	class UDPLayer : public UDPSocket, public MediaLayer {
		public:
			static const int DefaultPort = 32000;

			UDPLayer(int port) throw(const char*);
			virtual ~UDPLayer();

			virtual int read(uint8_t *buffer, int length) const;
			/**
			 * @brief send data of buffer to every group member
			 * @param buffer data to send
			 * @param length length of buffer
			 * @return number of bytes actually sent
			 */
			virtual int write(const uint8_t *buffer, int length) const;
			void add_groupmember(const std::string& addr, uint16_t port) throw(const char*);

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
	// MediaLayer
	// ----------------------------------------------------------------------------
	inline const std::string& MediaLayer::name() const {
		return dev_name;
	}
	inline const std::string& MediaLayer::system_name() const {
		return sys_name;
	}

	// ----------------------------------------------------------------------------
	// UARTLayer
	// ----------------------------------------------------------------------------
	inline int UARTLayer::read(uint8_t *buffer, int length) const {
		return UART::read((void*)buffer, (size_t)length);
	}
	inline int UARTLayer::write(const uint8_t *buffer, int length) const {
		return UART::write(buffer, length);
	}
	// ----------------------------------------------------------------------------
	// UDPLayer
	// ----------------------------------------------------------------------------
	inline int UDPLayer::read(uint8_t *buffer, int length) const {
		return UDPSocket::recv_any( (char*)buffer, length);
	}


} // namespace mavhub

#endif
