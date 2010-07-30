#ifndef _PROTOCOLLAYER_H_
#define _PROTOCOLLAYER_H_

#include <inttypes.h> //uint8_t
#include <list>
#include <errno.h>
#include "uart.h"
#include "network.h"

namespace mavhub {

	class MediaLayer {
		public:
			virtual int read(uint8_t *buffer, int length) const = 0;
			virtual int write(const uint8_t *buffer, int length) const = 0;
	};

	class UARTLayer : public UART, public MediaLayer {
		public:
			UARTLayer(const char* devicename, tcflag_t control_modes = B57600 | CS8 | CLOCAL | CREAD) throw(const char*);
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
		return UDPSocket::receive( (char*)buffer, length);
	}


} // namespace mavhub

#endif
