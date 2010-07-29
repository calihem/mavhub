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
			virtual bool data_available() const = 0;

			virtual int read(uint8_t *buffer, int length) const = 0;
			virtual int write(const uint8_t *buffer, int length) const = 0;
	};

	class UARTLayer : public UART, public MediaLayer {
		public:
			UARTLayer(const char* devicename, tcflag_t control_modes = B57600 | CS8 | CLOCAL | CREAD) throw(const char*);
			virtual ~UARTLayer();

			virtual bool data_available() const;
			virtual int read(uint8_t *buffer, int length) const;
			virtual int write(const uint8_t *buffer, int length) const;
	};

	class UDPLayer : public UDPSocket, public MediaLayer {
		public:
			static const int DefaultPort = 32000;

			UDPLayer(int port) throw(const char*);
			virtual ~UDPLayer();

			virtual bool data_available() const;
			virtual int read(uint8_t *buffer, int length) const;
			virtual int write(const uint8_t *buffer, int length) const;
			void add_groupmember(const std::string& addr, uint16_t port) throw(const char*);

		private:
			/// list of groupmembers with numeric ip addr and port
			std::list<num_addr_pair_t> groupmember_list;
	};

	// ----------------------------------------------------------------------------
	// UARTLayer
	// ----------------------------------------------------------------------------
	inline  bool UARTLayer::data_available() const {
		char c;
		int rc = UART::read(&c, 1);
		return ( (rc>=0) || (errno != EAGAIN) );
	}
	inline int UARTLayer::read(uint8_t *buffer, int length) const {
		return UART::read((void*)buffer, (size_t)length);
	}
	inline int UARTLayer::write(const uint8_t *buffer, int length) const {
		//TODO
		return 0;
	}
	// ----------------------------------------------------------------------------
	// UDPLayer
	// ----------------------------------------------------------------------------
	inline  bool UDPLayer::data_available() const {
		//TODO
		return false;
	}
	inline int UDPLayer::read(uint8_t *buffer, int length) const {
		return UDPSocket::receive( (char*)buffer, length);
	}
	inline int UDPLayer::write(const uint8_t *buffer, int length) const {
		//TODO
		return 0;
	}


} // namespace mavhub

#endif
