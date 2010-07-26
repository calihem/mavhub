#ifndef _PROTOCOLLAYER_H_
#define _PROTOCOLLAYER_H_

#include <inttypes.h> //uint8_t
#include <errno.h>
#include "uart.h"
// #include <sys/uio.h>

namespace mavhub {

	class MediaLayer {
		public:
			virtual bool data_available() const = 0;

			virtual int read(const uint8_t *buffer, int length) const = 0;
			virtual int write(const uint8_t *buffer, int length) const = 0;
	};

	class UARTLayer : public UART, public MediaLayer {
		public:
			UARTLayer(const char* devicename, tcflag_t control_modes = B57600 | CS8 | CLOCAL | CREAD) throw(const char*);
			virtual ~UARTLayer();

			virtual bool data_available() const;
			virtual int read(const uint8_t *buffer, int length) const;
			virtual int write(const uint8_t *buffer, int length) const;
	};

	// ----------------------------------------------------------------------------
	// UARTLayer
	// ----------------------------------------------------------------------------
	inline  bool UARTLayer::data_available() const {
		char c;
		int rc = UART::read(&c, 1);
		return ( (rc>=0) || (errno != EAGAIN) );
	}
	inline int UARTLayer::read(const uint8_t *buffer, int length) const {
		return UART::read((void*)buffer, (size_t)length);
	}
	inline int UARTLayer::write(const uint8_t *buffer, int length) const {
		return 0;
	}

} // namespace mavhub

#endif
