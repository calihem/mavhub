#ifndef _PROTOCOLLAYER_H_
#define _PROTOCOLLAYER_H_

#include "core/thread.h"
#include "core/logger.h"
#include "io/io.h"
#include "io/uart.h"
#include "io/network.h"

#include <inttypes.h> //uint8_t
#include <string>
#include <list>
#include <errno.h>
#include <mavlink.h>

namespace mavhub {
	class ProtocolStack;
	class MKPackage;

	// ----------------------------------------------------------------------------
	// Application Layers
	// ----------------------------------------------------------------------------
	class AppLayer : public cpp_pthread::PThread {
		public:
			AppLayer(const std::string& name, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
			virtual ~AppLayer() {};
			friend std::ostream& operator <<(std::ostream &os, const AppLayer &app);

			virtual void handle_input(const mavlink_message_t &msg) = 0;
			int id() const;
			const std::string& name() const;
			const ProtocolStack* owner() const;

		protected:
			friend class ProtocolStack;
			const ProtocolStack *_owner;
			Logger::log_level_t _loglevel;
			std::string _name;

			template <typename T>
			void log(const T& message, const Logger::log_level_t loglevel) const;
			template <typename T1, typename T2>
			void log(const T1& msg1, const T2& msg2, const Logger::log_level_t loglevel) const;
			template <typename T1, typename T2, typename T3>
			inline void log(const T1& msg1, const T2& msg2, const T3& msg3, const Logger::log_level_t loglevel) const;
			void id(const int id);
			void owner(const ProtocolStack *stack);
			virtual void print(std::ostream &os) const;
			virtual void run() = 0;
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
	template <typename T>
	inline void AppLayer::log(const T& message, const Logger::log_level_t loglevel) const {
		Logger::log(message, loglevel, AppLayer::_loglevel);
	}
	template <typename T1, typename T2>
	inline void AppLayer::log(const T1& msg1, const T2& msg2, const Logger::log_level_t loglevel) const {
		Logger::log(msg1, msg2, loglevel, AppLayer::_loglevel);
	}
	template <typename T1, typename T2, typename T3>
	inline void AppLayer::log(const T1& msg1, const T2& msg2, const T3& msg3, const Logger::log_level_t loglevel) const {
		Logger::log(msg1, msg2, msg3, loglevel, AppLayer::_loglevel);
	}
	inline const std::string& AppLayer::name() const {
		return _name;
	}
	inline const ProtocolStack* AppLayer::owner() const {
		return _owner;
	}
	inline void AppLayer::owner(const ProtocolStack *stack) {
		_owner = stack;
	}

	// ----------------------------------------------------------------------------
	// UDPLayer
	// ----------------------------------------------------------------------------
	inline ssize_t UDPLayer::read(void *buf, size_t nbyte) const {
		return UDPSocket::recv_any( static_cast<char*>(buf), nbyte);
	}


} // namespace mavhub

#endif
