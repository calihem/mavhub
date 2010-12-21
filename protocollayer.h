#ifndef _PROTOCOLLAYER_H_
#define _PROTOCOLLAYER_H_

#include <inttypes.h> //uint8_t
#include <list>
#include <errno.h>
#include <mavlink.h>
#include "thread.h"
#include "logger.h"
#include "lib/io.h"
#include "uart.h"
#include "network.h"
#include <string>

namespace mavhub {
	class ProtocolStack;
	class MKPackage;

	// ----------------------------------------------------------------------------
	// Application Layers
	// ----------------------------------------------------------------------------
	class AppLayer : public cpp_pthread::PThread {
		public:
			AppLayer(const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
			virtual ~AppLayer() {};
			virtual void handle_input(const mavlink_message_t &msg) = 0;
			inline int get_app_id() const;
			inline std::string get_app_name();

		protected:
			friend class ProtocolStack;
			const ProtocolStack *owner;
			Logger::log_level_t loglevel;
			// proposal
			int app_id;
			std::string app_name;

			virtual void run() = 0;
			void set_owner(const ProtocolStack *stack);
			void send(const mavlink_message_t &msg) const;
			void send(const MKPackage &msg) const;
			template <typename T>
			void log(const T& message, const Logger::log_level_t loglevel) const;
			template <typename T1, typename T2>
			void log(const T1& msg1, const T2& msg2, const Logger::log_level_t loglevel) const;
			template <typename T1, typename T2, typename T3>
			inline void log(const T1& msg1, const T2& msg2, const T3& msg3, const Logger::log_level_t loglevel) const;
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
	template <typename T>
	inline void AppLayer::log(const T& message, const Logger::log_level_t loglevel) const {
		Logger::log(message, loglevel, AppLayer::loglevel);
	}
	template <typename T1, typename T2>
	inline void AppLayer::log(const T1& msg1, const T2& msg2, const Logger::log_level_t loglevel) const {
		Logger::log(msg1, msg2, loglevel, AppLayer::loglevel);
	}
	template <typename T1, typename T2, typename T3>
	inline void AppLayer::log(const T1& msg1, const T2& msg2, const T3& msg3, const Logger::log_level_t loglevel) const {
		Logger::log(msg1, msg2, msg3, loglevel, AppLayer::loglevel);
	}
	inline int AppLayer::get_app_id() const {
		return app_id;
	}
	inline std::string AppLayer::get_app_name() {
		return app_name;
	}

	// ----------------------------------------------------------------------------
	// UDPLayer
	// ----------------------------------------------------------------------------
	inline ssize_t UDPLayer::read(void *buf, size_t nbyte) const {
		return UDPSocket::recv_any( static_cast<char*>(buf), nbyte);
	}


} // namespace mavhub

#endif
