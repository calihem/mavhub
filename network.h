#ifndef _NETWORK_H_
#define _NETWORK_H_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <map>
#include <string>


namespace mavhub {
	typedef std::pair<std::string, uint16_t> address_pair_t;
	
// 	typedef std::pair<const char*,int> address_pair;
// 	typedef std::map<int, address_pair> address_map;

	class Socket {
		public:
			void enable_blocking_mode(bool enabled);

		protected:
			Socket(int type, int protocol) throw(const char*);

			int sockfd;
			struct sockaddr_in si_self;
			mutable struct sockaddr_in si_other;
			
		private:
			Socket(const Socket &socket);
			void operator=(const Socket &socket);
	};

	class UDPSocket : public Socket {
		public:
			UDPSocket(int port) throw(const char*);
			virtual ~UDPSocket();

			int receive(char *buffer, int buflen) const;

			int recv_from(void *buffer, int buf_len, std::string &source_addr, uint16_t source_port) const throw(const char*);
			void send_to(const void *buffer, int buf_len, const std::string &foreign_addr, uint16_t foreign_port) const throw(const char*);

		private:
			void bind(int port) throw(const char*);
			void sendTo(const void *buffer, int bufferLength, const char *foreignIP, int foreignPort) const throw(const char*);
	};

	// ----------------------------------------------------------------------------
	// Socket
	// ----------------------------------------------------------------------------
} // namespace mavhub

#endif
