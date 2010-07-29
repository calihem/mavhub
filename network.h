#ifndef _NETWORK_H_
#define _NETWORK_H_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <map>
#include <string>


namespace mavhub {
	typedef std::pair<std::string, uint16_t> string_addr_pair_t;
	typedef std::pair<in_addr, uint16_t> num_addr_pair_t;
	
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

			/**
			 * @brief receive data from any system
			 * @param buffer data to receive
			 * @param buf_len maximal length of data to receive
			 */
			int receive(char *buffer, int buflen) const;
			/**
			 * @brief receive data from given system
			 * @param buffer data to receive
			 * @param buf_len maximal length of data to receive
			 * @param source_addr address of source system as a dots-and-number string
			 * @param source_port port number of source system in host byte order
			 */
			int recv_from(void *buffer, int buf_len, std::string &source_addr, uint16_t source_port) const throw(const char*);
			/**
			 * @brief send data to given system
			 * @param buffer data to send
			 * @param buf_len length of data to send
			 * @param foreign_addr address of destination system as a dots-and-number string
			 * @param foreign_port port number of destination system in host byte order
			 */
			void send_to(const void *buffer, int buf_len, const std::string &foreign_addr, uint16_t foreign_port) const throw(const char*);
			/**
			 * @brief send data to given system
			 * @param buffer data to send
			 * @param buf_len length of data to send
			 * @param foreign_addr numeric address of destination system
			 * @param foreign_port port number of destination system in host byte order
			 */
			void send_to(const void *buffer, int buf_len, in_addr foreign_addr, uint16_t foreign_port) const throw(const char*);

		private:
			void bind(int port) throw(const char*);
			//TODO: delete sendTo
			void sendTo(const void *buffer, int bufferLength, const char *foreignIP, int foreignPort) const throw(const char*);
	};

	// ----------------------------------------------------------------------------
	// Socket
	// ----------------------------------------------------------------------------
} // namespace mavhub

#endif
