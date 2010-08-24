#include "network.h"

#include "logger.h"
#include "utility.h"
#include <cstring> //memset
#include <netdb.h> //hostent

using std::string;

namespace mavhub {

// ----------------------------------------------------------------------------
// Socket
// ----------------------------------------------------------------------------
Socket::Socket(int type, int protocol) throw(const char*) {
	//socket(int domain, int type, int protocol)
	if( (sockfd = socket(PF_INET, type, protocol)) < 0 ) {
		throw "Creation of socket failed";
	}

	//configure own socket address
	bzero(&si_self, sizeof(si_self));	//clear struct
	si_self.sin_family = AF_INET;		//set address family

	//configure foreign socket address
	bzero(&si_other, sizeof(si_other));	//clear struct
	si_other.sin_family = AF_INET;	//set address family
}

Socket::Socket(int socket_fd) throw(const char*) :
		sockfd(socket_fd) {

	//configure own socket address
	bzero(&si_self, sizeof(si_self));	//clear struct
	si_self.sin_family = AF_INET;		//set address family

	//configure foreign socket address
	bzero(&si_other, sizeof(si_other));	//clear struct
	si_other.sin_family = AF_INET;	//set address family

}

Socket::~Socket() {
	close(sockfd);
}

void Socket::enable_blocking_mode(bool enabled) {
	if( ::mavhub::enable_blocking_mode(sockfd,enabled) != 0 ) {
		Logger::log("setting of blocking mode for socket failed", Logger::LOGLEVEL_ERROR);
	}
}

std::string Socket::foreign_addr() const throw(const char*) {
	struct sockaddr_storage addr;
	socklen_t len = sizeof(addr);
	char ip_str[INET6_ADDRSTRLEN];

	if(getpeername(sockfd, (sockaddr*)&addr, &len) < 0) {
		throw "Fetching of address info about the remote side of the connection failed";
		return string();
	} else if(addr.ss_family == AF_INET) { //IPv4
		struct sockaddr_in *sa_in = (struct sockaddr_in *)&addr;
		inet_ntop(AF_INET, &sa_in->sin_addr, ip_str, sizeof(ip_str));
	} else { //IPv6
		struct sockaddr_in6 *sa_in = (struct sockaddr_in6 *)&addr;
		inet_ntop(AF_INET6, &sa_in->sin6_addr, ip_str, sizeof(ip_str));
	}

	return string(ip_str);
}

uint16_t Socket::foreign_port() const throw(const char*) {
	struct sockaddr_storage addr;
	socklen_t len = sizeof(addr);
	uint16_t port = 0;
	
	if(getpeername(sockfd, (sockaddr*)&addr, &len) < 0) {
		throw "Fetching of address info about the remote side of the connection failed";
	} else if(addr.ss_family == AF_INET) { //IPv4
		struct sockaddr_in *sa_in = (struct sockaddr_in *)&addr;
		port = ntohs(sa_in->sin_port);
	} else { //IPv6
		struct sockaddr_in6 *sa_in = (struct sockaddr_in6 *)&addr;
		port = ntohs(sa_in->sin6_port);
	}

	return port;
}

void Socket::bind(int port) throw(const char*) {
	//assign server port
	si_self.sin_port = htons(port);

	if( ::bind(sockfd, (struct sockaddr*)&si_self, sizeof(si_self)) < 0) {
		throw "Binding of socket failed";
	}
}

// ----------------------------------------------------------------------------
// UDPSocket
// ----------------------------------------------------------------------------
UDPSocket::UDPSocket(int port) throw(const char*) :
		Socket(SOCK_DGRAM, IPPROTO_UDP) {

	//configure own socket address
	si_self.sin_addr.s_addr = htonl(INADDR_ANY);	//accept any IP address
	try{
		bind(port);
	}
	catch(const char *message) {
		throw message;
	}
}

UDPSocket::~UDPSocket() {
}

int UDPSocket::recv_any(void *buffer, int buf_len) const {
	int so_len = sizeof(si_other);

	return recvfrom(sockfd,			//socket
		buffer,				//buffer
		buf_len,			//length
		0,				//flags
		(struct sockaddr*)&si_other,	//address
		(socklen_t*)&so_len		//address_len
		);
}

int UDPSocket::recv_from(void *buffer, int buf_len, std::string &source_addr, uint16_t source_port) const throw(const char*) {
	sockaddr_in si_source;
	socklen_t source_len = sizeof(si_source);
	int rc;
	
	rc = recvfrom(sockfd,
		      buffer,
		      buf_len,
		      0,
		      (sockaddr*)&si_source,
		      (socklen_t*)&source_len
		      );

	if(rc < 0) {
		throw("UDPSocket::recv_from failed");
	}
	
	source_addr = inet_ntoa(si_source.sin_addr);
	source_port = ntohs(si_source.sin_port);

	return rc;
}

int UDPSocket::send_to(const void *buffer, int buf_len, const string &foreign_addr, uint16_t foreign_port) const throw(const char*) {
	in_addr num_foreign_addr;
	int rc;

	//convert string to numeric ip and assign it
	if( inet_aton(foreign_addr.c_str(), &num_foreign_addr) == 0) {
		throw "Assignment of IP Address failed";
	}
	try{
		rc = send_to(buffer, buf_len, num_foreign_addr, foreign_port);
	}
	catch(const char *message) {
		throw(message);
	}
	
	return rc;
}

int UDPSocket::send_to(const void *buffer, int buf_len, in_addr foreign_addr, uint16_t foreign_port) const throw(const char*) {
	//assign address
	si_other.sin_addr = foreign_addr;
	//assign port in network byte order
	si_other.sin_port = htons(foreign_port);

	//sendto(int socket,
	//	const void *message,
	//	size_t length,
	//	int flags,
	//	const struct sockaddr *dest_addr,
	//	socklen_t dest_len);
	int rc =  sendto(sockfd,
		buffer,
		buf_len,
		0,
		(struct sockaddr*)&si_other,
		sizeof(si_other)
		);
	if(rc < 0 ) {
		throw "Send failed";
	}

	return rc;
}

// ----------------------------------------------------------------------------
// TCPSocket
// ----------------------------------------------------------------------------
TCPSocket::TCPSocket() throw(const char*) :
		Socket(SOCK_STREAM, IPPROTO_TCP) {
}

TCPSocket::TCPSocket(const std::string &foreign_addr, uint16_t foreign_port) throw(const char*) :
		Socket(SOCK_STREAM, IPPROTO_TCP) {
			
	try {
		connect(foreign_addr, foreign_port);
	}
	catch(const char *message) {
		throw message;
	}
}

TCPSocket::TCPSocket(int socket_fd) throw(const char*) :
		Socket(socket_fd) {
}

TCPSocket::~TCPSocket() {
}

void TCPSocket::connect(const string &foreign_addr, uint16_t foreign_port) {

	// resolve name
	hostent *foreign_host;
	if( (foreign_host = gethostbyname(foreign_addr.c_str())) == NULL ) {
		//TODO: throw exception
	}
	
	si_other.sin_addr.s_addr = *((uint32_t*)foreign_host->h_addr_list[0]);
	si_other.sin_port = htons(foreign_port);
	
	if(::connect(sockfd, (sockaddr*)&si_other, sizeof(si_other)) < 0) {
		//TODO: throw exception
	}
}

void TCPSocket::disconnect() {
	sockaddr_in si_null;
	bzero(&si_null, sizeof(si_null));	//clear struct
	si_null.sin_family = AF_UNSPEC; 
	
	if(::connect(sockfd, (sockaddr*)&si_null, sizeof(si_null)) < 0) {
		//TODO: throw exception
	}
}

void TCPSocket::send(const void *buffer, int buf_len) {
	if(::send(sockfd, buffer, buf_len, 0) < 0) {
		//TODO: throw exception
	}
}

int TCPSocket::receive(void *buffer, int buf_len) const {
	return recv(sockfd,			//socket
		buffer,				//buffer
		buf_len,			//length
		0
		);
}

// ----------------------------------------------------------------------------
// TCPServerSocket
// ----------------------------------------------------------------------------
TCPServerSocket::TCPServerSocket(uint16_t port, int connections) throw(const char*) :
		Socket(SOCK_STREAM, IPPROTO_TCP) {

	//configure own socket address
	si_self.sin_addr.s_addr = htonl(INADDR_ANY);	//accept any IP address

	try {
		bind(port);
	}
	catch(const char *message) {
		throw message;
	}
	
	if(::listen(sockfd, connections) < 0) {
		throw "Listen failed";
	}
}

TCPServerSocket::~TCPServerSocket() {
}

TCPSocket* TCPServerSocket::accept() throw(const char*) {
	int socket_fd;
	if( (socket_fd = ::accept(sockfd, NULL, 0)) < 0 ) {
		throw "Accept failed";
	}

	return new TCPSocket(socket_fd);
}

} //namespace mavhub
