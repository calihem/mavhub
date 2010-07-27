#include "network.h"

#include "logger.h"
#include <cstring> //memset
#include <fcntl.h>

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

void Socket::enable_blocking_mode(bool enabled) {
	int mode = fcntl(sockfd, F_GETFL, 0);
	
	if(enabled) {
		mode &= ~O_NONBLOCK;
	} else {
		mode |= O_NONBLOCK;
	}
	if( fcntl(sockfd, F_SETFL, mode) != 0 ) {
		Logger::log("setting of blocking mode for socket failed", Logger::LOGLEVEL_ERROR);
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
	close(sockfd);
}

int UDPSocket::receive(char *buffer, int buflen) const {
	int so_len = sizeof(si_other);

	return recvfrom(sockfd,			//socket
		buffer,				//buffer
		buflen,				//length
		0,				//flags
		(struct sockaddr*)&si_other,	//address
		(socklen_t*)&so_len		//address_len
		);
}

void UDPSocket::bind(int port) throw(const char*) {
	//assign server port
	si_self.sin_port = htons(port);

	if( ::bind(sockfd, (struct sockaddr*)&si_self, sizeof(si_self)) < 0) {
		throw "Binding of socket failed";
	}
}

void UDPSocket::sendTo(const void *buffer, int bufferLength, const char *foreignIP, int foreignPort) const throw(const char*) {

	//assign IP
	if( inet_aton(foreignIP, &si_other.sin_addr) == 0) {
		throw "Assignment of IP Address failed";
	}
	//assign port
	si_other.sin_port = htons(foreignPort);

	//sendto(int socket,
	//	const void *message,
	//	size_t length,
	//	int flags,
	//	const struct sockaddr *dest_addr,
	//	socklen_t dest_len);
	if( sendto(sockfd,
		buffer,
		bufferLength,
		0,
		(struct sockaddr*)&si_other,
		sizeof(si_other)
		) < 0 ) {
		
		throw "Send failed";
	}
}


} //namespace mavhub
