#include "network.h"

#include "logger.h"
#include "utility.h"
#include <cstring> //memset

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

void Socket::enable_blocking_mode(bool enabled) {
	if( ::mavhub::enable_blocking_mode(sockfd,enabled) != 0 ) {
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


} //namespace mavhub
