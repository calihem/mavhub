#include "network.h"

#include "utility.h"

#include <cstring> //memset
#include <netdb.h> //hostent
#include <iostream>
#include <sstream>
#include <cerrno>

using std::string;

namespace mavhub {

std::ostream& operator <<(std::ostream &os, const string_addr_pair_t &string_addr_pair) {
	os << string_addr_pair.first << ":" << string_addr_pair.second;

	return os;
}

std::istream& operator >>(std::istream &is, string_addr_pair_t &string_addr_pair) {
	char tmp;

	//eat whitespaces
	is >> tmp;
	while(tmp == ' ' || tmp == '\t')
		is >> tmp;

	//read address
	uint8_t counter = 1;
	string_addr_pair.first.clear();
	while(tmp != ':' && tmp != ' ' && counter < 16) {
		string_addr_pair.first.push_back(tmp);
		is >> tmp;
		counter++;
	}
	//read port
	is >> string_addr_pair.second;

	return is;
}

// ----------------------------------------------------------------------------
// Socket
// ----------------------------------------------------------------------------
Socket::Socket(int type, int protocol) :
		IOInterface("", proto_to_name(protocol)) {

	open(type, protocol);

	//configure own socket address
	bzero(&si_self, sizeof(si_self));	//clear struct
	si_self.sin_family = AF_INET;		//set address family

	//configure foreign socket address
	bzero(&si_other, sizeof(si_other));	//clear struct
	si_other.sin_family = AF_INET;	//set address family
}

Socket::Socket(int socket_fd) :
		IOInterface("", "") { //FIXME

	fd = socket_fd;

	//configure own socket address
	bzero(&si_self, sizeof(si_self));	//clear struct
	si_self.sin_family = AF_INET;		//set address family

	//configure foreign socket address
	bzero(&si_other, sizeof(si_other));	//clear struct
	si_other.sin_family = AF_INET;	//set address family
}

Socket::~Socket() {
}

int Socket::open(const int type, const int protocol) {
	if(is_open()) return fd;

	//socket(int domain, int type, int protocol)
	fd = socket(PF_INET, type, protocol);
	return fd;
}

std::string Socket::foreign_addr() const throw(const std::exception&) {
	struct sockaddr_storage addr;
	socklen_t len = sizeof(addr);
	char ip_str[INET6_ADDRSTRLEN];

	if(getpeername(fd, (sockaddr*)&addr, &len) < 0) {
		throw std::runtime_error("Fetching of address info about the remote side of the connection failed");
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

uint16_t Socket::foreign_port() const throw(const std::exception&) {
	union sockaddr_u addr;
	socklen_t len = sizeof(addr.sockaddr);
	uint16_t port = 0;
	
	if(getpeername(fd, &addr.sockaddr, &len) < 0) {
		throw std::runtime_error("Fetching of address info about the remote side of the connection failed");
	} else if(addr.storage.ss_family == AF_INET) { //IPv4
		port = ntohs(addr.in.sin_port);
	} else { //IPv6
		port = ntohs(addr.in6.sin6_port);
	}

	return port;
}

const std::string& Socket::proto_to_name(const int type) {
	static const string protocol_names[3] = {
		"Socket",
		"UDP Port",
		"TCP Port"
	};

	switch(type) {
		case IPPROTO_UDP:
			return protocol_names[1];
		case IPPROTO_TCP:
			return protocol_names[2];
	}
	
	return protocol_names[0];
}

void Socket::bind(int port) throw(const std::exception&) {
	//assign server port
	si_self.sin_port = htons(port);

	if( ::bind(fd, (struct sockaddr*)&si_self, sizeof(si_self)) < 0) {
		throw std::runtime_error("Binding of socket failed");
	}
}

// ----------------------------------------------------------------------------
// UDPSocket
// ----------------------------------------------------------------------------
UDPSocket::UDPSocket(int port) throw(const std::exception&) :
		Socket(SOCK_DGRAM, IPPROTO_UDP) {

	//configure own socket address
	si_self.sin_addr.s_addr = htonl(INADDR_ANY);	//accept any IP address
	try{
		bind(port);
		std::stringstream outstream;
		outstream << port;
		_name = outstream.str();
	}
	catch(const std::exception& e) {
		throw e;
	}
}

UDPSocket::~UDPSocket() {
}

int UDPSocket::open() {
	return Socket::open(SOCK_DGRAM, IPPROTO_UDP);
}

int UDPSocket::recv_any(void *buffer, int buf_len) const {
	int so_len = sizeof(si_other);

	return recvfrom(fd,			//socket
		buffer,				//buffer
		buf_len,			//length
		0,				//flags
		(struct sockaddr*)&si_other,	//address
		(socklen_t*)&so_len		//address_len
		);
}

int UDPSocket::recv_from(void *buffer, int buf_len, std::string &source_addr, uint16_t source_port) const throw(const std::exception&) {
	sockaddr_in si_source;
	socklen_t source_len = sizeof(si_source);
	int rc;
	
	rc = recvfrom(fd,
		      buffer,
		      buf_len,
		      0,
		      (sockaddr*)&si_source,
		      (socklen_t*)&source_len
		      );

	if(rc < 0) {
		throw std::runtime_error("UDPSocket::recv_from failed");
	}
	
	source_addr = inet_ntoa(si_source.sin_addr);
	source_port = ntohs(si_source.sin_port);

	return rc;
}

int UDPSocket::send_to(const void *buffer, int buf_len, const string &foreign_addr, uint16_t foreign_port) const throw(const std::exception&) {
	in_addr num_foreign_addr;
	int rc;

	//convert string to numeric ip and assign it
	if( inet_aton(foreign_addr.c_str(), &num_foreign_addr) == 0) {
		throw std::runtime_error("Assignment of IP Address failed");
	}
	try{
		rc = send_to(buffer, buf_len, num_foreign_addr, foreign_port);
	}
	catch(const std::exception& e) {
		throw e;
	}
	
	return rc;
}

int UDPSocket::send_to(const void *buffer, int buf_len, in_addr foreign_addr, uint16_t foreign_port) const throw(const std::exception&) {
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
	int rc =  sendto(fd,
		buffer,
		buf_len,
		0,
		(struct sockaddr*)&si_other,
		sizeof(si_other)
		);
	if(rc < 0 ) {
		std::stringstream error_msg;
		error_msg << "Send failed with error " << strerror(errno);
		throw std::runtime_error( error_msg.str() );
	}

	return rc;
}

// ----------------------------------------------------------------------------
// TCPSocket
// ----------------------------------------------------------------------------
TCPSocket::TCPSocket() : Socket(SOCK_STREAM, IPPROTO_TCP) {}

TCPSocket::TCPSocket(const std::string &foreign_addr, uint16_t foreign_port) :
		Socket(SOCK_STREAM, IPPROTO_TCP) {

	try {
		connect(foreign_addr, foreign_port);
	}
	catch(const std::exception& e) {
		throw e;
	}
}

TCPSocket::TCPSocket(int socket_fd) :
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
	
	if(::connect(fd, (sockaddr*)&si_other, sizeof(si_other)) < 0) {
		//TODO: throw exception
	}
}

void TCPSocket::disconnect() {
	sockaddr_in si_null;
	bzero(&si_null, sizeof(si_null));	//clear struct
	si_null.sin_family = AF_UNSPEC; 
	
	if(::connect(fd, (sockaddr*)&si_null, sizeof(si_null)) < 0) {
		//TODO: throw exception
	}
}

void TCPSocket::send(const void *buffer, int buf_len) {
	if(::send(fd, buffer, buf_len, 0) < 0) {
		//TODO: throw exception
	}
}

int TCPSocket::receive(void *buffer, int buf_len) const {
	return recv(fd,			//socket
		buffer,				//buffer
		buf_len,			//length
		0
		);
}

// ----------------------------------------------------------------------------
// TCPServerSocket
// ----------------------------------------------------------------------------
TCPServerSocket::TCPServerSocket(uint16_t port, int connections) throw(const std::exception&) :
		Socket(SOCK_STREAM, IPPROTO_TCP) {

	//configure own socket address
	si_self.sin_addr.s_addr = htonl(INADDR_ANY);	//accept any IP address

	try {
		bind(port);
	}
	catch(const std::exception& e) {
		throw e;
	}
	
	if(::listen(fd, connections) < 0) {
		throw std::runtime_error("Listen failed");
	}
}

TCPServerSocket::~TCPServerSocket() {
}

TCPSocket* TCPServerSocket::accept() throw(const std::exception&) {
	int socket_fd;
	if( (socket_fd = ::accept(fd, NULL, 0)) < 0 ) {
		throw std::runtime_error("Accept failed");
	}

	return new TCPSocket(socket_fd);
}

} //namespace mavhub
