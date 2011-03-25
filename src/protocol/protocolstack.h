/****************************************************************************
** Copyright 2011 Humboldt-Universitaet zu Berlin
**
** This file is part of MAVHUB.
**
** MAVHUB is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** MAVHUB is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with MAVHUB.  If not, see <http://www.gnu.org/licenses/>.
**
*****************************************************************************/
/**
 * \file protocolstack.h
 * \date created at 2010/07/26
 * \author Michael Schulz
 *
 * \brief Declaration of protocol stack.
 *
 * \sa protocolstack.cpp
 */

#ifndef _PROTOCOLSTACK_H_
#define _PROTOCOLSTACK_H_

#include "core/thread.h"
#include "core/logger.h"
#include "protocol.h"
// #include "protocollayer.h"
#include "io/io.h"

#include <inttypes.h> //uint8_t
#include <errno.h>
#include <cstring> //strerror
#include <list>
#include <vector>
#include <typeinfo>	//typeid
 
//FIXME: remove iostream
#include <iostream>

namespace mavhub {

typedef std::list<cpp_io::IOInterface*> link_list_t;

template <class T> class AppLayer;

template <class T> class ProtocolStack;

template <class T>
std::ostream& operator <<(std::ostream&os, const ProtocolStack<T> &proto_stack) {
	if( proto_stack.interrupted() ) {
		os << "Protocol stack down" << std::endl;
	}

	os << std::endl << "_Interfaces_" << std::endl;
	os << proto_stack.link_list;

	os << std::endl << "_Applications_" << std::endl;
	os << proto_stack.app_list;

	return os;
}

/**
 * \class ProtocolStack
 * \brief Communication stack to handle different IO interfaces and message types.
 */
template <class T>
class ProtocolStack : public cpp_pthread::PThread {
	public:

		/**
		 * \brief The singleton instance of protocol stack.
		 * \return Reference to protocol stack
		 */
		static ProtocolStack<T>& instance();

		/**
		 * \brief Output stream operator for class ProtocolStack.
		 * \param[out] os The output stream.
		 * \tparam[in] proto_stack The protocol stack which should be streamed to output
		 * stream os.
		 * \return Reference to output stream os.
 		 */
		friend std::ostream& operator<< <>(std::ostream&os, const ProtocolStack<T> &proto_stack);

		/**
		 * \brief Adds an IO interface to the protocol stack. 
		 * \param interface Pointer of the interface to add.
		 * \param format Packet format of data going through the interface. 
		 * \return Error code.
		 * \retval 0 Success.
		 * \retval -1 NULL pointer given.
		 * \retval -2 Reached maximum number of interfaces
		 */
		int add_link(cpp_io::IOInterface *interface);
		/**
		 * \brief Get IO interface for given ID
		 * \param link_id ID of the interface.
		 * \return Pointer to IO interface.
		 */
		cpp_io::IOInterface* link(const unsigned int link_id);
		/**
		 * \brief Remove link with given ID from protocol stack.
		 * \return Error code.
		 * \retval 0 Success.
		 * \retval -1 ID not known
		 */
		int remove_link(const unsigned int link_id);

		/**
		 * \brief Adds given application to protocol stack.
		 *
		 * This changes the ownership of the application, i.e. memory
		 * will be freed by protocol stack.
		 * \param app Pointer to application.
		 * \return Error code.
		 * \retval 0 Success.
		 * \retval -1 NULL pointer given.
		 */
		int add_application(AppLayer<T> *app);
		/**
		 * \brief Get application for given application ID.
		 * \param app_id ID of the application
		 * \return Pointer to application.
		 */
		const AppLayer<T>* application(const unsigned int app_id) const;
		//TODO: int remove_application(const unsigned int app_id);

		/**
		 * \brief Set log level of protocol stack.
		 */
		void loglevel(const Logger::log_level_t loglevel);
		/**
		 * \brief Get log level of protocol stack.
		 */
		const Logger::log_level_t loglevel() const;

		/**
		 * \brief Sends a message over protocol stack.
		 * \param msg The  message to send.
		 * \sa send_over_links(const T &msg, const cpp_io::IOInterface *src_link = NULL) const
		 * \sa send_to_apps(const T &msg, const AppLayer<T> *src_app = NULL) const
		 */
		void send(const T &msg) const;
		/**
		 * \brief Sends a message over all links except source link.
		 * \param msg The  message to send.
		 * \param src_link The sender link.
		 * \sa send(const T &msg) const
		 */
		void send_over_links(const T &msg, const cpp_io::IOInterface *src_link = NULL) const;
		/**
		 * \brief Sends a message over to applications except source application.
		 * \param msg The  message to send.
		 * \param src_app The sender application.
		 * \sa send(const T &msg) const
		 */
		void send_to_apps(const T &msg, const AppLayer<T> *src_app = NULL) const;

	protected:
		/**
		 * \copydoc cpp_pthread::PThread::run()
		 *
		 * The run method polls in an infinite loop over all interfaces
		 * and collects the incoming data.
		 */
		virtual void run();
		
	private:
		/**
		 * \brief Private singleton constructor
		 * \param bufferlength The maximum size for incoming and outgoing data. 
		 * \param loglevel The local log level of the protocol stack.
		 */
		ProtocolStack(const int bufferlength = 512, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
		/// Private destructor 
		~ProtocolStack();
		/**
		 * \brief Copy constructor (intentionally undefined)
		 */
		ProtocolStack(const ProtocolStack&);
		/**
		 * \brief Assignment operator (intentionally undefined)
		 */
		ProtocolStack& operator=(const ProtocolStack&);

		/// Local log level
		Logger::log_level_t _loglevel;
		/// Mutex to protect link/ interface management structures
		mutable pthread_mutex_t link_mutex;
		/// List of all links/ interfaces
		link_list_t link_list;
		/// Highest file descriptor
		int highest_fd;
		/// Receive buffer
		std::vector<uint8_t> rx_buffer;
		/// Bufferlist of incoming messages with index of next byte
		std::list< std::pair<uint16_t,T> > index_msg_list;
		/// Transmit buffer
		mutable std::vector<uint8_t> tx_buffer;
		/// Mutex to protect tx_buffer
		mutable pthread_mutex_t tx_mutex;
		/// List of all registered applications
		std::list< AppLayer<T>* > app_list;

		/**
		 * \brief Read from all interfaces which are marked as ready in fd set.
		 * \param fds Filedescriptor set to check for ready interfaces.
		 */
		void read(const fd_set &fds);
};

/**
 * \brief Fills the filedescriptors of all IO interfaces to the given fd set.
 * \param link_list List of IO interfaces
 * \param fds The filedescriptor set to be filled.
 */
void linklist_to_file_set(const link_list_t &link_list, fd_set &fds);

// ----------------------------------------------------------------------------
// ProtocolStack
// ----------------------------------------------------------------------------
template <class T>
ProtocolStack<T>::ProtocolStack(const int bufferlength, const Logger::log_level_t loglevel) :
	_loglevel(loglevel),
	highest_fd(-1),
	rx_buffer(bufferlength),
	tx_buffer(bufferlength) {

	pthread_mutex_init(&link_mutex, NULL);
	pthread_mutex_init(&tx_mutex, NULL);
}

template <class T>
ProtocolStack<T>::~ProtocolStack() { }

template <class T>
ProtocolStack<T>& ProtocolStack<T>::instance() { 
	// instantiated on first use and will be guaranteed destroyed
	static ProtocolStack<T> instance;

	instance.start();

	return instance;
}

template <class T>
void ProtocolStack<T>::run() {
	if(link_list.size() == 0) return;

	fd_set read_fds;
	int fds_ready;
	timeval timeout;

	Logger::log(typeid(*this).name(), "entered run() with", link_list.size(), "interfaces", Logger::LOGLEVEL_DEBUG, loglevel());

	while( !interrupted() ) {
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;

		linklist_to_file_set(link_list, read_fds);
		fds_ready = select(highest_fd+1, &read_fds, NULL, NULL, &timeout);
		Logger::log(typeid(*this).name(), fds_ready, "link(s) ready to read", Logger::LOGLEVEL_DEBUG, loglevel());
		if(fds_ready < 0) {
			Logger::log("select failed with", strerror(errno), Logger::LOGLEVEL_ERROR, loglevel());
			continue;
		}
		if(fds_ready == 0) continue;

		read(read_fds);
	}

	Logger::log(typeid(*this).name(), "leaving run()", Logger::LOGLEVEL_DEBUG, loglevel());
}

template <class T>
void ProtocolStack<T>::read(const fd_set &fds) {
	int channel = 0;
	int received;
	int rc;

#define continue_link_loop ++msg_iter; channel++; continue;

	typename std::list< std::pair<uint16_t,T> >::iterator msg_iter = index_msg_list.begin(); 
	//iterate through interfaces
	link_list_t::iterator iface_iter;
	for(iface_iter = link_list.begin(); iface_iter != link_list.end(); ++iface_iter) {
		if( !FD_ISSET( (*iface_iter)->handle(), &fds ) ) {
			continue_link_loop;
		}

		received = (*iface_iter)->read( &rx_buffer[0], rx_buffer.size() );
		if(received < 0) {
			if(errno != EAGAIN) {
				Logger::log(typeid(*this).name(), "reading of interface failed with", strerror(errno), Logger::LOGLEVEL_ERROR, loglevel());
			}
			continue_link_loop;
		}
		//catch strange 0 byte
		if(received == 0) {
			continue_link_loop;
		}

		//received data, try to parse it
		Logger::log(typeid(*this).name(), "received", received, "bytes on channel", channel, Logger::LOGLEVEL_DEBUG, loglevel());
		for(uint16_t i=0; i<received; i++) {
			rc = parse_byte( rx_buffer[i], msg_iter->first, msg_iter->second );
			if(rc == 0) {
				Logger::log(typeid(*this).name(), "received packet on", (*iface_iter)->name(), Logger::LOGLEVEL_DEBUG, loglevel());
				send_to_apps(msg_iter->second);
				send_over_links(msg_iter->second, *iface_iter);
			} else if(rc < -1) {
				Logger::log(typeid(*this).name(), "parsing error", rc, "on", (*iface_iter)->name(), Logger::LOGLEVEL_WARN, loglevel() );
			}
		}
		continue_link_loop;
	}

#undef continue_link_loop
}

template <class T>
int ProtocolStack<T>::add_link(cpp_io::IOInterface *interface) {
	if(!interface) return -1;

	// stop execution of stack
	join();

	interface->enable_blocking_mode(false);

	{  //begin of link mutex scope
		cpp_pthread::Lock lm_lock(link_mutex);

		link_list.push_back(interface);
		T msg;
		index_msg_list.push_back( std::make_pair(0,msg) );
	}  //end of link mutex scope

	if(interface->handle() > highest_fd)
		highest_fd = interface->handle();

	Logger::log(typeid(*this).name(), "added link", interface->name(), Logger::LOGLEVEL_DEBUG, loglevel());
	start();

	return 0;
}

template <class T>
cpp_io::IOInterface* ProtocolStack<T>::link(unsigned int link_id) {
	if(link_list.size() >= link_id+1) { //ID is in range
		link_list_t::iterator iface_iter = link_list.begin();
		// seek iterator to right position
		for(unsigned int i = 0; i < link_id; i++) iface_iter++;
		return *iface_iter;
	}
	return NULL;
}

template <class T>
int ProtocolStack<T>::remove_link(const unsigned int link_id) {
	int rc = -1;

	if( !link(link_id) ) {
		return rc;
	}

	// stop execution of stack
	join();

	{  //begin of link mutex scope
		cpp_pthread::Lock lm_lock(link_mutex);

		link_list_t::iterator iface_iter = link_list.begin();
		// seek iterator to right position
		for(unsigned int i = 0; i < link_id; i++) iface_iter++;
		//TODO: remove msg buffer from msg_list
		link_list.erase(iface_iter);
		//TODO: get new highest_fd
		rc = 0;
	}  //end of link mutex scope

	if(link_list.size() > 0)
		start();

	return rc;
}

template <class T>
int ProtocolStack<T>::add_application(AppLayer<T> *app) {
	if(!app) return -1;

	app->owner(this);
	app_list.push_back(app);
	Logger::log(typeid(*this).name(), "added application", app->name(), Logger::LOGLEVEL_DEBUG, loglevel());

	return 0;
}

template <class T>
const AppLayer<T>* ProtocolStack<T>::application(const unsigned int app_id) const {
	if(app_list.size() >= app_id+1) { //ID is in range
		typename std::list< AppLayer<T>* >::const_iterator app_iter = app_list.begin();
		// seek iterator to right position
		for(unsigned int i = 0; i < app_id; i++) app_iter++;
		return *app_iter;
	}
	return NULL;
}

template <class T>
inline void ProtocolStack<T>::loglevel(const Logger::log_level_t loglevel) {
	_loglevel = loglevel;
}
template <class T>
inline const Logger::log_level_t ProtocolStack<T>::loglevel() const {
	return _loglevel;
}

template <class T>
void ProtocolStack<T>::send(const T &msg) const {
	send_to_apps(msg);
	send_over_links(msg);
}

template <class T>
void ProtocolStack<T>::send_over_links(const T &msg, const cpp_io::IOInterface *src_link) const {
	cpp_pthread::Lock tx_lock(tx_mutex);

	uint16_t length = serialize( msg, &tx_buffer[0], tx_buffer.size() );

	link_list_t::const_iterator iface_iter;
	for(iface_iter = link_list.begin(); iface_iter != link_list.end(); ++iface_iter) {
		if(*iface_iter != src_link) {
			(*iface_iter)->write(&tx_buffer[0], length);
			Logger::log(typeid(*this).name(), "sent", length, "bytes over interface", (*iface_iter)->name(), Logger::LOGLEVEL_DEBUG, loglevel());
		}
	}
}

template <class T>
void ProtocolStack<T>::send_to_apps(const T &msg, const AppLayer<T> *src_app) const {
	typename std::list< AppLayer<T>* >::const_iterator app_iter;
	for(app_iter = app_list.begin(); app_iter != app_list.end(); ++app_iter) {
		if(*app_iter != src_app)
			(*app_iter)->handle_input(msg);
	}
}

} // namespace mavhub

#endif
