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
 * \file protocollayer.h
 * \date created at 2010/07/26
 * \author Michael Schulz
 *
 * \brief Declaration of protocol layers.
 *
 * \sa protocollayer.cpp
 */

#ifndef _PROTOCOLLAYER_H_
#define _PROTOCOLLAYER_H_

#include "core/core.h"
#include "core/thread.h"
#include "core/logger.h"
#include "io/io.h"
#include "io/uart.h"
#include "io/network.h"
#include "protocol/protocolstack.h"

#include <inttypes.h> //uint8_t
#include <string>
#include <list>
#include <errno.h>
#include <mavlink.h>

namespace mavhub {

/**
 * \class AppInterface
 * \brief Interface class for applications.
 */
class AppInterface : public cpp_pthread::PThread {
	public:
		/**
		 * \brief AppInterface constructor with application name
		 * and optional log level.
		 * \param name Human readable application name.
		 * \param loglevel The local log level of the application.
		 */
		AppInterface(const std::string &name, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);

		/**
		 * \brief AppInterface destructor.
		 */
		virtual ~AppInterface() { };

		/**
		 * \brief Output stream operator for class AppLayer.
		 * \param[out] os The output stream.
		 * \tparam[in] app The application which should be streamed to output
		 * stream os.
		 * \return Reference to output stream os.
 		 */
		friend std::ostream& operator <<(std::ostream &os, const AppInterface &app);

		/**
		 * \brief Get the application name
		 * \return Reference to application name.
		 */
		const std::string& name() const;

		/**
		 * \brief Get the Owner
		 * \return Pointer to the owning instance (protocol stack).
		 */
		const void* owner() const;

		/**
		 * \brief Sets the owner
		 * \param owner The protocol stack owning the instance of
		 * the application layer
		 */
		void owner(const void *owner);

		/**
		 * \brief Get the system ID
		 * \return System ID
		 */
		static const uint16_t system_id();

	protected:
		const void *_owner;			///< pointer to owner
		Logger::log_level_t _loglevel;	///< local log level of application
		std::string _name;		///< human readable name of application

		/**
		 * \brief Writes message to logger.
		 *
		 * This method is only a shortcut to the corresponding log function
		 * of class Logger.
		 * \tparam message The message which will be written to logger.
		 * \param loglevel The log level of the message.
		 * \sa Logger::log
		 */
		template<typename T1>
		void log(const T1 &message, const Logger::log_level_t loglevel) const;

		/**
		 * \brief Writes two message to logger.
		 *
		 * This method is only a shortcut to the corresponding log function
		 * of class Logger.
		 * \tparam msg1 First message which will be written to logger.
		 * \tparam msg2 Second message which will be written to logger.
		 * \param loglevel The log level of the message.
		 * \sa Logger::log
		 */
		template<typename T1, typename T2>
		void log(const T1 &msg1, const T2 &msg2, const Logger::log_level_t loglevel) const;

		/**
		 * \brief Writes three message to logger.
		 *
		 * This method is only a shortcut to the corresponding log function
		 * of class Logger.
		 * \tparam msg1 First message which will be written to logger.
		 * \tparam msg2 Second message which will be written to logger.
		 * \tparam msg3 Third message which will be written to logger.
		 * \param loglevel The log level of the message.
		 * \sa Logger::log
		 */
		template<typename T1, typename T2, typename T3>
		inline void log(const T1 &msg1, const T2 &msg2, const T3 &msg3, const Logger::log_level_t loglevel) const;

		/**
		 * \brief Writes an application summary to output stream.
		 * \param os Output stream to which summary should be written.
		 */
		virtual void print(std::ostream &os) const;

		/**
		 * \copydoc PThread::run
		 */
		virtual void run() = 0;
};

/**
 * \class Application
 * \brief Base class for applications.
 */
template <class T>
class AppLayer : virtual public AppInterface {
	public:
		/**
		 * \brief AppLayer constructor with application name
		 * and optional log level.
		 * \param name Human readable application name.
		 * \param loglevel The local log level of the application.
		 */
		AppLayer(const std::string &name, const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);

		/**
		 * \brief AppLayer destructor.
		 */
		virtual ~AppLayer() { };

		/**
		 * \brief Handle new mavlink messages.
		 * \param msg The mavlink message to process.
		 */
		virtual void handle_input(const T &msg) = 0;

	protected:
		template <class TT> friend class ProtocolStack;

		/**
		 * \brief Main application loop.
		 * \sa PThread::run
		 */
		virtual void run() = 0;

		/**
		 * \brief Sends a message over protocol stack.
		 * \param msg The message to send.
		 */
		void send(const T &msg) const;
};

/**
 * \class UDPLayer
 * \brief UDP socket class which sends datagrams to a predefined group.
 */
class UDPLayer : public UDPSocket {
	public:
		/**
		 * \brief UDPLayer constructor.
		 * \param port The UDP port.
		 * \throws std::exception exception with description message
		 */
		UDPLayer(int port) throw(const std::exception&);

		/**
		 * UDPLayer destructor.
		 */
		virtual ~UDPLayer();

		/**
		 * \brief Reads data from UDP socket to buffer
		 * \param buf Pointer to receiving buffer. The buffer
		 * should have a size of at least nbyte.
		 * \param nbyte Maximum number of bytes read from socket
		 * \return Number of bytes actually read.
		 */
		virtual ssize_t read(void *buf, size_t nbyte) const;

		/**
		 * \brief Sends data of buffer to every group member
		 * \param buf Data to send
		 * \param nbyte Length of buffer
		 * \return number of bytes actually sent
		 */
		virtual ssize_t write(const void *buf, size_t nbyte) const;

		/**
		 * \brief Adds an IP-port-pair to group.
		 * \param addr IP address of new group member
		 * \param port UDP port of new group member
		 * \throws std::exception exception with description message
		 */
		void add_groupmember(const std::string &addr, uint16_t port) throw(const std::exception&);

		/**
		 * \brief Adds a list of IP-port-pairs to group.
		 * \param member_list List of new group members
		 * \throws std::exception exception with description message
		 */
		void add_groupmembers(const std::list<string_addr_pair_t> &member_list) throw(const std::exception&);

	protected:
		/**
		 * \brief Writes a summary of UDPLayer to output stream.
		 * \param os Output stream to which summary should be written.
		 */
		virtual void print(std::ostream &os) const;

	private:
		/// list of group members with numeric IP address and port
		std::list<num_addr_pair_t> groupmember_list;
};
// ----------------------------------------------------------------------------
// AppInterface
// ----------------------------------------------------------------------------
inline AppInterface::AppInterface(const std::string &name, const Logger::log_level_t loglevel) :
	_owner(NULL),
	_loglevel(loglevel),
	_name(name) { }

inline std::ostream& operator <<(std::ostream &os, const AppInterface &app) {
	app.print(os);
	return os;
}

template<typename T1>
inline void AppInterface::log(const T1 &message, const Logger::log_level_t loglevel) const {
	Logger::log(message, loglevel, AppInterface::_loglevel);
}
template<typename T1, typename T2>
inline void AppInterface::log(const T1 &msg1, const T2 &msg2, const Logger::log_level_t loglevel) const {
	Logger::log(msg1, msg2, loglevel, AppInterface::_loglevel);
}
template<typename T1, typename T2, typename T3>
inline void AppInterface::log(const T1 &msg1, const T2 &msg2, const T3 &msg3, const Logger::log_level_t loglevel) const {
	Logger::log(msg1, msg2, msg3, loglevel, AppInterface::_loglevel);
}

inline const std::string& AppInterface::name() const {
	return _name;
}

inline const void* AppInterface::owner() const {
	return _owner;
}

inline void AppInterface::owner(const void *owner) {
	_owner = owner;
}

inline void AppInterface::print(std::ostream &os) const {
	os << name() << ":" << std::endl
		<< "* loglevel: " << _loglevel << std::endl;
}

inline const uint16_t AppInterface::system_id() {
	return Core::system_id();
}

// ----------------------------------------------------------------------------
// AppLayer
// ----------------------------------------------------------------------------
template <class T>
AppLayer<T>::AppLayer(const std::string &name, const Logger::log_level_t loglevel) :
	AppInterface(name, loglevel) { }

template <class T>
inline void AppLayer<T>::send(const T &msg) const {
	ProtocolStack<T>::instance().send_to_apps(msg, this);
	ProtocolStack<T>::instance().send_over_links(msg);
}


// ----------------------------------------------------------------------------
// UDPLayer
// ----------------------------------------------------------------------------
inline ssize_t UDPLayer::read(void *buf, size_t nbyte) const {
	return UDPSocket::recv_any( static_cast<char*>(buf), nbyte);
}

} // namespace mavhub

#endif
