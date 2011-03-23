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

#include "thread.h"
#include <inttypes.h> //uint8_t
#include "protocollayer.h"
#include <list>
#include <vector>
#include <ostream>

namespace mavhub {
class MKPackage;

/**
 * \class ProtocolStack
 * \brief Communication stack to handle different IO interfaces and applications.
 * \todo Make ProtocolStack more generic in handling different package formats.
 */
class ProtocolStack : public cpp_pthread::PThread {
	public:
		/// Enumeration of supported packet formats
		enum packageformat_t {
			MAVLINKPACKAGE, ///< MAVLINK packet
			MKPACKAGE       ///< MikroKopter packet
		};
		/**
		 * \brief Output stream operator for enumeration type packageformat_t.
		 * \param[out] os The output stream.
		 * \tparam[in] format The package format which should be streamed to output
		 * stream os.
		 * \return Reference to output stream os.
 		 */
		friend std::ostream& operator <<(std::ostream&os, const packageformat_t &format);
		/**
		 * \brief Input stream operator for enumeration type packageformat_t.
		 * \param[in,out] is The input stream.
		 * \param[out] format The package format which should hold the value from
		 * input stream is.
		 * \return Reference to input stream is.
		 */
		friend std::istream& operator >>(std::istream&is, packageformat_t &format);

		/// Pair of IO interface and packet format
		typedef std::pair<cpp_io::IOInterface*, packageformat_t> interface_packet_pair_t;
		/// List of IO interfaces with corresponding packet format
		typedef std::list<interface_packet_pair_t> interface_packet_list_t;
		/// List of buffers
		typedef std::list<std::vector<uint8_t> > buffer_list_t;
		/// Pair of Application ID and Application
		typedef std::pair<uint16_t, AppLayer*> id_app_pair_t;

		/**
		 * \brief The singleton instance of protocol stack.
		 * \return Reference to protocol stack
		 */
		static ProtocolStack& instance();
		/**
		 * \brief Output stream operator for class ProtocolStack.
		 * \param[out] os The output stream.
		 * \tparam[in] proto_stack The protocol stack which should be streamed to output
		 * stream os.
		 * \return Reference to output stream os.
 		 */
		friend std::ostream& operator <<(std::ostream&os, const ProtocolStack &proto_stack);

		/**
		 * \brief Set the system ID.
		 * \param system_id The new system ID.
		 */
		void system_id(const uint8_t system_id);
		/**
		 * Get the current system ID.
		 * \return Current system ID.
		 */
		const uint8_t system_id() const;

		/**
		 * \brief Adds an IO interface to the protocol stack. 
		 * \param interface Pointer of the interface to add.
		 * \param format Packet format of data going through the interface. 
		 * \return Error code.
		 * \retval 0 Success.
		 * \retval -1 NULL pointer given.
		 * \retval -2 Reached maximum number of interfaces
		 */
		int add_link(cpp_io::IOInterface *interface, const packageformat_t format);
		/**
		 * \brief Get IO interface for given ID
		 * \param link_id ID of the interface.
		 * \return Pointer to IO interface.
		 */
		cpp_io::IOInterface* link(const unsigned int link_id);
		/**
		 * \brief Get a list of all IO interfaces.
		 * \return List of pointers to IO interfaces
		 */
		const std::list<cpp_io::IOInterface*> io_list() const;
		/**
		 * \brief Get a list of all IO interfaces with corresponding packet format.
		 * \return List of interface and packet format pairs.
		 */
		const interface_packet_list_t& link_list() const;
		/**
		 * \brief Remove link with given ID from protocol stack.
		 * \return Error code.
		 * \retval 0 Success.
		 * \retval -1 ID not known
		 */
		int remove_link(unsigned int link_id);

		/**
		 * \brief Adds given application to protocol stack.
		 *
		 * This changes the ownership of the application, i.e. memory
		 * will be freed by protocol stack.
		 * \param app Pointer to application.
		 */
		void add_application(AppLayer *app);
		/**
		 * \brief Get List of all applications.
		 * \return List of pointers to application.
		 */
		const std::list<AppLayer*>& application_list() const;
		/**
		 * \brief Get application for given application ID.
		 * \param app_id ID of the application
		 * \return Pointer to application.
		 */
		const AppLayer* application(const unsigned int app_id) const;
		/**
		 * \brief Sends a MAVLink message over protocol stack.
		 * \param msg The MAVLink message to send.
		 * \param app The sender application.
		 * \sa send(const MKPackage &msg, const AppLayer *app = NULL) const
		 */
		void send(const mavlink_message_t &msg, const AppLayer *app = NULL) const;
		/**
		 * \brief Sends a MikroKopter message over protocol stack.
		 * \param msg The MikroKopter message to send.
		 * \param app The sender application.
		 * \sa send(const mavlink_message_t &msg, const AppLayer *app = NULL) const
		 */
		void send(const MKPackage &msg, const AppLayer *app = NULL) const;

	protected:
		/**
		 * \copydoc cpp_pthread::PThread::run()
		 *
		 * The run method polls in an infinite loop over all interfaces
		 * and collects the incoming data.
		 */
		virtual void run();


	private:
		/// Private singleton constructor
		ProtocolStack(uint8_t system_id = -1);
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

		/**
		 * \brief Size of RX and TX buffers
		 *
		 * The BUFFERLENGTH should be the maximum packet length
		 * of all used packet formats. Otherwise data will be
		 * lost at receiving or sending.
		 */
		static const int BUFFERLENGTH = 512;

		/// True if run() is in polling loop
		bool loop_forever;
		/// System ID
		uint8_t sys_id;
		/// List of all links/ interfaces
		interface_packet_list_t interface_list;
		/// Highest file descriptor
		int highest_fd;
		/// Mutex to protect link/ interface management structures
		mutable pthread_mutex_t link_mutex;
		/// List of all registered applications
		std::list<AppLayer*> app_list;
		/// List of receive buffers
		buffer_list_t rx_buffer_list;
		/// Transmit buffer
		mutable uint8_t tx_buffer[BUFFERLENGTH];
		/// Mutex to protect tx_buffer
		mutable pthread_mutex_t tx_mutex;

		/**
		 * \brief Fills the filedescriptors of all IO interfaces to the given fd set.
		 * \param fds The filedescriptor set to be filled.
		 */
		void links_to_file_set(fd_set &fds) const;
		/**
		 * \brief Read from all interfaces which are marked as ready in fd set.
		 * \param fds Filedescriptor set to check for ready interfaces.
		 */
		void read(const fd_set &fds);
		/**
		 * \brief Transmits MAVLink message to every application in app_list.
		 * \param msg The MAVLink message to transmit.
		 */
		void transmit_to_apps(const mavlink_message_t &msg) const;
		/**
		 * \brief Retransmits MAVLink message over every IO interface except the source interface.
		 * \param msg The MAVLink message to transmit.
		 * \param src_iface The IO interface from which msg came.
		 */
		void retransmit(const mavlink_message_t &msg, const cpp_io::IOInterface *src_iface) const;
		/**
		 * \brief Retransmits MikroKopter message over every IO interface except the source interface.
		 * \param msg The MikroKopter message to transmit.
		 * \param src_iface The IO interface from which msg came.
		 * \todo Improve MKPackage support.
		 */
		void retransmit(const MKPackage &msg, const cpp_io::IOInterface *src_iface) const;
		/**
		 * \brief Retransmits MAVLink message to every application except the source application.
		 * \param msg The MAVLink message to transmit.
		 * \param src_app The application from which msg came.
		 */
		void retransmit_to_apps(const mavlink_message_t &msg, const AppLayer *src_app) const;
		/**
		 * \brief Converts MikroKopter packet to MAVLINK packet
		 * \todo Improve MKPackage support.
		 * \todo Remove mk2mavlink from ProtocolStack.
		 */
		int mk2mavlink(const MKPackage &mk_msg, mavlink_message_t &mav_msg);
};

/**
 * \brief Output stream operator for interface packet list.
 * \param[out] os The output stream.
 * \tparam[in] ifp_list The List of interface packet pairs
 * which should be streamed to output stream os.
 * \return Reference to output stream os.
 */
std::ostream& operator <<(std::ostream&os, const ProtocolStack::interface_packet_list_t &ifp_list);
/**
 * \brief Output stream operator for application list.
 * \param[out] os The output stream.
 * \tparam[in] app_list The List of applications which should be streamed to
 * output stream os.
 * \return Reference to output stream os.
 */
std::ostream& operator <<(std::ostream&os, const std::list<AppLayer*> &app_list);

// ----------------------------------------------------------------------------
// ProtocolStack
// ----------------------------------------------------------------------------
inline std::ostream& operator <<(std::ostream &os, const ProtocolStack::packageformat_t &format) {
	os << static_cast<int>(format);

	return os;
}
inline std::istream& operator >>(std::istream &is, ProtocolStack::packageformat_t &format) {
	int num_format;
	is >> num_format;
	switch(num_format) {
		case 1:
			format = ProtocolStack::MKPACKAGE;
			break;
		default:
			format = ProtocolStack::MAVLINKPACKAGE;
			break;
	}

	return is;
}

inline const std::list<AppLayer*>& ProtocolStack::application_list() const {
	return app_list;
}
inline const ProtocolStack::interface_packet_list_t& ProtocolStack::link_list() const {
	return interface_list;
}
inline void ProtocolStack::system_id(const uint8_t system_id) {
	sys_id = system_id;
}
inline const uint8_t ProtocolStack::system_id() const {
	return sys_id;
}
inline void ProtocolStack::transmit_to_apps(const mavlink_message_t &msg) const {
	std::list<AppLayer*>::const_iterator app_iter;
	for(app_iter = app_list.begin(); app_iter != app_list.end(); ++app_iter) {
		(*app_iter)->handle_input(msg);
	}
}
inline void ProtocolStack::retransmit_to_apps(const mavlink_message_t &msg, const AppLayer *src_app) const {
	std::list<AppLayer*>::const_iterator app_iter;
	for(app_iter = app_list.begin(); app_iter != app_list.end(); ++app_iter) {
		if(*app_iter != src_app)
			(*app_iter)->handle_input(msg);
	}
}

} // namespace mavhub

#endif
