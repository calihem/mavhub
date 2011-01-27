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
 * \file mavshell.h
 * \date created at 2010/08/24
 * \author Michael Schulz
 *
 * \brief Declaration of MAVShell.
 *
 * \sa mavshell.cpp
 */

#ifndef _MAVSHELL_H_
#define _MAVSHELL_H_

#include "core/thread.h"
#include "io/network.h"

#include <inttypes.h>   //uint8_t
#include <sstream>      //stringstream
#include <vector>

namespace mavhub {

/**
 * \class MAVShell
 * \brief Class to handle a TCP connection to configure mavhub.
 */
class MAVShell : public cpp_pthread::PThread {
	public:
		/**
		 * \brief MAVShell constructor with optional TCP port.
		 * \param port TCP communication port.
		 * \throws const char* error description message
		 */
		MAVShell(uint16_t port = 32000) throw(const char*);

		/**
		 * \brief MAVShell destructor.
		 */
		~MAVShell();

	protected:
		/**
		 * \brief The main loop waiting for new connections.
		 */
		virtual void run();


	private:
		static const int BUFFERLENGTH = 512;	///< size of buffer

		/**
		 * \brief Copy constructor (intentionally undefined)
		 */
		MAVShell(const MAVShell&);

		/**
		 * \brief Assignment operator (intentionally undefined)
		 */
		MAVShell& operator=(const MAVShell&);

		TCPServerSocket server_socket;	///< Socket for incoming connections.
		TCPSocket *client_socket;	///< Socket to remote connection. 
		std::stringstream input_stream;	///< Stream of received data.

		/**
		 * \brief Handles incoming data from #client_socket.
		 */
		void handle_client();

		/**
		 * \brief Sends a string message to #client_socket.
		 * \param msg Message to send.
		 */
		void send_message(const std::string &msg);

		/**
		 * \brief Parses the input stream and calls MAVShell::execute_cmd.
		 * \param stream The stream received from #client_socket.
		 */
		void tokenize_stream(std::stringstream &stream);

		/**
		 * \brief Executes the commands given as an argument vector.
		 * \param argv The argument vector to execute.
		 */
		void execute_cmd(const std::vector<std::string> &argv);

		/**
		 * \brief Sends a help message to #client_socket of supported commands.
		 */
		void send_help();

		/**
		 * \brief Sends a welcome message to #client_socket.
		 */
		void welcome();
};
// ----------------------------------------------------------------------------
// MAVShell
// ----------------------------------------------------------------------------


} // namespace mavhub

#endif
