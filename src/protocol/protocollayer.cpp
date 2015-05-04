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
 * \file protocollayer.cpp
 * \date created at 2010/07/26
 * \author Michael Schulz
 *
 * \brief Implementation of protocol layers.
 *
 * \sa protocollayer.h
 */

#include "protocollayer.h"
#include "protocolstack.h"

#include <sstream>

using namespace hub;

namespace mavhub {

#ifdef HAVE_MAVLINK_H
MavlinkAppLayer::MavlinkAppLayer(const std::string &name, const Logger::log_level_t loglevel) :
	AppInterface(name, loglevel),
	AppLayer<mavlink_message_t>(name, loglevel),
	component_id(0) {

	pthread_mutex_init(&tx_mav_mutex, NULL);
}

void MavlinkAppLayer::request_data_stream(const unsigned int target_system,
	const unsigned int target_component,
	const unsigned int stream_id,
	const unsigned int rate) const {

	Lock tx_lock(tx_mav_mutex);
	mavlink_msg_request_data_stream_pack(system_id(),
		component_id,
		&tx_mav_msg,
		target_system,
		target_component,
		stream_id,
		rate,
		rate > 0 ? 1 : 0);
	send(tx_mav_msg);
}

#endif

// ----------------------------------------------------------------------------
// UDPLayer
// ----------------------------------------------------------------------------
UDPLayer::UDPLayer(int port) throw(const std::exception&) :
	UDPSocket(port) {

	enable_blocking_mode(false);
}

UDPLayer::~UDPLayer() { }

void UDPLayer::add_groupmember(const std::string &addr, uint16_t port) throw(const std::exception&) {
	in_addr num_addr;

	if(inet_aton(addr.c_str(), &num_addr) == 0) {
		throw std::runtime_error("Assignment of IP Address failed");
	}

	groupmember_list.push_back( std::make_pair(num_addr, port) );
}

void UDPLayer::add_groupmembers(const std::list<string_addr_pair_t> &member_list) throw(const std::exception&) {
	if( member_list.empty() ) return;

	std::list<string_addr_pair_t>::const_iterator it;
	for(it = member_list.begin(); it != member_list.end(); ++it) {
          Logger::log("UDPLayer group members", it->first, it->second, Logger::LOGLEVEL_DEBUG);
		try {
			add_groupmember(it->first, it->second);
		}
		catch(const std::exception& e) {
			throw e;
		}
	}

}

ssize_t UDPLayer::write(const void *buf, size_t nbyte) const {
	std::list<num_addr_pair_t>::const_iterator gmember_iter;
	int rc = 0;

	for(gmember_iter = groupmember_list.begin(); gmember_iter != groupmember_list.end(); ++gmember_iter) {
		try{
			rc = send_to(buf, nbyte, gmember_iter->first, gmember_iter->second);
		}
		catch(const std::exception& e) {
			Logger::log(e.what(), Logger::LOGLEVEL_ERROR);
		}
	}

	return rc;
}

void UDPLayer::print(std::ostream &os) const {
	IOInterface::print(os);

	std::list<num_addr_pair_t>::const_iterator gmember_iter;
	os << "* Group members:" << std::endl;
	for(gmember_iter = groupmember_list.begin(); gmember_iter != groupmember_list.end(); ++gmember_iter) {
		os << "\t" << inet_ntoa(gmember_iter->first) << " " << gmember_iter->second << std::endl;
	}
}

} // namespace mavhub
