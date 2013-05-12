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
 * \file protocol.h
 * \date created at 2011/03/25
 * \author Michael Schulz
 *
 * \brief TODO.
 */

#ifndef _MAVHUB_PROTOCOL_H_
#define _MAVHUB_PROTOCOL_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#include <inttypes.h> //uint8_t
#include <ostream>

namespace mavhub {

/// Enumeration of supported protocols
enum protocol_type_t {
#ifdef HAVE_MAVLINK_H
	MAVLINK,
#endif // HAVE_MAVLINK_H
#ifdef HAVE_MKHUCHLINK_H
	MKHUCHLINK,
#endif // HAVE_MKHUCHLINK_H
#ifdef HAVE_MKLINK_H
	MKLINK,
#endif // HAVE_MKLINK_H
#ifdef HAVE_MSPLINK_H
        MSPLINK, // MultiWii Serial Protocol
#endif // HAVE_MSPLINK_H
	UnknownProtocol
};
std::ostream& operator <<(std::ostream &os, const protocol_type_t &protocol_type);
std::istream& operator >>(std::istream &is, protocol_type_t &protocol_type);

//TODO: add docu of serialize
template <typename T>
int serialize(const T &msg, uint8_t *buffer, const uint16_t length);

//FIXME: check docu of parse_byte
/**
 * \brief Transforms (part of) byte stream into message.
 * \param buffer part of byte stream
 * \param buf_len The number of bytes to read from buffer. 
 * \param msg The message to store the decoded byte stream.
 * \param index Start index of buffer in byte stream counting from beginning of message.
 * \return Index of next byte in stream. Negative Index means an error occured during parsing.
 * \retval 0 Succesfully parsed message.
 * \retval -1 No start sign found in stream.
 * \retval -2 Hash mismatch
 * \retval -3 Received data length out of range.
 */
template <typename T>
int parse_byte(const uint8_t input, uint16_t &index, T &msg);

} // namespace mavhub

#include "mavlink_protocol.h"
#include "mkhuchlink_protocol.h"
#include "mklink_protocol.h"
#include "msplink_protocol.h" // MultiWii Serial Protocol

#endif
