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
 * \file mavlink_protocol.h
 * \date created at 2011/03/25
 * \author Michael Schulz
 *
 * \brief TODO.
 *
 * \sa mavlink_protocol.cpp
 */

#ifndef _MAVHUB_MAVLINK_PROTOCOL_H_
#define _MAVHUB_MAVLINK_PROTOCOL_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H

//TODO #define MAVLINK_COMM_NUM_BUFFERS 1

#include <inttypes.h> //uint8_t
#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif
#include <mavlink.h>

namespace mavhub {

template <>
int serialize<mavlink_message_t>(const mavlink_message_t &msg, uint8_t *buffer, const uint16_t length);

template <>
int parse_byte<mavlink_message_t>(const uint8_t input, uint16_t &index, mavlink_message_t &msg);

/**
 * \brief Input stream operator for MAV types.
 * \param[in,out] is The input stream.
 * \tparam[out] type The MAV type which should hold the value from
 * input stream is.
 * \return Reference to input stream is.
 */
std::istream& operator >>(std::istream &is, enum MAV_TYPE &type);

/**
 * \brief Input stream operator for MAV types.
 * \param[in,out] is The input stream.
 * \tparam[out] type The autopilot type which should hold the value from
 * input stream is.
 * \return Reference to input stream is.
 */
std::istream& operator >>(std::istream &is, enum MAV_AUTOPILOT &type);

} // namespace mavhub

#endif // HAVE_MAVLINK_H

#endif
