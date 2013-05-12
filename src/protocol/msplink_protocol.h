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

#ifndef _MAVHUB_MSPLINK_PROTOCOL_H_
#define _MAVHUB_MSPLINK_PROTOCOL_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MSPLINK_H

#include <inttypes.h> //uint8_t
#include <msplink.h>


namespace mavhub {

template <>
int serialize<msp_message_t>(const msp_message_t &msg, uint8_t *buffer, const uint16_t length);

template <>
int parse_byte<msp_message_t>(const uint8_t input, uint16_t &index, msp_message_t &msg);
// int deserialize<msp_message_t>(const uint8_t channel, const uint8_t *buffer, const uint16_t length, msp_message_t &msg);
// int deserialize<msp_message_t>(const uint8_t *buffer, const uint16_t buf_len, msp_message_t &msg, const uint16_t index);

} // namespace mavhub

#endif // HAVE_MSPLINK_H

#endif
