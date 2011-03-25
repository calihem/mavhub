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
 * \file mklink_protocol.h
 * \date created at 2011/04/29
 * \author Michael Schulz
 *
 * \brief TODO.
 *
 * \sa mklink_protocol.cpp
 */

#ifndef _MAVHUB_MLINK_PROTOCOL_H_
#define _MAVHUB_MKLINK_PROTOCOL_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MKLINK_H

#include <inttypes.h> //uint8_t
#include <mklink.h>

namespace mavhub {

template <>
int serialize<mk_message_t>(const mk_message_t &msg, uint8_t *buffer, const uint16_t length);

template <>
int parse_byte<mk_message_t>(const uint8_t input, uint16_t &index, mk_message_t &msg);

// template <>
// int deserialize<mk_message_t>(const uint8_t channel, const uint8_t *buffer, const uint16_t length, mk_message_t &msg);
// int deserialize<mk_message_t>(const uint8_t *buffer, const uint16_t buf_len, mk_message_t &msg, const uint16_t index);

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>
mavlink_manual_control_t* copy(mavlink_manual_control_t *destination, const mk_extern_control_t *source);
mavlink_manual_control_t* copy(mavlink_manual_control_t *destination, const mk_extern_control_t *source);
#endif // HAVE_MAVLINK_H


} // namespace mavhub

#endif // HAVE_MKLINK_H

#endif
