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
 * \file protocolstack.cpp
 * \date created at 2010/07/26
 * \author Michael Schulz
 *
 * \brief Implementation of protocol stack.
 *
 * \sa protocolstack.h
 */

#include "protocolstack.h"

namespace mavhub {

void linklist_to_file_set(const link_list_t &link_list, fd_set &fds) {
	FD_ZERO(&fds);
	// add file descriptors to set
	link_list_t::const_iterator iface_iter;
	for(iface_iter = link_list.begin(); iface_iter != link_list.end(); ++iface_iter) {
		FD_SET( (*iface_iter)->handle(), &fds );
	}
}

} // namespace mavhub
