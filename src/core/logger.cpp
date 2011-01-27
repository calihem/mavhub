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
 * \file logger.cpp
 * \date created at 2010/07/26
 * \author Michael Schulz
 *
 * \brief Implementation of Logger Class.
 *
 * \sa logger.h
 */

#include "logger.h"

#if defined(FILELOG)
#include <fstream>
#else
#include <iostream>
#endif

namespace mavhub {

const char* Logger::LoglevelStrings[LoglevelNum] = {
	"generic",
	"debug",
	"info",
	"warning",
	"error",
	"fatal",
	"off"
};

std::ostream& operator <<(std::ostream &os, const Logger::log_level_t &level) {
	os << Logger::LoglevelStrings[static_cast<int>(level)];

	return os;
}

std::istream& operator >>(std::istream &is, Logger::log_level_t &level) {
	std::string level_string;
	is >> level_string;
	if(level_string.empty()) return is;

	switch(level_string.at(0)) {
		case 'e':
			level = Logger::LOGLEVEL_ERROR;
			break;
		case 'f':
			level = Logger::LOGLEVEL_FATAL;
			break;
		case 'd':
			level = Logger::LOGLEVEL_DEBUG;
			break;
		case 'g':
			level = Logger::LOGLEVEL_ALL;
			break;
		case 'i':
			level = Logger::LOGLEVEL_INFO;
			break;
		case 'o':
			level = Logger::LOGLEVEL_OFF;
			break;
		case 'w':
			level = Logger::LOGLEVEL_WARN;
			break;
		default:
			level = Logger::LOGLEVEL_ALL;
			break;
	}

	return is;
}


#if defined(_REENTRANT)
pthread_mutex_t Logger::stream_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

#if defined(STDOUTLOG) //log to stdout
Logger::log_level_t Logger::log_level = LOGLEVEL_WARN;
std::ostream* Logger::out_stream = &std::cout;

#elif defined(STDERRLOG) //log to stderr
Logger::log_level_t Logger::log_level = LOGLEVEL_WARN;
std::ostream* Logger::out_stream = &std::cerr;

#elif defined(FILELOG) //log to file
Logger::log_level_t Logger::log_level = LOGLEVEL_WARN;
//create new file, destruction is done by auto_ptr
std::auto_ptr<std::ostream> Logger::out_stream_auto_ptr
	= std::auto_ptr<std::ostream>( new std::ofstream(FILELOG, std::ios_base::app) );
//set out_stream to created file
std::ostream* Logger::out_stream = out_stream_auto_ptr.get();

#else //disable logging
Logger::log_level_t Logger::log_level = LOGLEVEL_OFF;
std::ostream* Logger::out_stream = NULL;
#endif

} // namespace mavhub
