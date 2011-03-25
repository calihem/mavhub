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
 * \file logger.h
 * \date created at 2010/07/26
 * \author Michael Schulz
 *
 * \brief Declaration of Logger Class.
 *
 * \sa logger.cpp
 */

#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <string>
#include <ostream>
#include <iomanip>
#include <memory>
#include <time.h>
#if defined(_REENTRANT)
#include <pthread.h>
#endif

#if !defined(STDOUTLOG) && !defined(STDERRLOG) && !defined(FILELOG)
#define DISABLELOGGER
#endif

namespace mavhub {

/**
 * \class Logger
 * \brief Leightweight logging class.
 */
class Logger {
	public:
		/// Enumeration of logging levels
		enum log_level_t {
			LOGLEVEL_ALL,   ///< Log everything
			LOGLEVEL_DEBUG, ///< Log debugging informations
			LOGLEVEL_INFO,  ///< Log general informations
			LOGLEVEL_WARN,  ///< Log warnings
			LOGLEVEL_ERROR, ///< Log errors
			LOGLEVEL_FATAL, ///< Log critical errors
			LOGLEVEL_OFF    ///< Log nothing
		};

		/**
		 * \brief Output stream operator for enumeration type log_level_t.
		 * \param[out] os The output stream.
		 * \tparam[in] level The log level which should be streamed to output
		 * stream os.
		 * \return Reference to output stream os.
 		 */
		friend std::ostream& operator <<(std::ostream &os, const log_level_t &level);
		/**
		 * \brief Input stream operator for enumeration type log_level_t.
		 * \param[in,out] is The input stream.
		 * \param[out] level The log level which should hold the value from
		 * input stream is.
		 * \return Reference to input stream is.
		 */
		friend std::istream& operator >>(std::istream &is, log_level_t &level);

		/**
		 * Returns true if global logging level is higher than LOG_OFF.
		 */
		static bool enabled();

		/**
		 * \brief Set global logging level.
		 * \param loglevel The new log level.
		 * \todo: rename setLogLevel to loglevel
		 */
		static void setLogLevel(log_level_t loglevel);

		/**
		 * Get global logging level.
		 * \return Current log level.
		 */
		static log_level_t loglevel();

		/**
		 * \brief Writes message to logger.
		 * \tparam message The message which will be written to logger
		 * by output stream operator.
		 * \param msg_loglevel The log level for msg.
		 * \param local_loglevel The log level of the current context 
		 * (application, module, etc.).
		 */
		template<typename T>
		static void log(const T &message,
				const log_level_t msg_loglevel = LOGLEVEL_ALL,
				const log_level_t local_loglevel = LOGLEVEL_ALL);

		/**
		 * \brief Writes two messages to logger
		 * \tparam msg1 First message which will be written to logger
		 * by output stream operator.
		 * \tparam msg2 Second message which will be written to logger
		 * by output stream operator.
		 * \param msg_loglevel The log level for both messages.
		 * \param local_loglevel The log level of the current context 
		 * (application, module, etc.).
		 */
		template<typename T1, typename T2>
		static void log(const T1 &msg1,
				const T2 &msg2,
				const log_level_t msg_loglevel = LOGLEVEL_ALL,
				const log_level_t local_loglevel = LOGLEVEL_ALL);
		/**
		 * \brief Writes three messages to logger
		 * \tparam msg1 First message which will be written to logger
		 * by output stream operator.
		 * \tparam msg2 Second message which will be written to logger
		 * by output stream operator.
		 * \tparam msg3 Third message which will be written to logger
		 * by output stream operator.
		 * \param msg_loglevel The log level for given messages.
		 * \param local_loglevel The log level of the current context 
		 * (application, module, etc.).
		 */
		template<typename T1, typename T2, typename T3>
		static void log(const T1 &msg1,
				const T2 &msg2,
				const T3 &msg3,
				const log_level_t msg_loglevel = LOGLEVEL_ALL,
				const log_level_t local_loglevel = LOGLEVEL_ALL);
		/**
		 * \brief Writes four messages to logger
		 * \tparam msg1 First message which will be written to logger
		 * by output stream operator.
		 * \tparam msg2 Second message which will be written to logger
		 * by output stream operator.
		 * \tparam msg3 Third message which will be written to logger
		 * by output stream operator.
		 * \tparam msg4 Fourth message which will be written to logger
		 * by output stream operator.
		 * \param msg_loglevel The log level for given messages.
		 * \param local_loglevel The log level of the current context 
		 * (application, module, etc.).
		 */
		template<typename T1, typename T2, typename T3, typename T4>
		static void log(const T1 &msg1,
				const T2 &msg2,
				const T3 &msg3,
				const T4 &msg4,
				const log_level_t msg_loglevel = LOGLEVEL_ALL,
				const log_level_t local_loglevel = LOGLEVEL_ALL);
		/**
		 * \brief Writes five messages to logger
		 * \tparam msg1 First message which will be written to logger
		 * by output stream operator.
		 * \tparam msg2 Second message which will be written to logger
		 * by output stream operator.
		 * \tparam msg3 Third message which will be written to logger
		 * by output stream operator.
		 * \tparam msg4 Fourth message which will be written to logger
		 * by output stream operator.
		 * \tparam msg5 Fifth message which will be written to logger
		 * by output stream operator.
		 * \param msg_loglevel The log level for given messages.
		 * \param local_loglevel The log level of the current context 
		 * (application, module, etc.).
		 */
		template<typename T1, typename T2, typename T3, typename T4, typename T5>
		static void log(const T1 &msg1,
				const T2 &msg2,
				const T3 &msg3,
				const T4 &msg4,
				const T5 &msg5,
				const log_level_t msg_loglevel = LOGLEVEL_ALL,
				const log_level_t local_loglevel = LOGLEVEL_ALL);
		/**
		 * \brief Writes six messages to logger
		 * \tparam msg1 First message which will be written to logger
		 * by output stream operator.
		 * \tparam msg2 Second message which will be written to logger
		 * by output stream operator.
		 * \tparam msg3 Third message which will be written to logger
		 * by output stream operator.
		 * \tparam msg4 Fourth message which will be written to logger
		 * by output stream operator.
		 * \tparam msg5 Fifth message which will be written to logger
		 * by output stream operator.
		 * \tparam msg6 Sixth message which will be written to logger
		 * by output stream operator.
		 * \param msg_loglevel The log level for given messages.
		 * \param local_loglevel The log level of the current context 
		 * (application, module, etc.).
		 */
		template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
		static void log(const T1 &msg1,
				const T2 &msg2,
				const T3 &msg3,
				const T4 &msg4,
				const T5 &msg5,
				const T6 &msg6,
				const log_level_t msg_loglevel = LOGLEVEL_ALL,
				const log_level_t local_loglevel = LOGLEVEL_ALL);

		/**
		 * \brief Writes message with log level LOGLEVEL_DEBUG to logger.
		 * \sa log(const T &message,
		 *	const log_level_t msg_loglevel,
		 *	const log_level_t local_loglevel);
		 */
		template<class T>
		static void debug(const T &message);

		/**
		 * \brief Writes message with log level LOGLEVEL_INFO to logger.
		 * \sa log(const T &message,
		 *	const log_level_t msg_loglevel,
		 *	const log_level_t local_loglevel);
		 */
		template<class T>
		static void info(const T &message);

		/**
		 * \brief Writes message with log level LOGLEVEL_WARN to logger.
		 * \sa log(const T &message,
		 *	const log_level_t msg_loglevel,
		 *	const log_level_t local_loglevel);
		 */
		template<class T>
		static void warn(const T &message);

		/**
		 * \brief Writes message with log level LOGLEVEL_ERROR to logger.
		 * \sa log(const T &message,
		 *	const log_level_t msg_loglevel,
		 *	const log_level_t local_loglevel);
		 */
		template<class T>
		static void error(const T &message);

		/**
		 * \brief Writes message with log level LOGLEVEL_FATAL to logger.
		 * \sa log(const T &message,
		 *	const log_level_t msg_loglevel,
		 *	const log_level_t local_loglevel);
		 */
		template<class T>
		static void fatal(const T &message);

		/**
		 * \brief Get logger stream.
		 * \return Returns pointer to logger stream.
		 */
		static std::ostream* output_stream();

	private:
		/// Number of different log levels
		static const int LoglevelNum = 7;
		/// Human readable form of log level
		static const char *LoglevelStrings[LoglevelNum];
		/// Current logging level
		static log_level_t log_level;
		/// Auto_ptr to take strict ownership of out_stream
		static std::auto_ptr<std::ostream> out_stream_auto_ptr;
		/// Pointer of output
		static std::ostream *out_stream;
#if defined(_REENTRANT)
		/// Mutex to protect output stream
		static pthread_mutex_t stream_mutex;
#endif

		/**
		 * \brief Constructor (intentionally undefined)
		 */
		Logger();

		/**
		 * \brief Destructor (intentionally undefined)
		 */
		~Logger();

		/**
		 * \brief Writes global log level and current time to output stream
		 * \param loglevel The log level to write to output stream.
		 */
		static void log_preamble(const log_level_t loglevel);

		/**
		 * \brief Copy constructor (intentionally undefined)
		 */
		Logger(const Logger &logger);

		/**
		 * \brief Assignment operator (intentionally undefined)
		 */
		void operator=(const Logger &logger);
};
// ----------------------------------------------------------------------------
// Logger
// ----------------------------------------------------------------------------
inline bool Logger::enabled() {
#if !defined(DISABLELOGGER)
	return log_level < LOGLEVEL_OFF;
#else
	return false;
#endif
}
inline void Logger::setLogLevel(log_level_t loglevel) {
#if !defined(DISABLELOGGER)
	if(out_stream)
		log_level = loglevel;
#endif
}
inline Logger::log_level_t Logger::loglevel() {
	return log_level;
}

#if defined(DISABLELOGGER)
inline void Logger::log_preamble(const log_level_t loglevel) { }
template<typename T>
inline void Logger::log(const T &message, const log_level_t msg_loglevel, const log_level_t local_loglevel) { }
template<typename T1, typename T2>
inline void Logger::log(const T1 &msg1,
			const T2 &msg2,
			const log_level_t msg_loglevel,
			const log_level_t local_loglevel) { }
template<typename T1, typename T2, typename T3>
inline void Logger::log(const T1 &msg1,
			const T2 &msg2,
			const T3 &msg3,
			const log_level_t msg_loglevel,
			const log_level_t local_loglevel) { }
#else
inline void Logger::log_preamble(const log_level_t loglevel) {
	time_t rawtime;
	struct tm *timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	*out_stream	<< "["
			<< std::setw(7) << std::setfill(' ') << std::left << loglevel
			<< " " << std::setw(2) << std::setfill('0') << std::right << timeinfo->tm_hour
			<< ":" << std::setw(2) << timeinfo->tm_min
			<< ":" << std::setw(2) << timeinfo->tm_sec
			<< "] ";
}
template<typename T>
void Logger::log(const T &message, const log_level_t msg_loglevel, const log_level_t local_loglevel) {
	if(msg_loglevel >= local_loglevel && msg_loglevel >= log_level) {
#if defined(_REENTRANT)
		pthread_mutex_lock(&stream_mutex);
#endif
		log_preamble(msg_loglevel);
		*out_stream << message << std::endl;
#if defined(_REENTRANT)
		pthread_mutex_unlock(&stream_mutex);
#endif
	}
}
template<typename T1, typename T2>
void Logger::log(const T1 &msg1,
		 const T2 &msg2,
		 const log_level_t msg_loglevel,
		 const log_level_t local_loglevel) {
	if(msg_loglevel >= local_loglevel && msg_loglevel >= log_level) {
#if defined(_REENTRANT)
		pthread_mutex_lock(&stream_mutex);
#endif
		log_preamble(msg_loglevel);
		*out_stream << msg1 << " " << msg2 << std::endl;
#if defined(_REENTRANT)
		pthread_mutex_unlock(&stream_mutex);
#endif
	}
}
template<typename T1, typename T2, typename T3>
void Logger::log(const T1 &msg1,
		 const T2 &msg2,
		 const T3 &msg3,
		 const log_level_t msg_loglevel,
		 const log_level_t local_loglevel) {
	if(msg_loglevel >= local_loglevel && msg_loglevel >= log_level) {
#if defined(_REENTRANT)
		pthread_mutex_lock(&stream_mutex);
#endif
		log_preamble(msg_loglevel);
		*out_stream << msg1 << " "  << msg2 << " "  << msg3 << std::endl;
#if defined(_REENTRANT)
		pthread_mutex_unlock(&stream_mutex);
#endif
	}
}
template<typename T1, typename T2, typename T3, typename T4>
void Logger::log(const T1 &msg1,
		 const T2 &msg2,
		 const T3 &msg3,
		 const T4 &msg4,
		 const log_level_t msg_loglevel,
		 const log_level_t local_loglevel) {
	if(msg_loglevel >= local_loglevel && msg_loglevel >= log_level) {
#if defined(_REENTRANT)
		pthread_mutex_lock(&stream_mutex);
#endif
		log_preamble(msg_loglevel);
		*out_stream << msg1 << " "  << msg2 << " "  << msg3 << " " << msg4 << std::endl;
#if defined(_REENTRANT)
		pthread_mutex_unlock(&stream_mutex);
#endif
	}
}
template<typename T1, typename T2, typename T3, typename T4, typename T5>
void Logger::log(const T1 &msg1,
		 const T2 &msg2,
		 const T3 &msg3,
		 const T4 &msg4,
		 const T5 &msg5,
		 const log_level_t msg_loglevel,
		 const log_level_t local_loglevel) {
	if(msg_loglevel >= local_loglevel && msg_loglevel >= log_level) {
#if defined(_REENTRANT)
		pthread_mutex_lock(&stream_mutex);
#endif
		log_preamble(msg_loglevel);
		*out_stream << msg1 << " "  << msg2 << " "  << msg3 << " " << msg4 << " " << msg5 << std::endl;
#if defined(_REENTRANT)
		pthread_mutex_unlock(&stream_mutex);
#endif
	}
}
template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
void Logger::log(const T1 &msg1,
		 const T2 &msg2,
		 const T3 &msg3,
		 const T4 &msg4,
		 const T5 &msg5,
		 const T6 &msg6,
		 const log_level_t msg_loglevel,
		 const log_level_t local_loglevel) {
	if(msg_loglevel >= local_loglevel && msg_loglevel >= log_level) {
#if defined(_REENTRANT)
		pthread_mutex_lock(&stream_mutex);
#endif
		log_preamble(msg_loglevel);
		*out_stream << msg1 << " "  << msg2 << " "  << msg3 << " " << msg4 << " " << msg5 << " " << msg6 << std::endl;
#if defined(_REENTRANT)
		pthread_mutex_unlock(&stream_mutex);
#endif
	}
}
#endif  //DISABLELOGGER
template<class T>
inline void Logger::debug(const T &message) {
	log(message, LOGLEVEL_DEBUG);
}
template<class T>
inline void Logger::info(const T &message) {
	log(message, LOGLEVEL_INFO);
}
template<class T>
inline void Logger::warn(const T &message) {
	log(message, LOGLEVEL_WARN);
}
template<class T>
inline void Logger::error(const T &message) {
	log(message, LOGLEVEL_ERROR);
}
template<class T>
inline void Logger::fatal(const T &message) {
	log(message, LOGLEVEL_FATAL);
}
inline std::ostream* Logger::output_stream() {
	return out_stream;
}

	#define LOG_PARAM(name)	\
	if( Logger::enabled() ) { \
		*Logger::output_stream()	<< __FILE__ \
						<< ":" << __LINE__ << ": " << # name \
						<< " = " << (name) << ::std::endl; }
#if defined(DISABLELOGGER)
	#undef LOG_PARAM
	#define LOG_PARAM(name)
#endif
} // namespace mavhub

#endif
