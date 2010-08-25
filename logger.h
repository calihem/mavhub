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

	class Logger {
		public:
			/// enumeration of logging levels
			enum log_level_t {
				LOGLEVEL_ALL,	//log everything
				LOGLEVEL_DEBUG,	//log debugging informations
				LOGLEVEL_INFO,	//log general informations
				LOGLEVEL_WARN,	//log warnings
				LOGLEVEL_ERROR,	//log errors
				LOGLEVEL_FATAL,	//log critical errors
				LOGLEVEL_OFF	//log nothing
			};

			/// return true if logging level is lower than LOG_OFF
			static bool enabled();
			/// set logging level
			static void setLogLevel(log_level_t loglevel);
			/// Get logging level
			static log_level_t loglevel();

			template <class T>
			static void log(const T& message, log_level_t loglevel = LOGLEVEL_ALL);
			template <class T1, class T2>
			static void log(const T1& msg1, const T2& msg2, log_level_t loglevel = LOGLEVEL_ALL);
			template <class T1, class T2, class T3>
			static void log(const T1& msg1, const T2& msg2, const T3& msg3, log_level_t loglevel = LOGLEVEL_ALL);

			template <class T>
			static void debug(const T& message);
			template <class T>
			static void info(const T& message);
			template <class T>
			static void warn(const T& message);
			template <class T>
			static void error(const T& message);
			template <class T>
			static void fatal(const T& message);
			/// return reference of pointer to output stream
			static std::ostream *& output_stream();

		private:
			/// number of different loglevel strings
			static const int LoglevelStrNum = 7;
			/// human readable form of log level
			static const char* LoglevelStrings[LoglevelStrNum];
			/// current logging level
			static log_level_t log_level;
			/// auto_ptr to take strict ownership of out_stream
			static std::auto_ptr< std::ostream > out_stream_auto_ptr;
			/// pointer of output
			static std::ostream *out_stream;
#if defined(_REENTRANT)
			/// mutex to protect output stream
			static pthread_mutex_t stream_mutex;
#endif
			Logger();
			~Logger();
			/// write loglevel and current time to output stream
			static void log_preamble(log_level_t loglevel);
			
			Logger(const Logger &logger);
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
#if !defined(DISABLELOGGER)
		return log_level;
#endif
	}

	inline void Logger::log_preamble(log_level_t loglevel) {
#if !defined(DISABLELOGGER)
		time_t rawtime;
		struct tm *timeinfo;

		time (&rawtime);
		timeinfo = localtime(&rawtime);
		*out_stream << "["
			<< std::setw(7) << std::setfill(' ') << std::left << LoglevelStrings[(int)loglevel]
			<< " " << std::setw(2) << std::setfill('0') << std::right << timeinfo->tm_hour
			<< ":" << std::setw(2) << timeinfo->tm_min
			<< ":" << std::setw(2) << timeinfo->tm_sec
			<< "] ";
#endif
	}
	template <class T>
	inline void Logger::log(const T& message, log_level_t loglevel) {
#if !defined(DISABLELOGGER)
		if( loglevel >= log_level  ) {
#if defined(_REENTRANT)
		pthread_mutex_lock(&stream_mutex);
#endif
			log_preamble(loglevel);
			*out_stream << message << std::endl;
#if defined(_REENTRANT)
		pthread_mutex_unlock(&stream_mutex);
#endif
		}
#endif //DISABLELOGGER
	}
	template <class T1, class T2>
	inline void Logger::log(const T1& msg1, const T2& msg2, log_level_t loglevel) {
#if !defined(DISABLELOGGER)
		if( loglevel >= log_level  ) {
#if defined(_REENTRANT)
		pthread_mutex_lock(&stream_mutex);
#endif
			log_preamble(loglevel);
			*out_stream << msg1 << " " << msg2 << std::endl;
#if defined(_REENTRANT)
		pthread_mutex_unlock(&stream_mutex);
#endif
		}
#endif //DISABLELOGGER
	}
	template <class T1, class T2, class T3>
	inline void Logger::log(const T1& msg1, const T2& msg2, const T3& msg3, log_level_t loglevel) {
#if !defined(DISABLELOGGER)
		if( loglevel >= log_level  ) {
#if defined(_REENTRANT)
		pthread_mutex_lock(&stream_mutex);
#endif
			log_preamble(loglevel);
			*out_stream << msg1 << " "  << msg2 << " "  << msg3 << std::endl;
#if defined(_REENTRANT)
		pthread_mutex_unlock(&stream_mutex);
#endif
		}
#endif
	}

	template <class T>
	inline void Logger::debug(const T& message) {
		log(message, LOGLEVEL_DEBUG);
	}
	template <class T>
	inline void Logger::info(const T& message) {
		log(message, LOGLEVEL_INFO);
	}
	template <class T>
	inline void Logger::warn(const T& message) {
		log(message, LOGLEVEL_WARN);
	}
	template <class T>
	inline void Logger::error(const T& message) {
		log(message, LOGLEVEL_ERROR);
	}
	template <class T>
	inline void Logger::fatal(const T& message) {
		log(message, LOGLEVEL_FATAL);
	}
	inline std::ostream *& Logger::output_stream() { return out_stream; }

	#define LOG_PARAM(name) \
	if( Logger::enabled() ) { \
		*Logger::output_stream() << __FILE__ \
			<< ":" << __LINE__ << ": " << #name \
			<< " = " << (name) << ::std::endl; }
#if defined(DISABLELOGGER)
	#undef LOG_PARAM
	#define LOG_PARAM(name)
#endif
} // namespace mavhub

#endif