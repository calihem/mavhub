#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <string>
#include <ostream>
#include <memory>
#include <time.h>

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

			Logger();
			virtual ~Logger();

			/// return true if logging level is lower than LOG_OFF
			static bool enabled();
			/// set logging level
			static void setLogLevel(log_level_t loglevel);
			static void log(const char *message, log_level_t loglevel = LOGLEVEL_ALL);
			static void log(const char *message, int value, log_level_t loglevel = LOGLEVEL_ALL);
			static void log(const char *message, int val1, int val2, log_level_t loglevel = LOGLEVEL_ALL);
			static void debug(const char *message);
			static void info(const char *message);
			static void warn(const char *message);
			static void error(const char *message);
			static void fatal(const char *message);
			/// return reference of pointer to output stream
			static std::ostream *& outputStream();

		private:
			/// number of different loglevel strings
			static const int LoglevelStrNum = 7;
			/// human readable form of log level
			static const char* LoglevelStrings[LoglevelStrNum];
			/// current logging level
			static log_level_t logLevel;
			///
			static std::auto_ptr< std::ostream > outStream_auto_ptr;
			/// pointer of output
			static std::ostream *outStream;

			/// write loglevel and current time to output stream
			static void logPreamble(log_level_t loglevel);
			
			Logger(const Logger &logger);
			void operator=(const Logger &logger);
	};
	// ----------------------------------------------------------------------------
	// Logger
	// ----------------------------------------------------------------------------
	inline Logger::Logger() {}
	inline Logger::~Logger() {}
	inline bool Logger::enabled() {
#if !defined(DISABLELOGGER)
		return logLevel < LOGLEVEL_OFF;
#else
		return false;
#endif
	}
	inline void Logger::setLogLevel(log_level_t loglevel) {
#if !defined(DISABLELOGGER)
		if(outStream)
			logLevel = loglevel;
#endif
	}
	inline void Logger::logPreamble(log_level_t loglevel) {
#if !defined(DISABLELOGGER)
		time_t rawtime;
		struct tm *timeinfo;

		time (&rawtime);
		timeinfo = localtime(&rawtime);
		*outStream << "["
			<< LoglevelStrings[(int)loglevel]
			<< " " << timeinfo->tm_hour
			<< ":" << timeinfo->tm_min
			<< ":" << timeinfo->tm_sec
			<< "] ";
#endif
	}
	inline void Logger::log(const char *message, log_level_t loglevel) {
#if !defined(DISABLELOGGER)
		if( loglevel >= logLevel  ) {
			logPreamble(loglevel);
			*outStream << message << std::endl;
		}
#endif
	}
	inline void Logger::log(const char *message, int value, log_level_t loglevel) {
#if !defined(DISABLELOGGER)
		if( loglevel >= logLevel  ) {
			logPreamble(loglevel);
			*outStream << message << value << std::endl;
		}
#endif
	}
	inline void Logger::log(const char *message, int val1, int val2, log_level_t loglevel) {
#if !defined(DISABLELOGGER)
		if( loglevel >= logLevel  ) {
			logPreamble(loglevel);
			*outStream << message << val1 << ", " << val2 << std::endl;
		}
#endif
	}
	inline void Logger::debug(const char *message) {
		log(message, LOGLEVEL_DEBUG);
	}
	inline void Logger::info(const char *message) {
		log(message, LOGLEVEL_INFO);
	}
	inline void Logger::warn(const char *message) {
		log(message, LOGLEVEL_WARN);
	}
	inline void Logger::error(const char *message) {
		log(message, LOGLEVEL_ERROR);
	}
	inline void Logger::fatal(const char *message) {
		log(message, LOGLEVEL_FATAL);
	}
	inline std::ostream *& Logger::outputStream() { return outStream; }

	#define LOG_PARAM(name) \
	if( Logger::enabled() ) { \
		*Logger::outputStream() << __FILE__ \
			<< ":" << __LINE__ << ": " << #name \
			<< " = " << (name) << ::std::endl; }
#if defined(DISABLELOGGER)
	#undef LOG_PARAM
	#define LOG_PARAM(name)
#endif
} // namespace mavhub

#endif