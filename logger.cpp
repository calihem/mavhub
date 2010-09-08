#include "logger.h"

#if !defined(DISABLELOGGER)

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
	os << static_cast<int>(level);

	return os;
}

std::istream& operator >>(std::istream &is, Logger::log_level_t &level) {
	int num_loglevel;
	is >> num_loglevel;
	level = static_cast<Logger::log_level_t>(num_loglevel % Logger::LoglevelNum);
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

#endif //DISABLELOGGER
