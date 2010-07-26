#include "logger.h"

#if !defined(DISABLELOGGER)

#if defined(FILELOG)
#include <fstream>
#else
#include <iostream>
#endif

namespace mavhub {
const char* Logger::LoglevelStrings[LoglevelStrNum] = {
	"generic",
	"debug",
	"info",
	"warning",
	"error",
	"fatal",
	"off"
};


#if defined(STDOUTLOG) //log to stdout
Logger::log_level_t Logger::logLevel = LOGLEVEL_WARN;
std::ostream* Logger::outStream = &std::cout;

#elif defined(STDERRLOG) //log to stderr
Logger::log_level_t Logger::logLevel = LOGLEVEL_WARN;
std::ostream* Logger::outStream = &std::cerr;

#elif defined(FILELOG) //log to file
Logger::log_level_t Logger::logLevel = LOGLEVEL_WARN;
//create new file, destruction is done by auto_ptr
std::auto_ptr<std::ostream> Logger::outStream_auto_ptr
	= std::auto_ptr<std::ostream>( new std::ofstream(FILELOG, std::ios_base::app) );
//set outStream to created file
std::ostream* Logger::outStream = outStream_auto_ptr.get();

#else //disable logging
Logger::log_level_t Logger::logLevel = LOGLEVEL_OFF;
std::ostream* Logger::outStream = NULL;
#endif

} // namespace mavhub

#endif //DISABLELOGGER