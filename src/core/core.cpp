#include "core.h"

namespace mavhub {

uint16_t Core::_system_id = 0;
pthread_mutex_t Core::system_id_mutex = PTHREAD_MUTEX_INITIALIZER;
int *Core::argc = NULL;
char **Core::argv = NULL;

} //namespace mavhub
