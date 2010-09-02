#include "datacenter.h"

namespace mavhub {

pthread_mutex_t DataCenter::raw_imu_mutex = PTHREAD_MUTEX_INITIALIZER;

} //namespace mavhub
