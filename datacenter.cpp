#include "datacenter.h"

namespace mavhub {

pthread_mutex_t DataCenter::raw_imu_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::bmp085_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::hmc5843_mutex = PTHREAD_MUTEX_INITIALIZER;

mavlink_huch_bmp085_t DataCenter::bmp085_data = {};
mavlink_huch_hmc5843_t DataCenter::hmc5843_data = {};
} //namespace mavhub
