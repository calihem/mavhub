#include "datacenter.h"

namespace mavhub {

pthread_mutex_t DataCenter::raw_imu_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::bmp085_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::hmc5843_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::exp_ctrl_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::mk_fc_mutex = PTHREAD_MUTEX_INITIALIZER;

mavlink_huch_bmp085_t DataCenter::bmp085_data = {};
mavlink_huch_hmc5843_t DataCenter::hmc5843_data = {};
mavlink_huch_exp_ctrl_rx_t DataCenter::exp_ctrl_rx_data = {};
	mavlink_huch_attitude_t DataCenter::huch_attitude = {};
	mavlink_huch_altitude_t DataCenter::huch_altitude = {};
	mavlink_huch_ranger_t DataCenter::huch_ranger = {};
	mavlink_mk_fc_status_t DataCenter::mk_fc_status = {};
} //namespace mavhub
