#include "datacenter.h"

namespace mavhub {

pthread_mutex_t DataCenter::raw_imu_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::mk_fc_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::huch_ranger_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::exp_ctrl_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::extctrl_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::huch_imu_raw_adc_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::huch_mk_imu_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::raw_pressure_mutex = PTHREAD_MUTEX_INITIALIZER;

mavlink_raw_imu_t DataCenter::raw_imu = {};
mavlink_huch_imu_raw_adc_t DataCenter::huch_imu_raw_adc = {};
mavlink_huch_mk_imu_t DataCenter::huch_mk_imu = {};
mavlink_raw_pressure_t DataCenter::raw_pressure = {};

mavlink_huch_attitude_t DataCenter::huch_attitude = {};
mavlink_huch_fc_altitude_t DataCenter::huch_altitude = {};
mavlink_huch_ranger_t DataCenter::huch_ranger = {};
mavlink_mk_fc_status_t DataCenter::mk_fc_status = {};
mavlink_huch_exp_ctrl_rx_t DataCenter::exp_ctrl_rx_data = {};

	int16_t DataCenter::extctrl_nick = 0;
	int16_t DataCenter::extctrl_roll = 0;
	int16_t DataCenter::extctrl_yaw = 0;

} //namespace mavhub
