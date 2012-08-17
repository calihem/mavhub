#include "datacenter.h"

namespace mavhub {

double DataCenter::extctrl_pitch = 0;
double DataCenter::extctrl_roll = 0;
double DataCenter::extctrl_yaw = 0;
double DataCenter::sensors[DC_NUMSENS] = {};

pthread_mutex_t DataCenter::sensors_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::extctrl_mutex = PTHREAD_MUTEX_INITIALIZER;

#ifdef HAVE_MAVLINK_H
pthread_mutex_t DataCenter::raw_imu_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::raw_pressure_mutex = PTHREAD_MUTEX_INITIALIZER;
#ifdef MAVLINK_ENABLED_HUCH
pthread_mutex_t DataCenter::mk_fc_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::huch_ranger_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::exp_ctrl_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::huch_imu_raw_adc_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::huch_mk_imu_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t DataCenter::huch_magnetic_kompass_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif // MAVLINK_ENABLED_HUCH

mavlink_raw_imu_t DataCenter::raw_imu = {};
mavlink_raw_pressure_t DataCenter::raw_pressure = {};
#ifdef MAVLINK_ENABLED_HUCH
mavlink_huch_imu_raw_adc_t DataCenter::huch_imu_raw_adc = {};
mavlink_huch_magnetic_kompass_t DataCenter::huch_magnetic_kompass = {};
mavlink_huch_mk_imu_t DataCenter::huch_mk_imu = {};
mavlink_huch_attitude_t DataCenter::huch_attitude = {};
mavlink_huch_fc_altitude_t DataCenter::huch_altitude = {};
mavlink_huch_ranger_t DataCenter::huch_ranger = {};
mavlink_mk_fc_status_t DataCenter::mk_fc_status = {};
mavlink_huch_exp_ctrl_rx_t DataCenter::exp_ctrl_rx_data = {};
#endif // MAVLINK_ENABLED_HUCH
#endif // HAVE_MAVLINK_H

#ifdef HAVE_OPENCV2
pthread_mutex_t DataCenter::fiducal_raw_mutex = PTHREAD_MUTEX_INITIALIZER;
cv::Mat DataCenter::fiducal_rot_raw = cv::Mat();
cv::Mat DataCenter::fiducal_trans_raw = cv::Mat();
#endif // HAVE_OPENCV2
	
} //namespace mavhub
