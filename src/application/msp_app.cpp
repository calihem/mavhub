#include "msp_app.h"

#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MSPLINK_H

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

using namespace cpp_pthread;

namespace mavhub {

// macros for state_vector
#define STATE_PARAM_REQUEST_LIST	1

const int8_t MSPApp::parameter_ids[parameter_count][15] = {
	"REVISION",
	"CHANNEL_0",
	"CHANNEL_1",
	"CHANNEL_2",
	"CHANNEL_3",
	"CHANNEL_4",
	"CHANNEL_5",
	"CHANNEL_6",
	"CHANNEL_7",
	"CHANNEL_8",
	"CHANNEL_9",
	"CHANNEL_10",
	"CHANNEL_11",
	"GLOBALCONFIG",
	"HEIGHT_MIN_GAS",
	"PRESSURE_PID_D",
	"HEIGHT_MAX",
	"HEIGHT_PID_P",
	"HEIGHT_GAIN",
	"HEIGHT_ACC_IMP",
	"HEIGHT_HOVER",
	"HEIGHT_GPS_Z",
	"HEIGHT_NEUTRAL",
	"STICK_PID_P",
	"STICK_PID_D",
	"PID_YAW_P",
	"GAS_MIN",
	"GAS_MAX",
	"GYRO_ACC_GAIN",
	"COMPASS_IMP",
	"GYRO_PID_P",
	"GYRO_PID_I",
	"GYRP_PID_D",
	"GYRO_YAW_P",
	"GYRO_YAW_I",
	"GYRO_STABILITY",
	"LOW_VOL_WARN",
	"GAS_EMERGENCY",
	"GAS_EMER_TIME",
	"RECEIVER",
	"PID_I",
	"USERPARAM_1",
	"USERPARAM_2",
	"USERPARAM_3",
	"USERPARAM_4",
	"SERVO_NICK_CTL",
	"SERVO_NICK_COM",
	"SERVO_NICK_MIN",
	"SERVO_NICK_MAX",
	"SERVO_ROLL_CTL",
	"SERVO_ROLL_COM",
	"SERVO_ROLL_MIN",
	"SERVO_ROLL_MAX",
	"SERVO_NICK_REF",
	"SERVO_CTL_SPEE",
	"CAM_ORIENT",
	"SERVO_3",
	"SERVO_4",
	"SERVO_5",
	"LOOP_GAS_LIMIT",
	"LOOP_THRESHOLD",
	"LOOP_HYSTERESE",
	"AXIS_LINK_1",
	"AXIS_LINK_2",
	"AXIS_YAW_CORR",
	"ANGLE_TUO_NICK",
	"ANGLE_TUO_ROLL",
	"GYRO_ACC_ALIGN",
	"DRIFT_COMP",
	"STAB_DYNAMIC",
	"USERPARAM_5",
	"USERPARAM_6",
	"USERPARAM_7",
	"USERPARAM_8",
	"J16_BITMASK",
	"J16_TIMING",
	"J17_BITMASK",
	"J17_TIMING",
	"J16_BITM_WARN",
	"J17_BITM_WARN",
	"NAVI_GPS_MCTRL",
	"NAVI_GPS_GAIN",
	"NAVI_GPS_P",
	"NAVI_GPS_I",
	"NAVI_GPS_D",
	"NAVI_GPS_P_LIM",
	"NAVI_GPS_I_LIM",
	"NAVI_GPS_D_LIM",
	"NAVI_GSP_ACC",
	"NAVI_GPS_MISAT",
	"NAVI_STITHRESH",
	"NAVI_WINDCORR",
	"NAVI_SPEEDCOMP",
	"NAVI_OPRADIUS",
	"NAVI_ANGLELIM",
	"NAVI_LOGINTIME",
	"EXTERNCONTROL",
	"ORIENT_ANGLE",
	"ORIENT_MODCTRL",
	"MOTORSAFSWITCH",
	"BITCONFIG",
	"SERVO_COMP_INV",
	"EXTRACONFIG",
	"NAME_0"
};


MSPApp::MSPApp(const Logger::log_level_t loglevel) :
	AppInterface("msp_app", loglevel),
	AppLayer<mavlink_message_t>("msp_app", loglevel),
	AppLayer<msp_message_t>("msp_app", loglevel)
// 	msp_dev(serial_port, UART::baudrate_to_speed(baudrate) | CS8 | CLOCAL | CREAD),
// 	message_time( get_time_us() ),
// 	attitude_time( get_time_us() ),
// 	parameter_request(255),
// 	parameter_time(0) {
	{
// 	pthread_mutex_init(&tx_mav_mutex, NULL);
// 	pthread_mutex_init(&tx_msp_mutex, NULL);
// 	msplink_msg_init(&tx_msp_msg);
          int mav_type = MAV_TYPE_GENERIC;
          int component_id = 0;
          int autopilot = MAV_AUTOPILOT_GENERIC;
          //system_id()
          mavlink_msg_heartbeat_pack(21,
                                     component_id,
                                     &heartbeat_msg,
                                     mav_type,
                                     autopilot,
                                     MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,	//base mode
                                     0,	//custom mode
                                     MAV_STATE_ACTIVE);	//system status
}

MSPApp::~MSPApp() {}

void MSPApp::handle_input(const mavlink_message_t &msg) {
	log("MSPApp got mavlink_message", static_cast<int>(msg.msgid), Logger::LOGLEVEL_DEBUG);
/*
	switch(msg.msgid) {
		case MAVLINK_MSG_ID_PING: {
			mavlink_ping_t ping;
			mavlink_msg_ping_decode(&msg, &ping);
			if(ping.target_system == 0) { //ping request
				if(ping.target_component != 0
				&& ping.target_component != component_id) break;

				//FIXME: put sending in run-loop
				Lock tx_lock(tx_mav_mutex);
				mavlink_msg_ping_pack(owner()->system_id(),
					component_id,
					&tx_mav_msg,
					mavlink_msg_ping_get_seq(&msg),
					msg.sysid,
					msg.compid,
					ping.time);
				send(tx_mav_msg);
			} else if(ping.target_system == owner()->system_id()) { //ping answer
				if(ping.target_component != component_id) break;
				//TODO: react on answer
			}
			break;
		}
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			if( (mavlink_msg_param_request_list_get_target_system(&msg) == owner()->system_id())
			&& (mavlink_msg_param_request_list_get_target_component(&msg) == component_id) ) {
				//ask for first parameter value
				msp_param_type_t param_type= REVISION;
				send(MSP_MSG_TYPE_PARAM_REQUEST, &param_type, sizeof(msp_param_type_t));
				parameter_request = REVISION;
				parameter_time = get_time_us();
			}
			break;
		case MAVLINK_MSG_ID_PARAM_SET:
			if( (mavlink_msg_param_set_get_target_system(&msg) == owner()->system_id())
			&& (mavlink_msg_param_set_get_target_component(&msg) == component_id) ) {
				int8_t parameter_id[15];
				mavlink_msg_param_set_get_param_id(&msg, parameter_id);
				uint16_t index = parameter_id_to_index(parameter_id);

				if(index >= 0) {
					//send parameter to MSP
					msp_param_t parameter;
					parameter.index = index;
					float value = mavlink_msg_param_set_get_param_value(&msg);
					parameter.value = static_cast<uint8_t>(value);
					send(MSP_MSG_TYPE_PARAM_VALUE, &parameter, sizeof(msp_param_t));
				}
			}
			break;
		case MAVLINK_MSG_ID_MANUAL_CONTROL:
			if( (mavlink_msg_param_request_read_get_target_system(&msg) == owner()->system_id()) ) {
				msp_extern_control_t extern_control;
				//set values
				extern_control.roll = static_cast<int16_t>( mavlink_msg_manual_control_get_roll(&msg) );
				extern_control.pitch = static_cast<int16_t>( mavlink_msg_manual_control_get_pitch(&msg) );
				extern_control.yaw = static_cast<int16_t>( mavlink_msg_manual_control_get_yaw(&msg) );
				extern_control.thrust = static_cast<uint16_t>( mavlink_msg_manual_control_get_thrust(&msg) );
				//set mask
                                extern_control.mask = 0;
				if( mavlink_msg_manual_control_get_roll_manual(&msg) ) {
					extern_control.mask |= (1 << ROLL_MANUAL_MASK);
				}
				if( mavlink_msg_manual_control_get_pitch_manual(&msg) ) {
					extern_control.mask |= (1 << PITCH_MANUAL_MASK);
				}
				if( mavlink_msg_manual_control_get_yaw_manual(&msg) ) {
					extern_control.mask |= (1 << YAW_MANUAL_MASK);
				}
				if( mavlink_msg_manual_control_get_thrust_manual(&msg) ) {
					extern_control.mask |= (1 << THRUST_MANUAL_MASK);
				}
				send(MSP_MSG_TYPE_EXT_CTRL, &extern_control, sizeof(msp_extern_control_t));
			}
			break;
		case MAVLINK_MSG_ID_ACTION:
			if( (mavlink_msg_action_get_target(&msg) == owner()->system_id())
			&& (mavlink_msg_action_get_target_component(&msg) == component_id) ) {
				msp_action_t action;
				action.id = mavlink_msg_action_get_action(&msg);
				send(MSP_MSG_TYPE_ACTION, &action, sizeof(msp_action_t));
			}
			break;
		default:
			break;
	}*/
}

  void MSPApp::handle_input(const msp_message_t& msg) {
    log("MSPApp got msp_message", static_cast<int>(msg.type), Logger::LOGLEVEL_DEBUG);
    // msp_raw_imu_t raw_imu;
    std::ostringstream logstream;
    const msp_ident_t *msp_ident;
    const msp_raw_imu_t *msp_raw_imu;
    mavlink_message_t mavmsg;
    
        
    switch(msg.type) {
    case MSP_IDENT: {
      msp_ident = reinterpret_cast<const msp_ident_t*>(msg.data);
      // logstream << message_time << " " 
      // 	<< msp_imu->x_adc_acc << " "
      // 	<< msp_imu->y_adc_acc << " "
      // 	<< msp_imu->z_adc_acc << " "
      // 	<< msp_imu->x_adc_gyro << " "
      // 	<< msp_imu->y_adc_gyro << " "
      // 	<< msp_imu->z_adc_gyro;
      logstream << "Got MSP_IDENT, MW Version: " << (int)msp_ident->version << ", Multitype:  " << (int)msp_ident->multitype;
      Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
      // mavlink send heartbeat
      AppLayer<mavlink_message_t>::send(heartbeat_msg);
      break;
    }
    case MSP_STATUS:
      break;
    case MSP_RAW_IMU:
      msp_raw_imu = reinterpret_cast<const msp_raw_imu_t*>(msg.data);
      // std::ostringstream logstream;
      logstream << "ACC x: " << msp_raw_imu->ax << ", y: " << msp_raw_imu->ay << ", z: " << msp_raw_imu->az;
      Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
      logstream.str("");
      logstream << "GYR x: " << msp_raw_imu->gx << ", y: " << msp_raw_imu->gy << ", z: " << msp_raw_imu->gz;
      Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
      // convert to mavlink
      mavlink_msg_raw_imu_pack(21, 0, &mavmsg, 0,
                               msp_raw_imu->ax, msp_raw_imu->ay, msp_raw_imu->az,
                               msp_raw_imu->gx, msp_raw_imu->gy, msp_raw_imu->gz,
                               msp_raw_imu->magx, msp_raw_imu->magy, msp_raw_imu->magz);
      AppLayer<mavlink_message_t>::send(mavmsg);
      break;
      
    // case MSP_MSG_TYPE_MSP_IMU: {
    //   const msp_msp_imu_t *msp_imu = reinterpret_cast<const msp_msp_imu_t*>(msg.data);
    //   // std::ostringstream logstream;
    //   // logstream << message_time << " " 
    //   // 	<< msp_imu->x_adc_acc << " "
    //   // 	<< msp_imu->y_adc_acc << " "
    //   // 	<< msp_imu->z_adc_acc << " "
    //   // 	<< msp_imu->x_adc_gyro << " "
    //   // 	<< msp_imu->y_adc_gyro << " "
    //   // 	<< msp_imu->z_adc_gyro;
    //   // Logger::log(logstream.str(), Logger::LOGLEVEL_ALL, Logger::LOGLEVEL_ALL);
    //   mavlink_huch_imu_raw_adc_t imu_raw_adc;
    //   imu_raw_adc.xacc = msp_imu->x_adc_acc;
    //   imu_raw_adc.yacc = msp_imu->y_adc_acc;
    //   imu_raw_adc.zacc = msp_imu->z_adc_acc;
    //   imu_raw_adc.xgyro = msp_imu->x_adc_gyro;
    //   imu_raw_adc.ygyro = msp_imu->y_adc_gyro;
    //   imu_raw_adc.zgyro = msp_imu->z_adc_gyro;
    //   DataCenter::set_huch_imu_raw_adc(imu_raw_adc);
    //   Lock tx_lock(tx_mav_mutex);
    //   //forward raw ADC
    //   mavlink_msg_huch_imu_raw_adc_encode(owner()->system_id(),
    //                                       component_id,
    //                                       &tx_mav_msg,
    //                                       &imu_raw_adc
    //                                       );
    //   send(tx_mav_msg);
    //   //forward MSP IMU
    //   //TODO: add compass value and baro
    //   mavlink_huch_msp_imu_t huch_msp_imu;
    //   huch_msp_imu.usec = message_time;
    //   huch_msp_imu.xacc = (2500*msp_imu->x_acc)/1024; //convert normalized analog to mg
    //   huch_msp_imu.yacc = (2500*msp_imu->y_acc)/1024;
    //   huch_msp_imu.zacc = (2500*msp_imu->z_acc)/1024;
    //   huch_msp_imu.xgyro = (6700*msp_imu->x_adc_gyro)/(3*1024); //convert analog to 0.1 deg./sec.
    //   huch_msp_imu.ygyro = (6700*msp_imu->y_adc_gyro)/(3*1024);
    //   huch_msp_imu.zgyro = (6700*msp_imu->z_adc_gyro)/(3*1024);
    //   DataCenter::set_huch_msp_imu(huch_msp_imu);
    //   mavlink_msg_huch_msp_imu_encode( owner()->system_id(),
    //                                    component_id,
    //                                    &tx_mav_msg,
    //                                    &huch_msp_imu
    //                                    );
    //   send(tx_mav_msg);
    //   //forward pressure
    //   mavlink_raw_pressure_t raw_pressure;
    //   raw_pressure.usec = message_time;
    //   raw_pressure.press_abs = msp_imu->press_abs;
    //   raw_pressure.press_diff1 = 0;	//press_diff1
    //   raw_pressure.press_diff2 = 0;	//press_diff2
    //   raw_pressure.temperature = 0;	//temperature
    //   DataCenter::set_raw_pressure(raw_pressure);
    //   mavlink_msg_raw_pressure_encode(owner()->system_id(),
    //                                   component_id,
    //                                   &tx_mav_msg,
    //                                   &raw_pressure
    //                                   );
    //   send(tx_mav_msg);
    //   //TODO: forward magneto
    //   break;
    // }
    // case MSP_MSG_TYPE_PARAM_VALUE: {
    //   const msp_param_t *param = reinterpret_cast<const msp_param_t*>(msg.data);
    //   //set parameter
    //   uint8_t index;
    //   if(param->index >= parameter_count)
    //     index = parameter_count-1;
    //   else
    //     index = param->index;
    //   parameters[index] = param->value;
    //   //ask for next parameter
    //   if(index < parameter_count - 1) {
    //     parameter_request = index + 1;
    //     msp_param_type_t param_type= static_cast<msp_param_type_t>(parameter_request);
    //     send(MSP_MSG_TYPE_PARAM_REQUEST, &param_type, sizeof(msp_param_type_t));
    //     parameter_time = message_time;
    //   } else { //got all parameters
    //     parameter_request = 255;
    //   }
    //   //inform others
    //   send_mavlink_param_value( static_cast<msp_param_type_t>(index) );
    //   break;
    // }
    // case MSP_MSG_TYPE_ACTION_ACK: {

    //   Lock tx_lock(tx_mav_mutex);
    //   mavlink_msg_action_ack_pack(owner()->system_id(), component_id, &tx_mav_msg, msg.data[0], msg.data[1]);
    //   send(tx_mav_msg);
    //   break;
    // }
    // case MSP_MSG_TYPE_SYSTEM_STATUS: {
    //   const msp_system_status_t *sys_status = reinterpret_cast<const msp_system_status_t*>(msg.data);
    //   Lock tx_lock(tx_mav_mutex);
    //   mavlink_msg_sys_status_pack(owner()->system_id(),
    //                               component_id,
    //                               &tx_mav_msg,
    //                               sys_status->mode,
    //                               sys_status->nav_mode,
    //                               sys_status->state,
    //                               1000, //FIXME: use glibtop to get load of linux system
    //                               sys_status->vbat*100, //convert dV to mV
    //                               0,//motor block (unsupported)
    //                               sys_status->packet_drop);
    //   send(tx_mav_msg);
    //   break;
    // }
    // case MSP_MSG_TYPE_BOOT:
    //   //TODO
    //   break;
    // case MSP_MSG_TYPE_ATTITUDE: {
    //   const msp_attitude_t *msp_attitude = reinterpret_cast<const msp_attitude_t*>(msg.data);
    //   mavlink_attitude_t mavlink_attitude;
    //   mavlink_attitude.usec = message_time;
    //   mavlink_attitude.roll = 0.001745329251994329577*(msp_attitude->roll_angle);
    //   mavlink_attitude.pitch = 0.001745329251994329577*(msp_attitude->pitch_angle);
    //   mavlink_attitude.yaw = 0.001745329251994329577*(msp_attitude->yaw_angle);
    //   uint64_t delta_time = mavlink_attitude.usec - attitude_time;
    //   if(delta_time > 150) {
    //     uint64_t delta_time = mavlink_attitude.usec - attitude_time;
    //     mavlink_attitude.rollspeed = (1745.329251994329577*(attitude.roll_angle - msp_attitude->roll_angle)) / delta_time;
    //     mavlink_attitude.pitchspeed = (1745.329251994329577*(attitude.pitch_angle - msp_attitude->pitch_angle)) / delta_time;
    //     mavlink_attitude.yawspeed = (1745.329251994329577*(attitude.yaw_angle - msp_attitude->yaw_angle)) / delta_time;
    //   } else {
    //     mavlink_attitude.rollspeed = 0;
    //     mavlink_attitude.pitchspeed = 0;
    //     mavlink_attitude.yawspeed = 0;
    //   }
    //   memcpy(&attitude, msp_attitude, sizeof(msp_attitude_t));
    //   attitude_time = mavlink_attitude.usec;
    //   Lock tx_lock(tx_mav_mutex);
    //   mavlink_msg_attitude_encode(owner()->system_id(),
    //                               component_id,
    //                               &tx_mav_msg,
    //                               &mavlink_attitude);
    //   send(tx_mav_msg);
    //   break;
    // }
    default:
      break;
    }
  }

void MSPApp::print(std::ostream &os) const {
// 	AppLayer<mavlink_message_t>::print(os);

// 	os << "* device: " << msp_dev;
}

void MSPApp::run() {
	// fd_set read_fds;
	// int fds_ready;
	// timeval timeout;
	// uint8_t input;
	// msp_message_t rx_msg;
	// msplink_status_t link_status;
	// uint64_t delta_time = 0, tmp64;
  msp_message_t msp_msg;
  msp_ident_t ident;
  msp_status_t status;
  msp_raw_imu_t raw_imu;

  msplink_msg_init_rq(&msp_msg);

  // msp_msg.sync = MSP_MESSAGE_START;
  log("MSPApp running", Logger::LOGLEVEL_DEBUG);

  // msplink_status_initialize(&link_status);

  while( !interrupted() ) {
    msp_msg.len = 0;
    msp_msg.type = MSP_IDENT;
    msplink_msg_encode(&msp_msg, MSP_IDENT, &ident, sizeof(ident));
    AppLayer<msp_message_t>::send(msp_msg);
    // usleep(10000);

    msp_msg.len = 0;
    msp_msg.type = MSP_STATUS;
    AppLayer<msp_message_t>::send(msp_msg);
    // usleep(10000);

    msp_msg.len = 0;
    msp_msg.type = MSP_RAW_IMU;
    AppLayer<msp_message_t>::send(msp_msg);
    // usleep(10000);

    Logger::log(name(), "run", Logger::LOGLEVEL_DEBUG);
    usleep(20000);
  }
  log("MSPApp stopped", Logger::LOGLEVEL_DEBUG);
}

// size_t MSPApp::send(const msp_message_t& msg) {
// 	size_t sent = 0;

// 	sent += msp_dev.write(&(msg.sync), 3);
// 	sent += msp_dev.write(msg.data, msg.len);
// 	sent += msp_dev.write(&(msg.hash), 1);

// 	return sent;
// }

// size_t MSPApp::send(const msp_msg_type_t type, const void *data, const uint8_t size) {
// 	size_t sent = 0;
// 	char buf = MSP_MESSAGE_START;
// 	Lock tx_lock(tx_msp_mutex);
// 
// 	//send header and data
// 	sent += msp_dev.write(&buf, 1);
// 	sent += msp_dev.write(&type, 1);
// 	sent += msp_dev.write(&size, 1);
// 	sent += msp_dev.write(data, size);
// 
// 	//calc and send hash value
// 	uint8_t hash;
// 	msplink_hash_init(&hash);
// 	hash = msplink_hash_update(type, hash);
// 	hash = msplink_hash_update(size, hash);
// 	const uint8_t *data_ptr = static_cast<const uint8_t*>(data);
// 	for(uint8_t i=0; i<size; i++) {
// 		hash = msplink_hash_update(*data_ptr++, hash);
// 	}
// 	hash = ~hash;
// 	sent += msp_dev.write(&hash, 1);

// 	return sent;
// }

// void MSPApp::send_heartbeat() {
/*	Lock tx_lock(tx_mav_mutex);
	mavlink_msg_heartbeat_pack(owner()->system_id(), component_id, &tx_mav_msg, MAV_QUADROTOR, MAV_AUTOPILOT_HUCH);
	send(tx_mav_msg);*/
// }

// void MSPApp::send_mavlink_param_value(const msp_param_type_t param_type) {
/*	Lock tx_lock(tx_mav_mutex);

	mavlink_msg_param_value_pack(owner()->system_id(),
		component_id,
		&tx_mav_msg,
		get_parameter_id(param_type),
		static_cast<float>( get_parameter(param_type) ),
		parameter_count,
		param_type);
	send(tx_mav_msg);*/
// }

// const int MSPApp::parameter_id_to_index(const int8_t *parameter_id) {
	//TODO: use efficient search algorithm
/*	for(int i=0; i<parameter_count; i++) {
		if(strncmp( (const char*)parameter_id, (const char*)parameter_ids[i], 15) == 0) {
			return i;
		}
	}*/
// 	return -1;
// }

} // namespace mavhub

#endif // HAVE_MSPLINK_H
#endif // HAVE_MAVLINK_H

