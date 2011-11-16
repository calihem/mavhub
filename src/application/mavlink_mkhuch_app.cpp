#include "mavlink_mkhuch_app.h"

#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MKHUCHLINK_H

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

using namespace cpp_pthread;

namespace mavhub {

// macros for state_vector
#define STATE_PARAM_REQUEST_LIST	1

const int8_t MAVLinkMKHUCHApp::parameter_ids[parameter_count][15] = {
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


MAVLinkMKHUCHApp::MAVLinkMKHUCHApp(const Logger::log_level_t loglevel) :
	AppInterface("mavlink_mkhuch_app", loglevel),
	AppLayer<mavlink_message_t>("mavlink_mkhuch_app", loglevel),
	AppLayer<mkhuch_message_t>("mavlink_mkhuch_app", loglevel),
	mkhuch_msg_time( get_time_us() ),
	heartbeat_time(0)
// 	parameter_request(255),
// 	parameter_time(0) {
	{
	pthread_mutex_init(&tx_mav_mutex, NULL);
	pthread_mutex_init(&tx_mkhuch_mutex, NULL);
// 	mkhuchlink_msg_init(&tx_mk_msg);
}

MAVLinkMKHUCHApp::~MAVLinkMKHUCHApp() {}

void MAVLinkMKHUCHApp::handle_input(const mavlink_message_t &msg) {
	log("MAVLinkMKHUCHApp got mavlink_message", static_cast<int>(msg.msgid), Logger::LOGLEVEL_DEBUG);
	
	switch(msg.msgid) {
		// case MAVLINK_MSG_ID_PING: {
		// 	mavlink_ping_t ping;
		// 	mavlink_msg_ping_decode(&msg, &ping);
		// 	if(ping.target_system == 0) { //ping request
		// 		if(ping.target_component != 0
		// 		&& ping.target_component != component_id) break;

		// 		//FIXME: put sending in run-loop
		// 		Lock tx_lock(tx_mav_mutex);
		// 		mavlink_msg_ping_pack(owner()->system_id(),
		// 			component_id,
		// 			&tx_mav_msg,
		// 			mavlink_msg_ping_get_seq(&msg),
		// 			msg.sysid,
		// 			msg.compid,
		// 			ping.time);
		// 		send(tx_mav_msg);
		// 	} else if(ping.target_system == owner()->system_id()) { //ping answer
		// 		if(ping.target_component != component_id) break;
		// 		//TODO: react on answer
		// 	}
		// 	break;
		// }

		// case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		// 	if( (mavlink_msg_param_request_list_get_target_system(&msg) == owner()->system_id())
		// 	&& (mavlink_msg_param_request_list_get_target_component(&msg) == component_id) ) {
		// 		//ask for first parameter value
		// 		mk_param_type_t param_type= REVISION;
		// 		send(MKHUCH_MSG_TYPE_PARAM_REQUEST, &param_type, sizeof(mk_param_type_t));
		// 		parameter_request = REVISION;
		// 		parameter_time = get_time_us();
		// 	}
		// 	break;

		// case MAVLINK_MSG_ID_PARAM_SET:
		// 	if( (mavlink_msg_param_set_get_target_system(&msg) == owner()->system_id())
		// 	&& (mavlink_msg_param_set_get_target_component(&msg) == component_id) ) {
		// 		int8_t parameter_id[15];
		// 		mavlink_msg_param_set_get_param_id(&msg, parameter_id);
		// 		uint16_t index = parameter_id_to_index(parameter_id);

		// 		if(index >= 0) {
		// 			//send parameter to MK
		// 			mkhuch_param_t parameter;
		// 			parameter.index = index;
		// 			float value = mavlink_msg_param_set_get_param_value(&msg);
		// 			parameter.value = static_cast<uint8_t>(value);
		// 			send(MKHUCH_MSG_TYPE_PARAM_VALUE, &parameter, sizeof(mkhuch_param_t));
		// 		}
		// 	}
		// 	break;

		// case MAVLINK_MSG_ID_MANUAL_CONTROL:
		// 	if( (mavlink_msg_param_request_read_get_target_system(&msg) == owner()->system_id()) ) {
		// 		mkhuch_extern_control_t extern_control;
		// 		//set values
		// 		extern_control.roll = static_cast<int16_t>( mavlink_msg_manual_control_get_roll(&msg) );
		// 		extern_control.pitch = static_cast<int16_t>( mavlink_msg_manual_control_get_pitch(&msg) );
		// 		extern_control.yaw = static_cast<int16_t>( mavlink_msg_manual_control_get_yaw(&msg) );
		// 		extern_control.thrust = static_cast<uint16_t>( mavlink_msg_manual_control_get_thrust(&msg) );
		// 		//set mask
    //                             extern_control.mask = 0;
		// 		if( mavlink_msg_manual_control_get_roll_manual(&msg) ) {
		// 			extern_control.mask |= (1 << ROLL_MANUAL_MASK);
		// 		}
		// 		if( mavlink_msg_manual_control_get_pitch_manual(&msg) ) {
		// 			extern_control.mask |= (1 << PITCH_MANUAL_MASK);
		// 		}
		// 		if( mavlink_msg_manual_control_get_yaw_manual(&msg) ) {
		// 			extern_control.mask |= (1 << YAW_MANUAL_MASK);
		// 		}
		// 		if( mavlink_msg_manual_control_get_thrust_manual(&msg) ) {
		// 			extern_control.mask |= (1 << THRUST_MANUAL_MASK);
		// 		}
		// 		send(MKHUCH_MSG_TYPE_EXT_CTRL, &extern_control, sizeof(mkhuch_extern_control_t));
		// 	}
		// 	break;

		case MAVLINK_MSG_ID_ACTION:
			log("MAVLinkMKHUCHApp got mkhuch_message: MAVLINK_MSG_ID_ACTION", Logger::LOGLEVEL_DEBUG);
			// if( (mavlink_msg_action_get_target(&msg) == owner()->system_id())
			// && (mavlink_msg_action_get_target_component(&msg) == component_id) ) {
			// 	mkhuch_action_t action;
			// 	action.id = mavlink_msg_action_get_action(&msg);
			// 	send(MKHUCH_MSG_TYPE_ACTION, &action, sizeof(mkhuch_action_t));
			// }
			break;

		case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
			if(mavlink_msg_request_data_stream_get_target_system(&msg) == system_id()) {
				log("MAVLinkMKHUCHApp got mkhuch_message: MAVLINK_MSG_ID_REQUEST_DATA_STREAM for sys:comp", system_id(), Logger::LOGLEVEL_DEBUG);
				log("MAVLinkMKHUCHApp got mkhuch_message: MAVLINK_MSG_ID_REQUEST_DATA_STREAM stream_id",
						(int)mavlink_msg_request_data_stream_get_req_stream_id (&msg),
						Logger::LOGLEVEL_DEBUG);
				log("MAVLinkMKHUCHApp got mkhuch_message: MAVLINK_MSG_ID_REQUEST_DATA_STREAM stream_rate",
						(int)mavlink_msg_request_data_stream_get_req_message_rate (&msg),
						Logger::LOGLEVEL_DEBUG);
				log("MAVLinkMKHUCHApp got mkhuch_message: MAVLINK_MSG_ID_REQUEST_DATA_STREAM stream_start_stop",
						(int)mavlink_msg_request_data_stream_get_start_stop (&msg),
						Logger::LOGLEVEL_DEBUG);

				mkhuch_request_stream_t request_stream;
				request_stream.id = mavlink_msg_request_data_stream_get_req_stream_id (&msg);
				request_stream.rate = mavlink_msg_request_data_stream_get_req_message_rate (&msg);
				Lock tx_lock(tx_mkhuch_mutex); // lock msg buf
				mkhuchlink_msg_encode(&tx_mkhuch_msg, MKHUCH_MSG_TYPE_REQUEST_STREAM, &request_stream, sizeof(mkhuch_request_stream_t));
				AppLayer<mkhuch_message_t>::send(tx_mkhuch_msg);
			}
			break;

		default:
			break;
	}
}

void MAVLinkMKHUCHApp::handle_input(const mkhuch_message_t& msg) {
	uint64_t last_mkhuch_msg_time = mkhuch_msg_time;
	mkhuch_msg_time = get_time_us();

	//log("MAVLinkMKHUCHApp got mkhuch_message", static_cast<int>(msg.type), Logger::LOGLEVEL_DEBUG);

	//send heartbeat approx. after 1s
	heartbeat_time += mkhuch_msg_time - last_mkhuch_msg_time;
	if( heartbeat_time > 1000000 ) {
		send_heartbeat();
		heartbeat_time = 0;
	}

	switch(msg.type) {
		case MKHUCH_MSG_TYPE_MK_IMU: {
			const mkhuch_mk_imu_t *mk_imu = reinterpret_cast<const mkhuch_mk_imu_t*>(msg.data);
			mavlink_huch_imu_raw_adc_t imu_raw_adc;
			imu_raw_adc.xacc = mk_imu->x_adc_acc;
			imu_raw_adc.yacc = mk_imu->y_adc_acc;
			imu_raw_adc.zacc = mk_imu->z_adc_acc;
			imu_raw_adc.xgyro = mk_imu->x_adc_gyro;
			imu_raw_adc.ygyro = mk_imu->y_adc_gyro;
			imu_raw_adc.zgyro = mk_imu->z_adc_gyro;
			DataCenter::set_huch_imu_raw_adc(imu_raw_adc);
			Lock tx_lock(tx_mav_mutex);
			//forward raw ADC
			mavlink_msg_huch_imu_raw_adc_encode(system_id(),
				component_id,
				&tx_mav_msg,
				&imu_raw_adc
				);
			AppLayer<mavlink_message_t>::send(tx_mav_msg);
			//forward MK IMU
			//TODO: add compass value and baro
			mavlink_huch_mk_imu_t huch_mk_imu;
			huch_mk_imu.usec = mkhuch_msg_time;
			huch_mk_imu.xacc = (2500*mk_imu->x_acc)/1024; //convert normalized analog to mg
			huch_mk_imu.yacc = (2500*mk_imu->y_acc)/1024;
			huch_mk_imu.zacc = (2500*mk_imu->z_acc)/1024;
			huch_mk_imu.xgyro = (6700*mk_imu->x_adc_gyro)/(3*1024); //convert analog to 0.1 deg./sec.
			huch_mk_imu.ygyro = (6700*mk_imu->y_adc_gyro)/(3*1024);
			huch_mk_imu.zgyro = (6700*mk_imu->z_adc_gyro)/(3*1024);
			DataCenter::set_huch_mk_imu(huch_mk_imu);
			mavlink_msg_huch_mk_imu_encode(system_id(),
				component_id,
				&tx_mav_msg,
				&huch_mk_imu
				);
			AppLayer<mavlink_message_t>::send(tx_mav_msg);
			//forward pressure
			mavlink_raw_pressure_t raw_pressure;
			raw_pressure.usec = mkhuch_msg_time;
			raw_pressure.press_abs = mk_imu->press_abs;
			raw_pressure.press_diff1 = 0;	//press_diff1
			raw_pressure.press_diff2 = 0;	//press_diff2
			raw_pressure.temperature = 0;	//temperature
			DataCenter::set_raw_pressure(raw_pressure);
			mavlink_msg_raw_pressure_encode(system_id(),
				component_id,
				&tx_mav_msg,
				&raw_pressure
				);
			AppLayer<mavlink_message_t>::send(tx_mav_msg);
			//TODO: forward magneto
			break;
		}
/*		case MKHUCH_MSG_TYPE_PARAM_VALUE: {
			const mkhuch_param_t *param = reinterpret_cast<const mkhuch_param_t*>(msg.data);
			//set parameter
			uint8_t index;
			if(param->index >= parameter_count)
				index = parameter_count-1;
			else
				index = param->index;
			parameters[index] = param->value;
			//ask for next parameter
			if(index < parameter_count - 1) {
				parameter_request = index + 1;
				mk_param_type_t param_type= static_cast<mk_param_type_t>(parameter_request);
				send(MKHUCH_MSG_TYPE_PARAM_REQUEST, &param_type, sizeof(mk_param_type_t));
				parameter_time = message_time;
			} else { //got all parameters
				parameter_request = 255;
			}
			//inform others
			send_mavlink_param_value( static_cast<mk_param_type_t>(index) );
			break;
		}*/
/*		case MKHUCH_MSG_TYPE_ACTION_ACK: {

			Lock tx_lock(tx_mav_mutex);
			mavlink_msg_action_ack_pack(owner()->system_id(), component_id, &tx_mav_msg, msg.data[0], msg.data[1]);
			send(tx_mav_msg);
			break;
		}*/
		case MKHUCH_MSG_TYPE_SYSTEM_STATUS: {
			const mkhuch_system_status_t *sys_status = reinterpret_cast<const mkhuch_system_status_t*>(msg.data);
			Lock tx_lock(tx_mav_mutex);
			mavlink_msg_sys_status_pack(system_id(),
				component_id,
				&tx_mav_msg,
				sys_status->mode,
				sys_status->nav_mode,
				sys_status->state,
				1000, //FIXME: use glibtop to get load of linux system
				sys_status->vbat*100, //convert dV to mV
				0,//motor block (unsupported)
				sys_status->packet_drop);
			AppLayer<mavlink_message_t>::send(tx_mav_msg);
			break;
		}
/*		case MKHUCH_MSG_TYPE_BOOT:
			//TODO
			break;*/
		case MKHUCH_MSG_TYPE_ATTITUDE: {
			const mkhuch_attitude_t *mkhuch_attitude = reinterpret_cast<const mkhuch_attitude_t*>(msg.data);
			mavlink_attitude_t mavlink_attitude;
			mavlink_attitude.usec = mkhuch_msg_time;
			mavlink_attitude.roll = 0.001745329251994329577*(mkhuch_attitude->roll_angle);		//convert cdeg to rad with (pi/180)/10 
			mavlink_attitude.pitch = 0.001745329251994329577*(mkhuch_attitude->pitch_angle);	//convert cdeg to rad with (pi/180)/10
			mavlink_attitude.yaw = 0.001745329251994329577*(mkhuch_attitude->yaw_angle);		//convert cdeg to rad with (pi/180)/10
			//FIXME
			mavlink_attitude.rollspeed = 0;
			mavlink_attitude.pitchspeed = 0;
			mavlink_attitude.yawspeed = 0;
			Lock tx_lock(tx_mav_mutex);
			mavlink_msg_attitude_encode(system_id(),
				component_id,
				&tx_mav_msg,
				&mavlink_attitude);
			AppLayer<mavlink_message_t>::send(tx_mav_msg);
			break;
		}
		default:
			break;
	}
}

void MAVLinkMKHUCHApp::print(std::ostream &os) const {
// 	AppLayer<mavlink_message_t>::print(os);

// 	os << "* device: " << mk_dev;
}

void MAVLinkMKHUCHApp::run() {
	log("MAVLinkMKHUCHApp running", Logger::LOGLEVEL_DEBUG);
/*	fd_set read_fds;
	int fds_ready;
	timeval timeout;
	uint8_t input;
	mkhuch_message_t rx_msg;
	mkhuchlink_status_t link_status;
	uint64_t delta_time = 0, tmp64;

	log("MAVLinkMKHUCHApp running", Logger::LOGLEVEL_DEBUG);

	mkhuchlink_status_initialize(&link_status);

	while( !interrupted() ) {
		timeout.tv_sec = 1; timeout.tv_usec = 0;
		FD_ZERO(&read_fds); FD_SET( mk_dev.handle(), &read_fds);

		fds_ready = select(mk_dev.handle()+1, &read_fds, NULL, NULL, &timeout);
		if(fds_ready < 0) {
			log("select on MK device failed with", strerror(errno), Logger::LOGLEVEL_ERROR);
			continue;
		}
		if(fds_ready == 0) continue;

		//read from device
		while( mk_dev.read(&input, 1) > 0 ) {
			if( mkhuchlink_parse_char(input, &rx_msg, &link_status) == 0) {
				tmp64 = message_time;
				message_time = get_time_us();

				handle_input(rx_msg);

				//send heartbeat approx. after 1s
				delta_time += message_time - tmp64;
				if( delta_time > 1000000 ) {
					send_heartbeat();
					delta_time = 0;
				}
			}
		}
		
		if(parameter_request != 255) {
			uint64_t time = get_time_us();
			if( (time - parameter_time) > 500000 ) { //message lost
				log("MAVLinkMKHUCHApp: request timeout for parameter index", static_cast<int>(parameter_request), Logger::LOGLEVEL_WARN);
				send(MKHUCH_MSG_TYPE_PARAM_REQUEST, &parameter_request, sizeof(mk_param_type_t));
				parameter_time = get_time_us();
			}
		}
	}
	log("MAVLinkMKHUCHApp stopped", Logger::LOGLEVEL_DEBUG);*/
}

// size_t MAVLinkMKHUCHApp::send(const mkhuch_message_t& msg) {
// 	size_t sent = 0;

// 	sent += mk_dev.write(&(msg.sync), 3);
// 	sent += mk_dev.write(msg.data, msg.len);
// 	sent += mk_dev.write(&(msg.hash), 1);

// 	return sent;
// }

// size_t MAVLinkMKHUCHApp::send(const mkhuch_msg_type_t type, const void *data, const uint8_t size) {
// 	size_t sent = 0;
// 	char buf = MKHUCH_MESSAGE_START;
// 	Lock tx_lock(tx_mk_mutex);
// 
// 	//send header and data
// 	sent += mk_dev.write(&buf, 1);
// 	sent += mk_dev.write(&type, 1);
// 	sent += mk_dev.write(&size, 1);
// 	sent += mk_dev.write(data, size);
// 
// 	//calc and send hash value
// 	uint8_t hash;
// 	mkhuchlink_hash_init(&hash);
// 	hash = mkhuchlink_hash_update(type, hash);
// 	hash = mkhuchlink_hash_update(size, hash);
// 	const uint8_t *data_ptr = static_cast<const uint8_t*>(data);
// 	for(uint8_t i=0; i<size; i++) {
// 		hash = mkhuchlink_hash_update(*data_ptr++, hash);
// 	}
// 	hash = ~hash;
// 	sent += mk_dev.write(&hash, 1);

// 	return sent;
// }

void MAVLinkMKHUCHApp::send_heartbeat() {
	Lock tx_lock(tx_mav_mutex);
	mavlink_msg_heartbeat_pack(system_id(), component_id, &tx_mav_msg, MAV_QUADROTOR, MAV_AUTOPILOT_HUCH);
	AppLayer<mavlink_message_t>::send(tx_mav_msg);
}

// void MAVLinkMKHUCHApp::send_mavlink_param_value(const mk_param_type_t param_type) {
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

// const int MAVLinkMKHUCHApp::parameter_id_to_index(const int8_t *parameter_id) {
	//TODO: use efficient search algorithm
/*	for(int i=0; i<parameter_count; i++) {
		if(strncmp( (const char*)parameter_id, (const char*)parameter_ids[i], 15) == 0) {
			return i;
		}
	}*/
// 	return -1;
// }

} // namespace mavhub

#endif // HAVE_MKHUCHLINK_H
#endif // HAVE_MAVLINK_H

