#include "mavlink_mk_app.h"

#ifdef HAVE_MKLINK_H

#include "core/logger.h"
#include "core/datacenter.h"
#include "utility.h"

using namespace cpp_pthread;

namespace mavhub {

MAVLinkMKApp::MAVLinkMKApp(const Logger::log_level_t loglevel) :
	AppInterface("mavlink_mk_app", loglevel),
	AppLayer<mavlink_message_t>("mavlink_mk_app", loglevel),
	AppLayer<mk_message_t>("mavlink_mk_app", loglevel),
	message_time( get_time_us() ),
	delta_time(0)
	{
	pthread_mutex_init(&tx_mav_mutex, NULL);
	pthread_mutex_init(&tx_mk_mutex, NULL);
// 	mkhuchlink_msg_init(&tx_mk_msg);
}

MAVLinkMKApp::~MAVLinkMKApp() {}

void MAVLinkMKApp::handle_input(const mavlink_message_t &msg) {
	log("MAVLinkMKApp got mavlink_message", static_cast<int>(msg.msgid), Logger::LOGLEVEL_DEBUG);
}

void MAVLinkMKApp::handle_input(const mk_message_t& msg) {
	uint64_t tmp64 = message_time;
	message_time = get_time_us();
	log("MAVLinkMKApp got mk_message", static_cast<int>(msg.type), Logger::LOGLEVEL_DEBUG);

	//send heartbeat approx. after 1s
	delta_time += message_time - tmp64;
	if( delta_time > 1000000 ) {
		send_heartbeat();
		delta_time = 0;
	}

	//TODO: check coded
	//TODO: check size of msg

	switch(msg.type) {
		case MK_MSG_TYPE_DEBUGOUT:
			break;
		case MK_MSG_TYPE_EXT_CTRL: {
			mavlink_manual_control_t manual_control;
			copy(&manual_control, (mk_extern_control_t*)msg.data);
			Lock tx_mav_lock(tx_mav_mutex);
			mavlink_msg_manual_control_encode(system_id(),
				component_id,
				&tx_mav_msg,
				&manual_control);
			AppLayer<mavlink_message_t>::send(tx_mav_msg);
			break;
		}
		case MK_MSG_TYPE_COMPASS:
			break;
		case MK_MSG_TYPE_POLL_DEBUG:
			break;
		case MK_MSG_TYPE_ANGLE_OUT: {
			mavlink_debug_t debug;
			debug.ind = 0;
			mk_angles_t *mk_angles = (mk_angles_t*)msg.data;
			debug.value = ((float)mk_angles->pitch) / 10;
			Lock tx_mav_lock(tx_mav_mutex);
			mavlink_msg_debug_encode(system_id(),
				component_id,
				&tx_mav_msg,
				&debug);
			AppLayer<mavlink_message_t>::send(tx_mav_msg);
			break;
		}
		case MK_MSG_TYPE_MOTORTEST:
			break;
		case MK_MSG_TYPE_SETTING:
			break;
		default:
			break;
	}

}

void MAVLinkMKApp::print(std::ostream &os) const {
// 	AppLayer<mavlink_message_t>::print(os);

// 	os << "* device: " << mk_dev;
}

void MAVLinkMKApp::run() {
	log("MAVLinkMKApp running", Logger::LOGLEVEL_DEBUG);
/*	fd_set read_fds;
	int fds_ready;
	timeval timeout;
	uint8_t input;
	mkhuch_message_t rx_msg;
	mkhuchlink_status_t link_status;
	uint64_t delta_time = 0, tmp64;

	log("MAVLinkMKApp running", Logger::LOGLEVEL_DEBUG);

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
				log("MAVLinkMKApp: request timeout for parameter index", static_cast<int>(parameter_request), Logger::LOGLEVEL_WARN);
				send(MKHUCH_MSG_TYPE_PARAM_REQUEST, &parameter_request, sizeof(mk_param_type_t));
				parameter_time = get_time_us();
			}
		}
	}
	log("MAVLinkMKApp stopped", Logger::LOGLEVEL_DEBUG);*/
}

// size_t MAVLinkMKApp::send(const mkhuch_message_t& msg) {
// 	size_t sent = 0;

// 	sent += mk_dev.write(&(msg.sync), 3);
// 	sent += mk_dev.write(msg.data, msg.len);
// 	sent += mk_dev.write(&(msg.hash), 1);

// 	return sent;
// }

// size_t MAVLinkMKApp::send(const mkhuch_msg_type_t type, const void *data, const uint8_t size) {
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

void MAVLinkMKApp::send_heartbeat() {
	Lock tx_lock(tx_mav_mutex);
	mavlink_msg_heartbeat_pack(system_id(), component_id, &tx_mav_msg, MAV_QUADROTOR, MAV_AUTOPILOT_HUCH);
	AppLayer<mavlink_message_t>::send(tx_mav_msg);
}

// void MAVLinkMKApp::send_mavlink_param_value(const mk_param_type_t param_type) {
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

// const int MAVLinkMKApp::parameter_id_to_index(const int8_t *parameter_id) {
	//TODO: use efficient search algorithm
/*	for(int i=0; i<parameter_count; i++) {
		if(strncmp( (const char*)parameter_id, (const char*)parameter_ids[i], 15) == 0) {
			return i;
		}
	}*/
// 	return -1;
// }

} // namespace mavhub

#endif // HAVE_MKLINK_H
