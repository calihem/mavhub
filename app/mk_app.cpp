#include "mk_app.h"

#include "logger.h"
#include "utility.h"
#include "protocolstack.h"
#include <mavlink.h>

using namespace cpp_pthread;

namespace mavhub {

// macros for state_vector
#define STATE_PARAM_REQUEST_LIST	0x01
#define STATE_PING_REQUEST		0x02

const int8_t MKApp::parameter_ids[parameter_count][15] = {
	"USERPARAM_1   ",
	"USERPARAM_2   ",
	"USERPARAM_3   ",
	"USERPARAM_4   ",
	"USERPARAM_5   ",
	"USERPARAM_6   ",
	"USERPARAM_7   ",
	"USERPARAM_8   ",
	"EXTERN_CONTROL"
};


MKApp::MKApp(const Logger::log_level_t loglevel) :
		AppLayer(loglevel),
		state_vector(0) {
	pthread_mutex_init(&tx_mutex, NULL);
}

MKApp::~MKApp() {}

void MKApp::handle_input(const mavlink_message_t &msg) {
	log("MKApp got mavlink_message", Logger::LOGLEVEL_DEBUG);

	switch(msg.msgid) {
		case MAVLINK_MSG_ID_PING:
			if(mavlink_msg_ping_get_target_system(&msg) == 0) { //ping request
				//FIXME: put sending in run-loop
				Lock tx_lock(tx_mutex);
				mavlink_msg_ping_pack(owner->system_id(),
					component_id,
					&tx_msg,
					mavlink_msg_ping_get_seq(&msg),
					msg.sysid,
					msg.compid,
					get_time_us());
				send(tx_msg);
			} else { //ping answer
				//TODO
			}
			break;
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			if( (mavlink_msg_param_request_list_get_target_system(&msg) == owner->system_id())
			&& (mavlink_msg_param_request_list_get_target_component(&msg) == component_id) ) {
				state_vector |= STATE_PARAM_REQUEST_LIST;
			}
			break;
		}
/*		case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			if( (mavlink_msg_param_request_read_get_target_system(msg) == owner->system_id())
			&& (mavlink_msg_param_request_read_get_target_component(msg) == component_id) ) {
				uint16_t index = param_id2index((char*)msg->payload+2);
				if(index >= 0) {
					generic_32bit value;
					value.b[3] = parameters_get(index);
					usart0_send_param_value(index, &value.f);
				}
			}
			break;
		}
*/		case MAVLINK_MSG_ID_PARAM_SET:
			if( (mavlink_msg_param_set_get_target_system(&msg) == owner->system_id())
			&& (mavlink_msg_param_set_get_target_component(&msg) == component_id) ) {
				int8_t parameter_id[15];
				mavlink_msg_param_set_get_param_id(&msg, parameter_id);
				uint16_t index = parameter_id_to_index(parameter_id);
				
				if(index >= 0) {
					float value = mavlink_msg_param_set_get_param_value(&msg);
					parameters[index] = static_cast<uint8_t>(value);
					
					//TODO: send to MK and read from MK

					send_mavlink_param_value( static_cast<mk_param_type_t>(index) );
				}
			}
			break;
		default:
			break;
	}
}

void MKApp::run() {

	while(1) {
		handle_statuses();
		send_heartbeat();
// 		send_messages()

		sleep(1);
	}

	return;
}

void MKApp::handle_statuses() {
	if(state_vector & STATE_PARAM_REQUEST_LIST) {
		//TODO: get params from MK
				
		//FIXME: do a delayed send
		for(int i=0; i<parameter_count; i++) {
			send_mavlink_param_value( static_cast<mk_param_type_t>(i) );
		}
		state_vector &= ~STATE_PARAM_REQUEST_LIST;
	}
}

void MKApp::send_heartbeat() {
	Lock tx_lock(tx_mutex);
	mavlink_msg_heartbeat_pack(owner->system_id(), component_id, &tx_msg, MAV_QUADROTOR, MAV_AUTOPILOT_GENERIC);
	send(tx_msg);
}

void MKApp::send_mavlink_param_value(const mk_param_type_t param_type) {
	Lock tx_lock(tx_mutex);

	mavlink_msg_param_value_pack(owner->system_id(),
		component_id,
		&tx_msg,
		get_parameter_id(param_type),
		static_cast<float>( get_parameter(param_type) ),
		parameter_count,
		param_type);
	send(tx_msg);
}

const int MKApp::parameter_id_to_index(const int8_t *parameter_id) {
	//TODO: use efficient search algorithm
	for(int i=0; i<parameter_count; i++) {
		if(strncmp( (const char*)parameter_id, (const char*)parameter_ids[i], 15) == 0) {
			return i;
		}
	}
	return -1;
}

} // namespace mavhub
