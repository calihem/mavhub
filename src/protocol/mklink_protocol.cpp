#include "protocol.h"

namespace mavhub {

#ifdef HAVE_MKLINK_H
template <>
int serialize<mk_message_t>(const mk_message_t &msg, uint8_t *buffer, const uint16_t length) {

	if(msg.coded) { // serialize coded message
		if(length < msg.size + 6) return 0;
		buffer[0] = MK_MESSAGE_START;
		buffer[1] = msg.addr;
		buffer[2] = msg.type;
		memcpy(buffer+3, msg.data, msg.size);
		*(uint16_t*)(buffer+3+msg.size) = mklink_hash_calculate(buffer, 3+msg.size);
		buffer[5+msg.size] = MK_MESSAGE_STOP;
		
		return 5+msg.size;
	}

	// serialize uncoded message
	buffer[0] = MK_MESSAGE_START;
	buffer[1] = 'a' + msg.addr;
	buffer[2] = msg.type;
	uint16_t size = mklink_encode64( msg.data, msg.size, buffer+3 );
	*(uint16_t*)(buffer+3+size) = mklink_hash_calculate(buffer, 3+size);
	buffer[5+size] = MK_MESSAGE_STOP;

	return 5+size;
}

template <>
int parse_byte<mk_message_t>(const uint8_t input, uint16_t &index, mk_message_t &msg) {
	static mklink_status_t link_status;
	link_status.seq = index;

	int rc = mklink_parse_char(input, &msg, &link_status);
	index = link_status.seq;
	return rc;
}

#ifdef HAVE_MAVLINK_H

// ----------------------------------------------------------------------------
// MAVLINK <- MK
// ----------------------------------------------------------------------------
mavlink_manual_control_t* copy(mavlink_manual_control_t *destination, const mk_extern_control_t *source) {
	if(!destination || !source)
		return destination;

	destination->target = 0; //FIXME
	destination->roll = source->roll;
	destination->pitch = source->pitch;
	destination->yaw = source->yaw;
	destination->thrust = source->thrust;

	destination->roll_manual = 1;
	destination->pitch_manual = 1;
	destination->yaw_manual = 1;
	destination->thrust_manual = 1;

	return destination;
}

// ----------------------------------------------------------------------------
// MK <- MAVLINK
// ----------------------------------------------------------------------------
mk_extern_control_t* copy(mk_extern_control_t *destination, const mavlink_manual_control_t *source) {

	destination->roll = source->roll;
	destination->pitch = source->pitch;
	destination->yaw = source->yaw;
	destination->thrust = source->thrust;

	return destination;
}

// mavlink_message_t& copy(mavlink_message_t &destination, const mk_message_t &source) {
	//TODO
// 	return destination;
// }
#endif // HAVE_MAVLINK_H


#endif // HAVE_MKLINK_H

} //namespace mavhub
