#include "protocol.h"

#include <istream>
#include <cassert>

namespace mavhub {

#ifdef HAVE_MAVLINK_H
template <>
int serialize<mavlink_message_t>(const mavlink_message_t &msg, uint8_t *buffer, const uint16_t length) {
	if(length < MAVLINK_MAX_PACKET_LEN) return 0;

	return mavlink_msg_to_send_buffer(buffer, &msg);
}

template <>
int parse_byte<mavlink_message_t>(const uint8_t input, uint16_t &index, mavlink_message_t &msg) {
	switch(index) {
		case 0: //sync
			if(input != MAVLINK_STX) {
				return -1;
			}
			mavlink_start_checksum(&msg);
			index++;
			break;
		case 1: //length
			msg.len = input;
			mavlink_update_checksum(&msg, input);
			index++;
			break;
		case 2: //sequence
			msg.seq = input;
			mavlink_update_checksum(&msg, input);
			index++;
			break;
		case 3: //system id
			msg.sysid = input;
			mavlink_update_checksum(&msg, input);
			index++;
			break;
		case 4: //component id
			msg.compid = input;
			mavlink_update_checksum(&msg, input);
			index++;
			break;
		case 5: //message id
			msg.msgid = input;
			mavlink_update_checksum(&msg, input);
			index++;
			break;
		default: //payload + crc
			if(index < msg.len + 6) { //payload
				assert(MAVLINK_MAX_PAYLOAD_LEN == 255);
				msg.payload[index-6] = input;
				mavlink_update_checksum(&msg, input);
				index++;
			} else if(index == msg.len + 6) { //crc_a
				if(msg.ck_a != input) {
					index = 0;
					return -2;
				}
				index++;
			} else if(index == msg.len + 7) { //crc_b
				index = 0;
				if(msg.ck_b != input) {
					return -2;
				}
				return 0;
			} else { //index error
				index = 0;
				return -3;
			}
			break;
	}

	return index;
}

std::istream& operator >>(std::istream &is, enum MAV_TYPE &type) {
	int int_type;
	is >> int_type;
	if(int_type >= 0 && int_type <= 6) {
		type = static_cast<enum MAV_TYPE>(int_type);
	}
	return is;
}

std::istream& operator >>(std::istream &is, enum MAV_AUTOPILOT_TYPE &type) {
	int int_type;
	is >> int_type;
#ifdef MAVLINK_ENABLED_HUCH
	if(int_type >= 0 && int_type <= 4) {
#else
	if(int_type >= 0 && int_type <= 3) {
#endif
		type = static_cast<enum MAV_AUTOPILOT_TYPE>(int_type);
	}
	return is;
}

#endif // HAVE_MAVLINK_H

} //namespace mavhub
