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
#if MAVLINK_CRC_EXTRA
        static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;
#endif

	switch(index) {
		case 0: //sync
			if(input != MAVLINK_STX) {
				return -1;
			}
			mavlink_start_checksum(&msg);
			index++;
			// magic field is set after msg is validated by crc
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
		default: { //payload + crc
			uint8_t *payload = &(msg.magic); //magic has position index-6 in mavlink_message_t
			if(index < msg.len + 6) { //payload
				assert(MAVLINK_MAX_PAYLOAD_LEN == 255);
				payload[index++] = input;
				mavlink_update_checksum(&msg, input);
			} else if(index == msg.len + 6) { //crc_a
#if MAVLINK_CRC_EXTRA
				mavlink_update_checksum(&msg, mavlink_message_crcs[msg.msgid]);
#endif
				if( input != (msg.checksum & 0xff) ) {
					index = 0;
					return -2;
				}
				// copy crc at the end of payload for using memcpy
				payload[index++] = input;
			} else if(index == msg.len + 7) { //crc_b
				if( input != (msg.checksum >> 8) ) {
					index = 0;
					return -2;
				}
				// got valid message
				msg.magic = MAVLINK_STX;
				// copy crc at the end of payload for using memcpy
				payload[index] = input;
				index = 0;
				return 0;
			} else { //index error
				index = 0;
				return -3;
			}
			break;
		}
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

std::istream& operator >>(std::istream &is, enum MAV_AUTOPILOT &type) {
	int int_type;
	is >> int_type;

	if(int_type >= 0 && int_type < MAV_AUTOPILOT_ENUM_END) {
		type = static_cast<enum MAV_AUTOPILOT>(int_type);
	}
	return is;
}

#endif // HAVE_MAVLINK_H

} //namespace mavhub
