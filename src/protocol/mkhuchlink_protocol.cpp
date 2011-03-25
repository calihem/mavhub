#include "protocol.h"

namespace mavhub {

#ifdef HAVE_MKHUCHLINK_H

template <>
int serialize<mkhuch_message_t>(const mkhuch_message_t &msg, uint8_t *buffer, const uint16_t length) {
	int needed_size = 3+msg.len+1;
	if(length < needed_size) return 0;

	memcpy(buffer, &(msg.sync), 3);
	memcpy(buffer+3, msg.data, msg.len);
	memcpy(buffer+3+msg.len, &(msg.hash), 1);

	return needed_size;
}

template <>
int parse_byte<mkhuch_message_t>(const uint8_t input, uint16_t &index, mkhuch_message_t &msg) {
	static mkhuchlink_status_t link_status;
	link_status.seq = index;

	int rc = mkhuchlink_parse_char(input, &msg, &link_status);
	index = link_status.seq;
	return rc;
}

#endif // HAVE_MKHUCHLINK_H

} //namespace mavhub
