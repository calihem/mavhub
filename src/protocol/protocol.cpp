#include "protocol.h"
#include "core/logger.h"
#include <algorithm> //transform

#include <stdio.h>

namespace mavhub {

std::ostream& operator <<(std::ostream &os, const protocol_type_t &protocol_type) {
  // Logger::log(protocol_type, Logger::LOGLEVEL_DEBUG);
	switch(protocol_type) {
#ifdef HAVE_MAVLINK_H
		case MAVLINK:
			os << "mavlink";
			break;
#endif // HAVE_MAVLINK_H
#ifdef HAVE_MKHUCHLINK_H
		case MKHUCHLINK:
			os << "mkhuchlink";
			break;
#endif // HAVE_MKHUCHLINK_H
#ifdef HAVE_MKLINK_H
		case MKLINK:
			os << "mklink";
			break;
#endif // HAVE_MKLINK_H
#ifdef HAVE_MSPLINK_H
		case MSPLINK:
			os << "msplink";
                        // Logger::log("msplink", Logger::LOGLEVEL_DEBUG);
			break;
#endif // HAVE_MSPLINK_H
		default:
			os << "unknown";
			break;
	}

	return os;
}

std::istream& operator >>(std::istream &is, protocol_type_t &protocol_type) {
	protocol_type = UnknownProtocol;

	std::string protocol_type_str;
	is >> protocol_type_str;
	if( protocol_type_str.size() == 0) return is;

	// transform to lower case
	std::transform(protocol_type_str.begin(), protocol_type_str.end(), protocol_type_str.begin(), ::tolower);

	switch( protocol_type_str[0] ) {
		case 'm':
#ifdef HAVE_MAVLINK_H
			if(protocol_type_str == "mavlink") {
				protocol_type = MAVLINK;
			} else
#endif // HAVE_MAVLINK_H
#ifdef HAVE_MKHUCHLINK_H
			if(protocol_type_str == "mkhuch" 
			|| protocol_type_str == "mkhuchlink") {
				protocol_type = MKHUCHLINK;
			} else
#endif // HAVE_MKHUCHLINK_H
#ifdef HAVE_MKLINK_H
			if(protocol_type_str == "mk"
			|| protocol_type_str == "mklink" ) {
				protocol_type = MKLINK;
			} else
#endif // HAVE_MKLINK_H
#ifdef HAVE_MSPLINK_H
			if(protocol_type_str == "msp"
			|| protocol_type_str == "msplink" ) {
				protocol_type = MSPLINK;
			} else
#endif // HAVE_MSPLINK_H
			{ }
			break;
		default:
			break;
	}

        fprintf(stdout, "%d\n", protocol_type);
	return is;
}

// #ifdef HAVE_MAVLINK_H
// template <>
// int serialize<mavlink_message_t>(const mavlink_message_t &msg, uint8_t *buffer, const uint16_t length) {
// 	if(length < MAVLINK_MAX_PACKET_LEN) return 0;
// 
// 	return mavlink_msg_to_send_buffer(buffer, &msg);
// }
// 
// // template <>
// // int deserialize<mavlink_message_t>(const uint8_t *buffer, const uint16_t buf_len, mavlink_message_t &msg, const uint16_t index) {
// // int deserialize<mavlink_message_t>(const uint8_t channel, const uint8_t *buffer, const uint16_t length, mavlink_message_t &msg) {
// // 	return 0;
// /*	static mavlink_status_t status;
// 
// 	for(uint16_t i = 0; i < length; i++) {
// 		if( mavlink_parse_char(channel, buffer[i], &msg, &status) ) {
// 			return 0;
// 		}
// 	}
// 
// 	return 1;*/
// // }
// #endif // HAVE_MAVLINK_H


/*int deserialize<mkhuch_message_t>(const uint8_t *buffer, const uint16_t buf_len, mkhuch_message_t &msg, const uint16_t index ) {
// int deserialize<mkhuch_message_t>(const uint8_t channel, const uint8_t *buffer, const uint16_t length, mkhuch_message_t &msg) {
	static mkhuchlink_status_t link_status;
	link_status.seq = index;

	int rc;
	for(uint16_t i = 0; i < buf_len; i++) {
		rc = mkhuchlink_parse_char(buffer[i], &msg, &link_status);
		if(rc == 0 || rc < 0) {
			return rc;
		}
	}

	return index+buf_len;*/
// }


} //namespace mavhub
