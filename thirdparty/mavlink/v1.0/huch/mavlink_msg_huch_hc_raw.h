// MESSAGE HUCH_HC_RAW PACKING

#define MAVLINK_MSG_ID_HUCH_HC_RAW 220

typedef struct __mavlink_huch_hc_raw_t
{
 uint64_t usec; ///< timestamp in microseconds
 int32_t raw0; ///< raw altitude readings for logging and playback to controller
 int32_t raw1; ///< raw altitude readings for logging and playback to controller
 int32_t raw2; ///< raw altitude readings for logging and playback to controller
 int32_t raw3; ///< raw altitude readings for logging and playback to controller
 int32_t raw4; ///< raw altitude readings for logging and playback to controller
} mavlink_huch_hc_raw_t;

#define MAVLINK_MSG_ID_HUCH_HC_RAW_LEN 28
#define MAVLINK_MSG_ID_220_LEN 28



#define MAVLINK_MESSAGE_INFO_HUCH_HC_RAW { \
	"HUCH_HC_RAW", \
	6, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_hc_raw_t, usec) }, \
         { "raw0", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_huch_hc_raw_t, raw0) }, \
         { "raw1", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_huch_hc_raw_t, raw1) }, \
         { "raw2", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_huch_hc_raw_t, raw2) }, \
         { "raw3", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_huch_hc_raw_t, raw3) }, \
         { "raw4", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_huch_hc_raw_t, raw4) }, \
         } \
}


/**
 * @brief Pack a huch_hc_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec timestamp in microseconds
 * @param raw0 raw altitude readings for logging and playback to controller
 * @param raw1 raw altitude readings for logging and playback to controller
 * @param raw2 raw altitude readings for logging and playback to controller
 * @param raw3 raw altitude readings for logging and playback to controller
 * @param raw4 raw altitude readings for logging and playback to controller
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_hc_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, int32_t raw0, int32_t raw1, int32_t raw2, int32_t raw3, int32_t raw4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, raw0);
	_mav_put_int32_t(buf, 12, raw1);
	_mav_put_int32_t(buf, 16, raw2);
	_mav_put_int32_t(buf, 20, raw3);
	_mav_put_int32_t(buf, 24, raw4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 28);
#else
	mavlink_huch_hc_raw_t packet;
	packet.usec = usec;
	packet.raw0 = raw0;
	packet.raw1 = raw1;
	packet.raw2 = raw2;
	packet.raw3 = raw3;
	packet.raw4 = raw4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 28);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_HC_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 28, 10);
}

/**
 * @brief Pack a huch_hc_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec timestamp in microseconds
 * @param raw0 raw altitude readings for logging and playback to controller
 * @param raw1 raw altitude readings for logging and playback to controller
 * @param raw2 raw altitude readings for logging and playback to controller
 * @param raw3 raw altitude readings for logging and playback to controller
 * @param raw4 raw altitude readings for logging and playback to controller
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_hc_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,int32_t raw0,int32_t raw1,int32_t raw2,int32_t raw3,int32_t raw4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, raw0);
	_mav_put_int32_t(buf, 12, raw1);
	_mav_put_int32_t(buf, 16, raw2);
	_mav_put_int32_t(buf, 20, raw3);
	_mav_put_int32_t(buf, 24, raw4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 28);
#else
	mavlink_huch_hc_raw_t packet;
	packet.usec = usec;
	packet.raw0 = raw0;
	packet.raw1 = raw1;
	packet.raw2 = raw2;
	packet.raw3 = raw3;
	packet.raw4 = raw4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 28);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_HC_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 28, 10);
}

/**
 * @brief Encode a huch_hc_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_hc_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_hc_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_hc_raw_t* huch_hc_raw)
{
	return mavlink_msg_huch_hc_raw_pack(system_id, component_id, msg, huch_hc_raw->usec, huch_hc_raw->raw0, huch_hc_raw->raw1, huch_hc_raw->raw2, huch_hc_raw->raw3, huch_hc_raw->raw4);
}

/**
 * @brief Send a huch_hc_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param usec timestamp in microseconds
 * @param raw0 raw altitude readings for logging and playback to controller
 * @param raw1 raw altitude readings for logging and playback to controller
 * @param raw2 raw altitude readings for logging and playback to controller
 * @param raw3 raw altitude readings for logging and playback to controller
 * @param raw4 raw altitude readings for logging and playback to controller
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_hc_raw_send(mavlink_channel_t chan, uint64_t usec, int32_t raw0, int32_t raw1, int32_t raw2, int32_t raw3, int32_t raw4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, raw0);
	_mav_put_int32_t(buf, 12, raw1);
	_mav_put_int32_t(buf, 16, raw2);
	_mav_put_int32_t(buf, 20, raw3);
	_mav_put_int32_t(buf, 24, raw4);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_HC_RAW, buf, 28, 10);
#else
	mavlink_huch_hc_raw_t packet;
	packet.usec = usec;
	packet.raw0 = raw0;
	packet.raw1 = raw1;
	packet.raw2 = raw2;
	packet.raw3 = raw3;
	packet.raw4 = raw4;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_HC_RAW, (const char *)&packet, 28, 10);
#endif
}

#endif

// MESSAGE HUCH_HC_RAW UNPACKING


/**
 * @brief Get field usec from huch_hc_raw message
 *
 * @return timestamp in microseconds
 */
static inline uint64_t mavlink_msg_huch_hc_raw_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field raw0 from huch_hc_raw message
 *
 * @return raw altitude readings for logging and playback to controller
 */
static inline int32_t mavlink_msg_huch_hc_raw_get_raw0(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field raw1 from huch_hc_raw message
 *
 * @return raw altitude readings for logging and playback to controller
 */
static inline int32_t mavlink_msg_huch_hc_raw_get_raw1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field raw2 from huch_hc_raw message
 *
 * @return raw altitude readings for logging and playback to controller
 */
static inline int32_t mavlink_msg_huch_hc_raw_get_raw2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field raw3 from huch_hc_raw message
 *
 * @return raw altitude readings for logging and playback to controller
 */
static inline int32_t mavlink_msg_huch_hc_raw_get_raw3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field raw4 from huch_hc_raw message
 *
 * @return raw altitude readings for logging and playback to controller
 */
static inline int32_t mavlink_msg_huch_hc_raw_get_raw4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Decode a huch_hc_raw message into a struct
 *
 * @param msg The message to decode
 * @param huch_hc_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_hc_raw_decode(const mavlink_message_t* msg, mavlink_huch_hc_raw_t* huch_hc_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_hc_raw->usec = mavlink_msg_huch_hc_raw_get_usec(msg);
	huch_hc_raw->raw0 = mavlink_msg_huch_hc_raw_get_raw0(msg);
	huch_hc_raw->raw1 = mavlink_msg_huch_hc_raw_get_raw1(msg);
	huch_hc_raw->raw2 = mavlink_msg_huch_hc_raw_get_raw2(msg);
	huch_hc_raw->raw3 = mavlink_msg_huch_hc_raw_get_raw3(msg);
	huch_hc_raw->raw4 = mavlink_msg_huch_hc_raw_get_raw4(msg);
#else
	memcpy(huch_hc_raw, _MAV_PAYLOAD(msg), 28);
#endif
}
