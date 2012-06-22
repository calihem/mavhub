// MESSAGE HUCH_GENERIC_CHANNEL PACKING

#define MAVLINK_MSG_ID_HUCH_GENERIC_CHANNEL 229

typedef struct __mavlink_huch_generic_channel_t
{
 uint64_t usec; ///< Timestamp in microseconds
 float value; ///< Channel value
 uint16_t index; ///< Channel index
} mavlink_huch_generic_channel_t;

#define MAVLINK_MSG_ID_HUCH_GENERIC_CHANNEL_LEN 14
#define MAVLINK_MSG_ID_229_LEN 14



#define MAVLINK_MESSAGE_INFO_HUCH_GENERIC_CHANNEL { \
	"HUCH_GENERIC_CHANNEL", \
	3, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_generic_channel_t, usec) }, \
         { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_huch_generic_channel_t, value) }, \
         { "index", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_huch_generic_channel_t, index) }, \
         } \
}


/**
 * @brief Pack a huch_generic_channel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp in microseconds
 * @param index Channel index
 * @param value Channel value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_generic_channel_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, uint16_t index, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, value);
	_mav_put_uint16_t(buf, 12, index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 14);
#else
	mavlink_huch_generic_channel_t packet;
	packet.usec = usec;
	packet.value = value;
	packet.index = index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 14);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_GENERIC_CHANNEL;
	return mavlink_finalize_message(msg, system_id, component_id, 14, 149);
}

/**
 * @brief Pack a huch_generic_channel message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp in microseconds
 * @param index Channel index
 * @param value Channel value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_generic_channel_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,uint16_t index,float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, value);
	_mav_put_uint16_t(buf, 12, index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 14);
#else
	mavlink_huch_generic_channel_t packet;
	packet.usec = usec;
	packet.value = value;
	packet.index = index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 14);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_GENERIC_CHANNEL;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14, 149);
}

/**
 * @brief Encode a huch_generic_channel struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_generic_channel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_generic_channel_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_generic_channel_t* huch_generic_channel)
{
	return mavlink_msg_huch_generic_channel_pack(system_id, component_id, msg, huch_generic_channel->usec, huch_generic_channel->index, huch_generic_channel->value);
}

/**
 * @brief Send a huch_generic_channel message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp in microseconds
 * @param index Channel index
 * @param value Channel value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_generic_channel_send(mavlink_channel_t chan, uint64_t usec, uint16_t index, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, value);
	_mav_put_uint16_t(buf, 12, index);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_GENERIC_CHANNEL, buf, 14, 149);
#else
	mavlink_huch_generic_channel_t packet;
	packet.usec = usec;
	packet.value = value;
	packet.index = index;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_GENERIC_CHANNEL, (const char *)&packet, 14, 149);
#endif
}

#endif

// MESSAGE HUCH_GENERIC_CHANNEL UNPACKING


/**
 * @brief Get field usec from huch_generic_channel message
 *
 * @return Timestamp in microseconds
 */
static inline uint64_t mavlink_msg_huch_generic_channel_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field index from huch_generic_channel message
 *
 * @return Channel index
 */
static inline uint16_t mavlink_msg_huch_generic_channel_get_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field value from huch_generic_channel message
 *
 * @return Channel value
 */
static inline float mavlink_msg_huch_generic_channel_get_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a huch_generic_channel message into a struct
 *
 * @param msg The message to decode
 * @param huch_generic_channel C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_generic_channel_decode(const mavlink_message_t* msg, mavlink_huch_generic_channel_t* huch_generic_channel)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_generic_channel->usec = mavlink_msg_huch_generic_channel_get_usec(msg);
	huch_generic_channel->value = mavlink_msg_huch_generic_channel_get_value(msg);
	huch_generic_channel->index = mavlink_msg_huch_generic_channel_get_index(msg);
#else
	memcpy(huch_generic_channel, _MAV_PAYLOAD(msg), 14);
#endif
}
