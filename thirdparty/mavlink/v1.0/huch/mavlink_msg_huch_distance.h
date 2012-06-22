// MESSAGE HUCH_DISTANCE PACKING

#define MAVLINK_MSG_ID_HUCH_DISTANCE 217

typedef struct __mavlink_huch_distance_t
{
 uint64_t usec; ///< timestamp in microseconds
 float distance; ///< relativ distance in meter
} mavlink_huch_distance_t;

#define MAVLINK_MSG_ID_HUCH_DISTANCE_LEN 12
#define MAVLINK_MSG_ID_217_LEN 12



#define MAVLINK_MESSAGE_INFO_HUCH_DISTANCE { \
	"HUCH_DISTANCE", \
	2, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_distance_t, usec) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_huch_distance_t, distance) }, \
         } \
}


/**
 * @brief Pack a huch_distance message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec timestamp in microseconds
 * @param distance relativ distance in meter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_distance_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_distance_t packet;
	packet.usec = usec;
	packet.distance = distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_DISTANCE;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 228);
}

/**
 * @brief Pack a huch_distance message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec timestamp in microseconds
 * @param distance relativ distance in meter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_distance_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_distance_t packet;
	packet.usec = usec;
	packet.distance = distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_DISTANCE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 228);
}

/**
 * @brief Encode a huch_distance struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_distance C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_distance_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_distance_t* huch_distance)
{
	return mavlink_msg_huch_distance_pack(system_id, component_id, msg, huch_distance->usec, huch_distance->distance);
}

/**
 * @brief Send a huch_distance message
 * @param chan MAVLink channel to send the message
 *
 * @param usec timestamp in microseconds
 * @param distance relativ distance in meter
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_distance_send(mavlink_channel_t chan, uint64_t usec, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, distance);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_DISTANCE, buf, 12, 228);
#else
	mavlink_huch_distance_t packet;
	packet.usec = usec;
	packet.distance = distance;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_DISTANCE, (const char *)&packet, 12, 228);
#endif
}

#endif

// MESSAGE HUCH_DISTANCE UNPACKING


/**
 * @brief Get field usec from huch_distance message
 *
 * @return timestamp in microseconds
 */
static inline uint64_t mavlink_msg_huch_distance_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field distance from huch_distance message
 *
 * @return relativ distance in meter
 */
static inline float mavlink_msg_huch_distance_get_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a huch_distance message into a struct
 *
 * @param msg The message to decode
 * @param huch_distance C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_distance_decode(const mavlink_message_t* msg, mavlink_huch_distance_t* huch_distance)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_distance->usec = mavlink_msg_huch_distance_get_usec(msg);
	huch_distance->distance = mavlink_msg_huch_distance_get_distance(msg);
#else
	memcpy(huch_distance, _MAV_PAYLOAD(msg), 12);
#endif
}
