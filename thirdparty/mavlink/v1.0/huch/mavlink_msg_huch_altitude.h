// MESSAGE HUCH_ALTITUDE PACKING

#define MAVLINK_MSG_ID_HUCH_ALTITUDE 214

typedef struct __mavlink_huch_altitude_t
{
 uint64_t usec; ///< timestamp in microseconds
 float altitude; ///< absolute altitude in meter
} mavlink_huch_altitude_t;

#define MAVLINK_MSG_ID_HUCH_ALTITUDE_LEN 12
#define MAVLINK_MSG_ID_214_LEN 12



#define MAVLINK_MESSAGE_INFO_HUCH_ALTITUDE { \
	"HUCH_ALTITUDE", \
	2, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_altitude_t, usec) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_huch_altitude_t, altitude) }, \
         } \
}


/**
 * @brief Pack a huch_altitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec timestamp in microseconds
 * @param altitude absolute altitude in meter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_altitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_altitude_t packet;
	packet.usec = usec;
	packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_ALTITUDE;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 96);
}

/**
 * @brief Pack a huch_altitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec timestamp in microseconds
 * @param altitude absolute altitude in meter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_altitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_altitude_t packet;
	packet.usec = usec;
	packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_ALTITUDE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 96);
}

/**
 * @brief Encode a huch_altitude struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_altitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_altitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_altitude_t* huch_altitude)
{
	return mavlink_msg_huch_altitude_pack(system_id, component_id, msg, huch_altitude->usec, huch_altitude->altitude);
}

/**
 * @brief Send a huch_altitude message
 * @param chan MAVLink channel to send the message
 *
 * @param usec timestamp in microseconds
 * @param altitude absolute altitude in meter
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_altitude_send(mavlink_channel_t chan, uint64_t usec, float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, altitude);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_ALTITUDE, buf, 12, 96);
#else
	mavlink_huch_altitude_t packet;
	packet.usec = usec;
	packet.altitude = altitude;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_ALTITUDE, (const char *)&packet, 12, 96);
#endif
}

#endif

// MESSAGE HUCH_ALTITUDE UNPACKING


/**
 * @brief Get field usec from huch_altitude message
 *
 * @return timestamp in microseconds
 */
static inline uint64_t mavlink_msg_huch_altitude_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field altitude from huch_altitude message
 *
 * @return absolute altitude in meter
 */
static inline float mavlink_msg_huch_altitude_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a huch_altitude message into a struct
 *
 * @param msg The message to decode
 * @param huch_altitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_altitude_decode(const mavlink_message_t* msg, mavlink_huch_altitude_t* huch_altitude)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_altitude->usec = mavlink_msg_huch_altitude_get_usec(msg);
	huch_altitude->altitude = mavlink_msg_huch_altitude_get_altitude(msg);
#else
	memcpy(huch_altitude, _MAV_PAYLOAD(msg), 12);
#endif
}
