// MESSAGE HUCH_RAW_PRESSURE PACKING

#define MAVLINK_MSG_ID_HUCH_RAW_PRESSURE 213

typedef struct __mavlink_huch_raw_pressure_t
{
 uint64_t usec; ///< timestamp in microseconds
 int32_t pressure; ///< pressure in pascal
} mavlink_huch_raw_pressure_t;

#define MAVLINK_MSG_ID_HUCH_RAW_PRESSURE_LEN 12
#define MAVLINK_MSG_ID_213_LEN 12



#define MAVLINK_MESSAGE_INFO_HUCH_RAW_PRESSURE { \
	"HUCH_RAW_PRESSURE", \
	2, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_raw_pressure_t, usec) }, \
         { "pressure", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_huch_raw_pressure_t, pressure) }, \
         } \
}


/**
 * @brief Pack a huch_raw_pressure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec timestamp in microseconds
 * @param pressure pressure in pascal
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_raw_pressure_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, int32_t pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, pressure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_raw_pressure_t packet;
	packet.usec = usec;
	packet.pressure = pressure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_RAW_PRESSURE;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 47);
}

/**
 * @brief Pack a huch_raw_pressure message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec timestamp in microseconds
 * @param pressure pressure in pascal
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_raw_pressure_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,int32_t pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, pressure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_raw_pressure_t packet;
	packet.usec = usec;
	packet.pressure = pressure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_RAW_PRESSURE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 47);
}

/**
 * @brief Encode a huch_raw_pressure struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_raw_pressure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_raw_pressure_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_raw_pressure_t* huch_raw_pressure)
{
	return mavlink_msg_huch_raw_pressure_pack(system_id, component_id, msg, huch_raw_pressure->usec, huch_raw_pressure->pressure);
}

/**
 * @brief Send a huch_raw_pressure message
 * @param chan MAVLink channel to send the message
 *
 * @param usec timestamp in microseconds
 * @param pressure pressure in pascal
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_raw_pressure_send(mavlink_channel_t chan, uint64_t usec, int32_t pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, pressure);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_RAW_PRESSURE, buf, 12, 47);
#else
	mavlink_huch_raw_pressure_t packet;
	packet.usec = usec;
	packet.pressure = pressure;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_RAW_PRESSURE, (const char *)&packet, 12, 47);
#endif
}

#endif

// MESSAGE HUCH_RAW_PRESSURE UNPACKING


/**
 * @brief Get field usec from huch_raw_pressure message
 *
 * @return timestamp in microseconds
 */
static inline uint64_t mavlink_msg_huch_raw_pressure_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field pressure from huch_raw_pressure message
 *
 * @return pressure in pascal
 */
static inline int32_t mavlink_msg_huch_raw_pressure_get_pressure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a huch_raw_pressure message into a struct
 *
 * @param msg The message to decode
 * @param huch_raw_pressure C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_raw_pressure_decode(const mavlink_message_t* msg, mavlink_huch_raw_pressure_t* huch_raw_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_raw_pressure->usec = mavlink_msg_huch_raw_pressure_get_usec(msg);
	huch_raw_pressure->pressure = mavlink_msg_huch_raw_pressure_get_pressure(msg);
#else
	memcpy(huch_raw_pressure, _MAV_PAYLOAD(msg), 12);
#endif
}
