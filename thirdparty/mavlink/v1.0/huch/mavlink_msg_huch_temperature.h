// MESSAGE HUCH_TEMPERATURE PACKING

#define MAVLINK_MSG_ID_HUCH_TEMPERATURE 215

typedef struct __mavlink_huch_temperature_t
{
 uint64_t usec; ///< timestamp in microseconds
 int32_t temperature; ///< temperature in 0.1C
} mavlink_huch_temperature_t;

#define MAVLINK_MSG_ID_HUCH_TEMPERATURE_LEN 12
#define MAVLINK_MSG_ID_215_LEN 12



#define MAVLINK_MESSAGE_INFO_HUCH_TEMPERATURE { \
	"HUCH_TEMPERATURE", \
	2, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_temperature_t, usec) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_huch_temperature_t, temperature) }, \
         } \
}


/**
 * @brief Pack a huch_temperature message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec timestamp in microseconds
 * @param temperature temperature in 0.1C
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_temperature_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, int32_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_temperature_t packet;
	packet.usec = usec;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_TEMPERATURE;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 228);
}

/**
 * @brief Pack a huch_temperature message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec timestamp in microseconds
 * @param temperature temperature in 0.1C
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_temperature_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,int32_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_temperature_t packet;
	packet.usec = usec;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_TEMPERATURE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 228);
}

/**
 * @brief Encode a huch_temperature struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_temperature C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_temperature_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_temperature_t* huch_temperature)
{
	return mavlink_msg_huch_temperature_pack(system_id, component_id, msg, huch_temperature->usec, huch_temperature->temperature);
}

/**
 * @brief Send a huch_temperature message
 * @param chan MAVLink channel to send the message
 *
 * @param usec timestamp in microseconds
 * @param temperature temperature in 0.1C
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_temperature_send(mavlink_channel_t chan, uint64_t usec, int32_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, temperature);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_TEMPERATURE, buf, 12, 228);
#else
	mavlink_huch_temperature_t packet;
	packet.usec = usec;
	packet.temperature = temperature;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_TEMPERATURE, (const char *)&packet, 12, 228);
#endif
}

#endif

// MESSAGE HUCH_TEMPERATURE UNPACKING


/**
 * @brief Get field usec from huch_temperature message
 *
 * @return timestamp in microseconds
 */
static inline uint64_t mavlink_msg_huch_temperature_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field temperature from huch_temperature message
 *
 * @return temperature in 0.1C
 */
static inline int32_t mavlink_msg_huch_temperature_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a huch_temperature message into a struct
 *
 * @param msg The message to decode
 * @param huch_temperature C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_temperature_decode(const mavlink_message_t* msg, mavlink_huch_temperature_t* huch_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_temperature->usec = mavlink_msg_huch_temperature_get_usec(msg);
	huch_temperature->temperature = mavlink_msg_huch_temperature_get_temperature(msg);
#else
	memcpy(huch_temperature, _MAV_PAYLOAD(msg), 12);
#endif
}
