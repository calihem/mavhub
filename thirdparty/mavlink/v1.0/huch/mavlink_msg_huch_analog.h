// MESSAGE HUCH_ANALOG PACKING

#define MAVLINK_MSG_ID_HUCH_ANALOG 219

typedef struct __mavlink_huch_analog_t
{
 uint64_t usec; ///< timestamp in microseconds
 int32_t analog; ///< analog value in ADC units
} mavlink_huch_analog_t;

#define MAVLINK_MSG_ID_HUCH_ANALOG_LEN 12
#define MAVLINK_MSG_ID_219_LEN 12



#define MAVLINK_MESSAGE_INFO_HUCH_ANALOG { \
	"HUCH_ANALOG", \
	2, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_analog_t, usec) }, \
         { "analog", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_huch_analog_t, analog) }, \
         } \
}


/**
 * @brief Pack a huch_analog message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec timestamp in microseconds
 * @param analog analog value in ADC units
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_analog_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, int32_t analog)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, analog);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_analog_t packet;
	packet.usec = usec;
	packet.analog = analog;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_ANALOG;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 121);
}

/**
 * @brief Pack a huch_analog message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec timestamp in microseconds
 * @param analog analog value in ADC units
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_analog_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,int32_t analog)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, analog);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_analog_t packet;
	packet.usec = usec;
	packet.analog = analog;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_ANALOG;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 121);
}

/**
 * @brief Encode a huch_analog struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_analog C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_analog_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_analog_t* huch_analog)
{
	return mavlink_msg_huch_analog_pack(system_id, component_id, msg, huch_analog->usec, huch_analog->analog);
}

/**
 * @brief Send a huch_analog message
 * @param chan MAVLink channel to send the message
 *
 * @param usec timestamp in microseconds
 * @param analog analog value in ADC units
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_analog_send(mavlink_channel_t chan, uint64_t usec, int32_t analog)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int32_t(buf, 8, analog);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_ANALOG, buf, 12, 121);
#else
	mavlink_huch_analog_t packet;
	packet.usec = usec;
	packet.analog = analog;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_ANALOG, (const char *)&packet, 12, 121);
#endif
}

#endif

// MESSAGE HUCH_ANALOG UNPACKING


/**
 * @brief Get field usec from huch_analog message
 *
 * @return timestamp in microseconds
 */
static inline uint64_t mavlink_msg_huch_analog_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field analog from huch_analog message
 *
 * @return analog value in ADC units
 */
static inline int32_t mavlink_msg_huch_analog_get_analog(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a huch_analog message into a struct
 *
 * @param msg The message to decode
 * @param huch_analog C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_analog_decode(const mavlink_message_t* msg, mavlink_huch_analog_t* huch_analog)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_analog->usec = mavlink_msg_huch_analog_get_usec(msg);
	huch_analog->analog = mavlink_msg_huch_analog_get_analog(msg);
#else
	memcpy(huch_analog, _MAV_PAYLOAD(msg), 12);
#endif
}
