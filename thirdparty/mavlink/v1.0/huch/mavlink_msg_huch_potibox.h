// MESSAGE HUCH_POTIBOX PACKING

#define MAVLINK_MSG_ID_HUCH_POTIBOX 227

typedef struct __mavlink_huch_potibox_t
{
 uint64_t usec; ///< timestamp in microseconds
 int16_t a[6]; ///< analog values 1-6
 int16_t d[4]; ///< digital values 1-4
} mavlink_huch_potibox_t;

#define MAVLINK_MSG_ID_HUCH_POTIBOX_LEN 28
#define MAVLINK_MSG_ID_227_LEN 28

#define MAVLINK_MSG_HUCH_POTIBOX_FIELD_A_LEN 6
#define MAVLINK_MSG_HUCH_POTIBOX_FIELD_D_LEN 4

#define MAVLINK_MESSAGE_INFO_HUCH_POTIBOX { \
	"HUCH_POTIBOX", \
	3, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_potibox_t, usec) }, \
         { "a", NULL, MAVLINK_TYPE_INT16_T, 6, 8, offsetof(mavlink_huch_potibox_t, a) }, \
         { "d", NULL, MAVLINK_TYPE_INT16_T, 4, 20, offsetof(mavlink_huch_potibox_t, d) }, \
         } \
}


/**
 * @brief Pack a huch_potibox message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec timestamp in microseconds
 * @param a analog values 1-6
 * @param d digital values 1-4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_potibox_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, const int16_t *a, const int16_t *d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int16_t_array(buf, 8, a, 6);
	_mav_put_int16_t_array(buf, 20, d, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 28);
#else
	mavlink_huch_potibox_t packet;
	packet.usec = usec;
	mav_array_memcpy(packet.a, a, sizeof(int16_t)*6);
	mav_array_memcpy(packet.d, d, sizeof(int16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 28);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_POTIBOX;
	return mavlink_finalize_message(msg, system_id, component_id, 28, 112);
}

/**
 * @brief Pack a huch_potibox message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec timestamp in microseconds
 * @param a analog values 1-6
 * @param d digital values 1-4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_potibox_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,const int16_t *a,const int16_t *d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int16_t_array(buf, 8, a, 6);
	_mav_put_int16_t_array(buf, 20, d, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 28);
#else
	mavlink_huch_potibox_t packet;
	packet.usec = usec;
	mav_array_memcpy(packet.a, a, sizeof(int16_t)*6);
	mav_array_memcpy(packet.d, d, sizeof(int16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 28);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_POTIBOX;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 28, 112);
}

/**
 * @brief Encode a huch_potibox struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_potibox C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_potibox_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_potibox_t* huch_potibox)
{
	return mavlink_msg_huch_potibox_pack(system_id, component_id, msg, huch_potibox->usec, huch_potibox->a, huch_potibox->d);
}

/**
 * @brief Send a huch_potibox message
 * @param chan MAVLink channel to send the message
 *
 * @param usec timestamp in microseconds
 * @param a analog values 1-6
 * @param d digital values 1-4
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_potibox_send(mavlink_channel_t chan, uint64_t usec, const int16_t *a, const int16_t *d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int16_t_array(buf, 8, a, 6);
	_mav_put_int16_t_array(buf, 20, d, 4);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_POTIBOX, buf, 28, 112);
#else
	mavlink_huch_potibox_t packet;
	packet.usec = usec;
	mav_array_memcpy(packet.a, a, sizeof(int16_t)*6);
	mav_array_memcpy(packet.d, d, sizeof(int16_t)*4);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_POTIBOX, (const char *)&packet, 28, 112);
#endif
}

#endif

// MESSAGE HUCH_POTIBOX UNPACKING


/**
 * @brief Get field usec from huch_potibox message
 *
 * @return timestamp in microseconds
 */
static inline uint64_t mavlink_msg_huch_potibox_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field a from huch_potibox message
 *
 * @return analog values 1-6
 */
static inline uint16_t mavlink_msg_huch_potibox_get_a(const mavlink_message_t* msg, int16_t *a)
{
	return _MAV_RETURN_int16_t_array(msg, a, 6,  8);
}

/**
 * @brief Get field d from huch_potibox message
 *
 * @return digital values 1-4
 */
static inline uint16_t mavlink_msg_huch_potibox_get_d(const mavlink_message_t* msg, int16_t *d)
{
	return _MAV_RETURN_int16_t_array(msg, d, 4,  20);
}

/**
 * @brief Decode a huch_potibox message into a struct
 *
 * @param msg The message to decode
 * @param huch_potibox C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_potibox_decode(const mavlink_message_t* msg, mavlink_huch_potibox_t* huch_potibox)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_potibox->usec = mavlink_msg_huch_potibox_get_usec(msg);
	mavlink_msg_huch_potibox_get_a(msg, huch_potibox->a);
	mavlink_msg_huch_potibox_get_d(msg, huch_potibox->d);
#else
	memcpy(huch_potibox, _MAV_PAYLOAD(msg), 28);
#endif
}
