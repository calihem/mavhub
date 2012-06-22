// MESSAGE HUCH_MAGNETIC_KOMPASS PACKING

#define MAVLINK_MSG_ID_HUCH_MAGNETIC_KOMPASS 216

typedef struct __mavlink_huch_magnetic_kompass_t
{
 uint64_t usec; ///< timestamp in microseconds
 float data_x; ///< magnetic field x-direction in Gs
 float data_y; ///< magnetic field y-direction in Gs
 float data_z; ///< magnetic field z-direction in Gs
} mavlink_huch_magnetic_kompass_t;

#define MAVLINK_MSG_ID_HUCH_MAGNETIC_KOMPASS_LEN 20
#define MAVLINK_MSG_ID_216_LEN 20



#define MAVLINK_MESSAGE_INFO_HUCH_MAGNETIC_KOMPASS { \
	"HUCH_MAGNETIC_KOMPASS", \
	4, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_magnetic_kompass_t, usec) }, \
         { "data_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_huch_magnetic_kompass_t, data_x) }, \
         { "data_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_huch_magnetic_kompass_t, data_y) }, \
         { "data_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_huch_magnetic_kompass_t, data_z) }, \
         } \
}


/**
 * @brief Pack a huch_magnetic_kompass message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec timestamp in microseconds
 * @param data_x magnetic field x-direction in Gs
 * @param data_y magnetic field y-direction in Gs
 * @param data_z magnetic field z-direction in Gs
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_magnetic_kompass_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, float data_x, float data_y, float data_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, data_x);
	_mav_put_float(buf, 12, data_y);
	_mav_put_float(buf, 16, data_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 20);
#else
	mavlink_huch_magnetic_kompass_t packet;
	packet.usec = usec;
	packet.data_x = data_x;
	packet.data_y = data_y;
	packet.data_z = data_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_MAGNETIC_KOMPASS;
	return mavlink_finalize_message(msg, system_id, component_id, 20, 246);
}

/**
 * @brief Pack a huch_magnetic_kompass message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec timestamp in microseconds
 * @param data_x magnetic field x-direction in Gs
 * @param data_y magnetic field y-direction in Gs
 * @param data_z magnetic field z-direction in Gs
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_magnetic_kompass_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,float data_x,float data_y,float data_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, data_x);
	_mav_put_float(buf, 12, data_y);
	_mav_put_float(buf, 16, data_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 20);
#else
	mavlink_huch_magnetic_kompass_t packet;
	packet.usec = usec;
	packet.data_x = data_x;
	packet.data_y = data_y;
	packet.data_z = data_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_MAGNETIC_KOMPASS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20, 246);
}

/**
 * @brief Encode a huch_magnetic_kompass struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_magnetic_kompass C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_magnetic_kompass_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_magnetic_kompass_t* huch_magnetic_kompass)
{
	return mavlink_msg_huch_magnetic_kompass_pack(system_id, component_id, msg, huch_magnetic_kompass->usec, huch_magnetic_kompass->data_x, huch_magnetic_kompass->data_y, huch_magnetic_kompass->data_z);
}

/**
 * @brief Send a huch_magnetic_kompass message
 * @param chan MAVLink channel to send the message
 *
 * @param usec timestamp in microseconds
 * @param data_x magnetic field x-direction in Gs
 * @param data_y magnetic field y-direction in Gs
 * @param data_z magnetic field z-direction in Gs
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_magnetic_kompass_send(mavlink_channel_t chan, uint64_t usec, float data_x, float data_y, float data_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, data_x);
	_mav_put_float(buf, 12, data_y);
	_mav_put_float(buf, 16, data_z);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_MAGNETIC_KOMPASS, buf, 20, 246);
#else
	mavlink_huch_magnetic_kompass_t packet;
	packet.usec = usec;
	packet.data_x = data_x;
	packet.data_y = data_y;
	packet.data_z = data_z;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_MAGNETIC_KOMPASS, (const char *)&packet, 20, 246);
#endif
}

#endif

// MESSAGE HUCH_MAGNETIC_KOMPASS UNPACKING


/**
 * @brief Get field usec from huch_magnetic_kompass message
 *
 * @return timestamp in microseconds
 */
static inline uint64_t mavlink_msg_huch_magnetic_kompass_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field data_x from huch_magnetic_kompass message
 *
 * @return magnetic field x-direction in Gs
 */
static inline float mavlink_msg_huch_magnetic_kompass_get_data_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field data_y from huch_magnetic_kompass message
 *
 * @return magnetic field y-direction in Gs
 */
static inline float mavlink_msg_huch_magnetic_kompass_get_data_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field data_z from huch_magnetic_kompass message
 *
 * @return magnetic field z-direction in Gs
 */
static inline float mavlink_msg_huch_magnetic_kompass_get_data_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a huch_magnetic_kompass message into a struct
 *
 * @param msg The message to decode
 * @param huch_magnetic_kompass C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_magnetic_kompass_decode(const mavlink_message_t* msg, mavlink_huch_magnetic_kompass_t* huch_magnetic_kompass)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_magnetic_kompass->usec = mavlink_msg_huch_magnetic_kompass_get_usec(msg);
	huch_magnetic_kompass->data_x = mavlink_msg_huch_magnetic_kompass_get_data_x(msg);
	huch_magnetic_kompass->data_y = mavlink_msg_huch_magnetic_kompass_get_data_y(msg);
	huch_magnetic_kompass->data_z = mavlink_msg_huch_magnetic_kompass_get_data_z(msg);
#else
	memcpy(huch_magnetic_kompass, _MAV_PAYLOAD(msg), 20);
#endif
}
