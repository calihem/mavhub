// MESSAGE HUCH_ATTITUDE_CONTROL PACKING

#define MAVLINK_MSG_ID_HUCH_ATTITUDE_CONTROL 228

typedef struct __mavlink_huch_attitude_control_t
{
 float roll; ///< roll
 float pitch; ///< pitch
 float yaw; ///< yaw
 float thrust; ///< thrust
 uint8_t target; ///< The system to be controlled
 uint8_t mask; ///< Additive / absolute mask
} mavlink_huch_attitude_control_t;

#define MAVLINK_MSG_ID_HUCH_ATTITUDE_CONTROL_LEN 18
#define MAVLINK_MSG_ID_228_LEN 18



#define MAVLINK_MESSAGE_INFO_HUCH_ATTITUDE_CONTROL { \
	"HUCH_ATTITUDE_CONTROL", \
	6, \
	{  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_huch_attitude_control_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_huch_attitude_control_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_huch_attitude_control_t, yaw) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_huch_attitude_control_t, thrust) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_huch_attitude_control_t, target) }, \
         { "mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_huch_attitude_control_t, mask) }, \
         } \
}


/**
 * @brief Pack a huch_attitude_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system to be controlled
 * @param roll roll
 * @param pitch pitch
 * @param yaw yaw
 * @param thrust thrust
 * @param mask Additive / absolute mask
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_attitude_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target, float roll, float pitch, float yaw, float thrust, uint8_t mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, thrust);
	_mav_put_uint8_t(buf, 16, target);
	_mav_put_uint8_t(buf, 17, mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 18);
#else
	mavlink_huch_attitude_control_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.target = target;
	packet.mask = mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_ATTITUDE_CONTROL;
	return mavlink_finalize_message(msg, system_id, component_id, 18, 85);
}

/**
 * @brief Pack a huch_attitude_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system to be controlled
 * @param roll roll
 * @param pitch pitch
 * @param yaw yaw
 * @param thrust thrust
 * @param mask Additive / absolute mask
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_attitude_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target,float roll,float pitch,float yaw,float thrust,uint8_t mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, thrust);
	_mav_put_uint8_t(buf, 16, target);
	_mav_put_uint8_t(buf, 17, mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 18);
#else
	mavlink_huch_attitude_control_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.target = target;
	packet.mask = mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_ATTITUDE_CONTROL;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 85);
}

/**
 * @brief Encode a huch_attitude_control struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_attitude_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_attitude_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_attitude_control_t* huch_attitude_control)
{
	return mavlink_msg_huch_attitude_control_pack(system_id, component_id, msg, huch_attitude_control->target, huch_attitude_control->roll, huch_attitude_control->pitch, huch_attitude_control->yaw, huch_attitude_control->thrust, huch_attitude_control->mask);
}

/**
 * @brief Send a huch_attitude_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system to be controlled
 * @param roll roll
 * @param pitch pitch
 * @param yaw yaw
 * @param thrust thrust
 * @param mask Additive / absolute mask
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_attitude_control_send(mavlink_channel_t chan, uint8_t target, float roll, float pitch, float yaw, float thrust, uint8_t mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, thrust);
	_mav_put_uint8_t(buf, 16, target);
	_mav_put_uint8_t(buf, 17, mask);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_ATTITUDE_CONTROL, buf, 18, 85);
#else
	mavlink_huch_attitude_control_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.target = target;
	packet.mask = mask;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_ATTITUDE_CONTROL, (const char *)&packet, 18, 85);
#endif
}

#endif

// MESSAGE HUCH_ATTITUDE_CONTROL UNPACKING


/**
 * @brief Get field target from huch_attitude_control message
 *
 * @return The system to be controlled
 */
static inline uint8_t mavlink_msg_huch_attitude_control_get_target(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field roll from huch_attitude_control message
 *
 * @return roll
 */
static inline float mavlink_msg_huch_attitude_control_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from huch_attitude_control message
 *
 * @return pitch
 */
static inline float mavlink_msg_huch_attitude_control_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from huch_attitude_control message
 *
 * @return yaw
 */
static inline float mavlink_msg_huch_attitude_control_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field thrust from huch_attitude_control message
 *
 * @return thrust
 */
static inline float mavlink_msg_huch_attitude_control_get_thrust(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field mask from huch_attitude_control message
 *
 * @return Additive / absolute mask
 */
static inline uint8_t mavlink_msg_huch_attitude_control_get_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Decode a huch_attitude_control message into a struct
 *
 * @param msg The message to decode
 * @param huch_attitude_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_attitude_control_decode(const mavlink_message_t* msg, mavlink_huch_attitude_control_t* huch_attitude_control)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_attitude_control->roll = mavlink_msg_huch_attitude_control_get_roll(msg);
	huch_attitude_control->pitch = mavlink_msg_huch_attitude_control_get_pitch(msg);
	huch_attitude_control->yaw = mavlink_msg_huch_attitude_control_get_yaw(msg);
	huch_attitude_control->thrust = mavlink_msg_huch_attitude_control_get_thrust(msg);
	huch_attitude_control->target = mavlink_msg_huch_attitude_control_get_target(msg);
	huch_attitude_control->mask = mavlink_msg_huch_attitude_control_get_mask(msg);
#else
	memcpy(huch_attitude_control, _MAV_PAYLOAD(msg), 18);
#endif
}
