// MESSAGE HUCH_EXT_CTRL PACKING

#define MAVLINK_MSG_ID_HUCH_EXT_CTRL 223

typedef struct __mavlink_huch_ext_ctrl_t
{
 int16_t roll; ///< Desired fixed point roll angle in cdeg, i.e. a value of 300 means 3 degrees leaning to right.
 int16_t pitch; ///< Desired fixed point pitch angle in cdeg, i.e. a value of 3000 means 30 degrees.
 int16_t yaw; ///< Desired fixed point yaw angle in cdeg, i.e. a value of 1500 means 15 degrees.
 uint16_t thrust; ///< Collective thrust, normalied to 0 ... 1000.
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t mask; ///< 0x01 => set roll; 0x02 => set pitch; 0x04 => set yaw; 0x08 => set thrust;
} mavlink_huch_ext_ctrl_t;

#define MAVLINK_MSG_ID_HUCH_EXT_CTRL_LEN 11
#define MAVLINK_MSG_ID_223_LEN 11



#define MAVLINK_MESSAGE_INFO_HUCH_EXT_CTRL { \
	"HUCH_EXT_CTRL", \
	7, \
	{  { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_huch_ext_ctrl_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_huch_ext_ctrl_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_huch_ext_ctrl_t, yaw) }, \
         { "thrust", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_huch_ext_ctrl_t, thrust) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_huch_ext_ctrl_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_huch_ext_ctrl_t, target_component) }, \
         { "mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_huch_ext_ctrl_t, mask) }, \
         } \
}


/**
 * @brief Pack a huch_ext_ctrl message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param mask 0x01 => set roll; 0x02 => set pitch; 0x04 => set yaw; 0x08 => set thrust;
 * @param roll Desired fixed point roll angle in cdeg, i.e. a value of 300 means 3 degrees leaning to right.
 * @param pitch Desired fixed point pitch angle in cdeg, i.e. a value of 3000 means 30 degrees.
 * @param yaw Desired fixed point yaw angle in cdeg, i.e. a value of 1500 means 15 degrees.
 * @param thrust Collective thrust, normalied to 0 ... 1000.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_ext_ctrl_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t mask, int16_t roll, int16_t pitch, int16_t yaw, uint16_t thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[11];
	_mav_put_int16_t(buf, 0, roll);
	_mav_put_int16_t(buf, 2, pitch);
	_mav_put_int16_t(buf, 4, yaw);
	_mav_put_uint16_t(buf, 6, thrust);
	_mav_put_uint8_t(buf, 8, target_system);
	_mav_put_uint8_t(buf, 9, target_component);
	_mav_put_uint8_t(buf, 10, mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 11);
#else
	mavlink_huch_ext_ctrl_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.mask = mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 11);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_EXT_CTRL;
	return mavlink_finalize_message(msg, system_id, component_id, 11, 175);
}

/**
 * @brief Pack a huch_ext_ctrl message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param mask 0x01 => set roll; 0x02 => set pitch; 0x04 => set yaw; 0x08 => set thrust;
 * @param roll Desired fixed point roll angle in cdeg, i.e. a value of 300 means 3 degrees leaning to right.
 * @param pitch Desired fixed point pitch angle in cdeg, i.e. a value of 3000 means 30 degrees.
 * @param yaw Desired fixed point yaw angle in cdeg, i.e. a value of 1500 means 15 degrees.
 * @param thrust Collective thrust, normalied to 0 ... 1000.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_ext_ctrl_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t mask,int16_t roll,int16_t pitch,int16_t yaw,uint16_t thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[11];
	_mav_put_int16_t(buf, 0, roll);
	_mav_put_int16_t(buf, 2, pitch);
	_mav_put_int16_t(buf, 4, yaw);
	_mav_put_uint16_t(buf, 6, thrust);
	_mav_put_uint8_t(buf, 8, target_system);
	_mav_put_uint8_t(buf, 9, target_component);
	_mav_put_uint8_t(buf, 10, mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 11);
#else
	mavlink_huch_ext_ctrl_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.mask = mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 11);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_EXT_CTRL;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 11, 175);
}

/**
 * @brief Encode a huch_ext_ctrl struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_ext_ctrl C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_ext_ctrl_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_ext_ctrl_t* huch_ext_ctrl)
{
	return mavlink_msg_huch_ext_ctrl_pack(system_id, component_id, msg, huch_ext_ctrl->target_system, huch_ext_ctrl->target_component, huch_ext_ctrl->mask, huch_ext_ctrl->roll, huch_ext_ctrl->pitch, huch_ext_ctrl->yaw, huch_ext_ctrl->thrust);
}

/**
 * @brief Send a huch_ext_ctrl message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param mask 0x01 => set roll; 0x02 => set pitch; 0x04 => set yaw; 0x08 => set thrust;
 * @param roll Desired fixed point roll angle in cdeg, i.e. a value of 300 means 3 degrees leaning to right.
 * @param pitch Desired fixed point pitch angle in cdeg, i.e. a value of 3000 means 30 degrees.
 * @param yaw Desired fixed point yaw angle in cdeg, i.e. a value of 1500 means 15 degrees.
 * @param thrust Collective thrust, normalied to 0 ... 1000.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_ext_ctrl_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t mask, int16_t roll, int16_t pitch, int16_t yaw, uint16_t thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[11];
	_mav_put_int16_t(buf, 0, roll);
	_mav_put_int16_t(buf, 2, pitch);
	_mav_put_int16_t(buf, 4, yaw);
	_mav_put_uint16_t(buf, 6, thrust);
	_mav_put_uint8_t(buf, 8, target_system);
	_mav_put_uint8_t(buf, 9, target_component);
	_mav_put_uint8_t(buf, 10, mask);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_EXT_CTRL, buf, 11, 175);
#else
	mavlink_huch_ext_ctrl_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.mask = mask;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_EXT_CTRL, (const char *)&packet, 11, 175);
#endif
}

#endif

// MESSAGE HUCH_EXT_CTRL UNPACKING


/**
 * @brief Get field target_system from huch_ext_ctrl message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_huch_ext_ctrl_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field target_component from huch_ext_ctrl message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_huch_ext_ctrl_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field mask from huch_ext_ctrl message
 *
 * @return 0x01 => set roll; 0x02 => set pitch; 0x04 => set yaw; 0x08 => set thrust;
 */
static inline uint8_t mavlink_msg_huch_ext_ctrl_get_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field roll from huch_ext_ctrl message
 *
 * @return Desired fixed point roll angle in cdeg, i.e. a value of 300 means 3 degrees leaning to right.
 */
static inline int16_t mavlink_msg_huch_ext_ctrl_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field pitch from huch_ext_ctrl message
 *
 * @return Desired fixed point pitch angle in cdeg, i.e. a value of 3000 means 30 degrees.
 */
static inline int16_t mavlink_msg_huch_ext_ctrl_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field yaw from huch_ext_ctrl message
 *
 * @return Desired fixed point yaw angle in cdeg, i.e. a value of 1500 means 15 degrees.
 */
static inline int16_t mavlink_msg_huch_ext_ctrl_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field thrust from huch_ext_ctrl message
 *
 * @return Collective thrust, normalied to 0 ... 1000.
 */
static inline uint16_t mavlink_msg_huch_ext_ctrl_get_thrust(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Decode a huch_ext_ctrl message into a struct
 *
 * @param msg The message to decode
 * @param huch_ext_ctrl C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_ext_ctrl_decode(const mavlink_message_t* msg, mavlink_huch_ext_ctrl_t* huch_ext_ctrl)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_ext_ctrl->roll = mavlink_msg_huch_ext_ctrl_get_roll(msg);
	huch_ext_ctrl->pitch = mavlink_msg_huch_ext_ctrl_get_pitch(msg);
	huch_ext_ctrl->yaw = mavlink_msg_huch_ext_ctrl_get_yaw(msg);
	huch_ext_ctrl->thrust = mavlink_msg_huch_ext_ctrl_get_thrust(msg);
	huch_ext_ctrl->target_system = mavlink_msg_huch_ext_ctrl_get_target_system(msg);
	huch_ext_ctrl->target_component = mavlink_msg_huch_ext_ctrl_get_target_component(msg);
	huch_ext_ctrl->mask = mavlink_msg_huch_ext_ctrl_get_mask(msg);
#else
	memcpy(huch_ext_ctrl, _MAV_PAYLOAD(msg), 11);
#endif
}
