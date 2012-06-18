// MESSAGE MK_EXTERN_CONTROL PACKING

#define MAVLINK_MSG_ID_MK_EXTERN_CONTROL 201

typedef struct __mavlink_mk_extern_control_t
{
 uint16_t gas; ///< 
 uint16_t height; ///< 
 uint8_t remote_buttons; ///< 
 int8_t nick; ///< 
 int8_t roll; ///< 
 int8_t yaw; ///< 
 uint8_t AP_flags; ///< 
 uint8_t frame; ///< 
 uint8_t config; ///< 
} mavlink_mk_extern_control_t;

#define MAVLINK_MSG_ID_MK_EXTERN_CONTROL_LEN 11
#define MAVLINK_MSG_ID_201_LEN 11



#define MAVLINK_MESSAGE_INFO_MK_EXTERN_CONTROL { \
	"MK_EXTERN_CONTROL", \
	9, \
	{  { "gas", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mk_extern_control_t, gas) }, \
         { "height", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_mk_extern_control_t, height) }, \
         { "remote_buttons", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mk_extern_control_t, remote_buttons) }, \
         { "nick", NULL, MAVLINK_TYPE_INT8_T, 0, 5, offsetof(mavlink_mk_extern_control_t, nick) }, \
         { "roll", NULL, MAVLINK_TYPE_INT8_T, 0, 6, offsetof(mavlink_mk_extern_control_t, roll) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT8_T, 0, 7, offsetof(mavlink_mk_extern_control_t, yaw) }, \
         { "AP_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_mk_extern_control_t, AP_flags) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_mk_extern_control_t, frame) }, \
         { "config", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_mk_extern_control_t, config) }, \
         } \
}


/**
 * @brief Pack a mk_extern_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param remote_buttons 
 * @param nick 
 * @param roll 
 * @param yaw 
 * @param gas 
 * @param height 
 * @param AP_flags 
 * @param frame 
 * @param config 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mk_extern_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t remote_buttons, int8_t nick, int8_t roll, int8_t yaw, uint16_t gas, uint16_t height, uint8_t AP_flags, uint8_t frame, uint8_t config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[11];
	_mav_put_uint16_t(buf, 0, gas);
	_mav_put_uint16_t(buf, 2, height);
	_mav_put_uint8_t(buf, 4, remote_buttons);
	_mav_put_int8_t(buf, 5, nick);
	_mav_put_int8_t(buf, 6, roll);
	_mav_put_int8_t(buf, 7, yaw);
	_mav_put_uint8_t(buf, 8, AP_flags);
	_mav_put_uint8_t(buf, 9, frame);
	_mav_put_uint8_t(buf, 10, config);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 11);
#else
	mavlink_mk_extern_control_t packet;
	packet.gas = gas;
	packet.height = height;
	packet.remote_buttons = remote_buttons;
	packet.nick = nick;
	packet.roll = roll;
	packet.yaw = yaw;
	packet.AP_flags = AP_flags;
	packet.frame = frame;
	packet.config = config;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 11);
#endif

	msg->msgid = MAVLINK_MSG_ID_MK_EXTERN_CONTROL;
	return mavlink_finalize_message(msg, system_id, component_id, 11, 169);
}

/**
 * @brief Pack a mk_extern_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param remote_buttons 
 * @param nick 
 * @param roll 
 * @param yaw 
 * @param gas 
 * @param height 
 * @param AP_flags 
 * @param frame 
 * @param config 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mk_extern_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t remote_buttons,int8_t nick,int8_t roll,int8_t yaw,uint16_t gas,uint16_t height,uint8_t AP_flags,uint8_t frame,uint8_t config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[11];
	_mav_put_uint16_t(buf, 0, gas);
	_mav_put_uint16_t(buf, 2, height);
	_mav_put_uint8_t(buf, 4, remote_buttons);
	_mav_put_int8_t(buf, 5, nick);
	_mav_put_int8_t(buf, 6, roll);
	_mav_put_int8_t(buf, 7, yaw);
	_mav_put_uint8_t(buf, 8, AP_flags);
	_mav_put_uint8_t(buf, 9, frame);
	_mav_put_uint8_t(buf, 10, config);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 11);
#else
	mavlink_mk_extern_control_t packet;
	packet.gas = gas;
	packet.height = height;
	packet.remote_buttons = remote_buttons;
	packet.nick = nick;
	packet.roll = roll;
	packet.yaw = yaw;
	packet.AP_flags = AP_flags;
	packet.frame = frame;
	packet.config = config;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 11);
#endif

	msg->msgid = MAVLINK_MSG_ID_MK_EXTERN_CONTROL;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 11, 169);
}

/**
 * @brief Encode a mk_extern_control struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mk_extern_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mk_extern_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mk_extern_control_t* mk_extern_control)
{
	return mavlink_msg_mk_extern_control_pack(system_id, component_id, msg, mk_extern_control->remote_buttons, mk_extern_control->nick, mk_extern_control->roll, mk_extern_control->yaw, mk_extern_control->gas, mk_extern_control->height, mk_extern_control->AP_flags, mk_extern_control->frame, mk_extern_control->config);
}

/**
 * @brief Send a mk_extern_control message
 * @param chan MAVLink channel to send the message
 *
 * @param remote_buttons 
 * @param nick 
 * @param roll 
 * @param yaw 
 * @param gas 
 * @param height 
 * @param AP_flags 
 * @param frame 
 * @param config 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mk_extern_control_send(mavlink_channel_t chan, uint8_t remote_buttons, int8_t nick, int8_t roll, int8_t yaw, uint16_t gas, uint16_t height, uint8_t AP_flags, uint8_t frame, uint8_t config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[11];
	_mav_put_uint16_t(buf, 0, gas);
	_mav_put_uint16_t(buf, 2, height);
	_mav_put_uint8_t(buf, 4, remote_buttons);
	_mav_put_int8_t(buf, 5, nick);
	_mav_put_int8_t(buf, 6, roll);
	_mav_put_int8_t(buf, 7, yaw);
	_mav_put_uint8_t(buf, 8, AP_flags);
	_mav_put_uint8_t(buf, 9, frame);
	_mav_put_uint8_t(buf, 10, config);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MK_EXTERN_CONTROL, buf, 11, 169);
#else
	mavlink_mk_extern_control_t packet;
	packet.gas = gas;
	packet.height = height;
	packet.remote_buttons = remote_buttons;
	packet.nick = nick;
	packet.roll = roll;
	packet.yaw = yaw;
	packet.AP_flags = AP_flags;
	packet.frame = frame;
	packet.config = config;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MK_EXTERN_CONTROL, (const char *)&packet, 11, 169);
#endif
}

#endif

// MESSAGE MK_EXTERN_CONTROL UNPACKING


/**
 * @brief Get field remote_buttons from mk_extern_control message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_mk_extern_control_get_remote_buttons(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field nick from mk_extern_control message
 *
 * @return 
 */
static inline int8_t mavlink_msg_mk_extern_control_get_nick(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  5);
}

/**
 * @brief Get field roll from mk_extern_control message
 *
 * @return 
 */
static inline int8_t mavlink_msg_mk_extern_control_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  6);
}

/**
 * @brief Get field yaw from mk_extern_control message
 *
 * @return 
 */
static inline int8_t mavlink_msg_mk_extern_control_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  7);
}

/**
 * @brief Get field gas from mk_extern_control message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_mk_extern_control_get_gas(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field height from mk_extern_control message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_mk_extern_control_get_height(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field AP_flags from mk_extern_control message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_mk_extern_control_get_AP_flags(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field frame from mk_extern_control message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_mk_extern_control_get_frame(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field config from mk_extern_control message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_mk_extern_control_get_config(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Decode a mk_extern_control message into a struct
 *
 * @param msg The message to decode
 * @param mk_extern_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_mk_extern_control_decode(const mavlink_message_t* msg, mavlink_mk_extern_control_t* mk_extern_control)
{
#if MAVLINK_NEED_BYTE_SWAP
	mk_extern_control->gas = mavlink_msg_mk_extern_control_get_gas(msg);
	mk_extern_control->height = mavlink_msg_mk_extern_control_get_height(msg);
	mk_extern_control->remote_buttons = mavlink_msg_mk_extern_control_get_remote_buttons(msg);
	mk_extern_control->nick = mavlink_msg_mk_extern_control_get_nick(msg);
	mk_extern_control->roll = mavlink_msg_mk_extern_control_get_roll(msg);
	mk_extern_control->yaw = mavlink_msg_mk_extern_control_get_yaw(msg);
	mk_extern_control->AP_flags = mavlink_msg_mk_extern_control_get_AP_flags(msg);
	mk_extern_control->frame = mavlink_msg_mk_extern_control_get_frame(msg);
	mk_extern_control->config = mavlink_msg_mk_extern_control_get_config(msg);
#else
	memcpy(mk_extern_control, _MAV_PAYLOAD(msg), 11);
#endif
}
