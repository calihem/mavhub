// MESSAGE HUCH_LIN_SEN_CTRL PACKING

#define MAVLINK_MSG_ID_HUCH_LIN_SEN_CTRL 225

typedef struct __mavlink_huch_lin_sen_ctrl_t
{
 uint32_t msec; ///< timestamp in milliseconds
 uint16_t arg; ///< command argument
 uint8_t cmd; ///< command
} mavlink_huch_lin_sen_ctrl_t;

#define MAVLINK_MSG_ID_HUCH_LIN_SEN_CTRL_LEN 7
#define MAVLINK_MSG_ID_225_LEN 7



#define MAVLINK_MESSAGE_INFO_HUCH_LIN_SEN_CTRL { \
	"HUCH_LIN_SEN_CTRL", \
	3, \
	{  { "msec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_huch_lin_sen_ctrl_t, msec) }, \
         { "arg", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_huch_lin_sen_ctrl_t, arg) }, \
         { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_huch_lin_sen_ctrl_t, cmd) }, \
         } \
}


/**
 * @brief Pack a huch_lin_sen_ctrl message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param msec timestamp in milliseconds
 * @param cmd command
 * @param arg command argument
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_lin_sen_ctrl_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t msec, uint8_t cmd, uint16_t arg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[7];
	_mav_put_uint32_t(buf, 0, msec);
	_mav_put_uint16_t(buf, 4, arg);
	_mav_put_uint8_t(buf, 6, cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 7);
#else
	mavlink_huch_lin_sen_ctrl_t packet;
	packet.msec = msec;
	packet.arg = arg;
	packet.cmd = cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 7);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_LIN_SEN_CTRL;
	return mavlink_finalize_message(msg, system_id, component_id, 7, 169);
}

/**
 * @brief Pack a huch_lin_sen_ctrl message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param msec timestamp in milliseconds
 * @param cmd command
 * @param arg command argument
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_lin_sen_ctrl_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t msec,uint8_t cmd,uint16_t arg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[7];
	_mav_put_uint32_t(buf, 0, msec);
	_mav_put_uint16_t(buf, 4, arg);
	_mav_put_uint8_t(buf, 6, cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 7);
#else
	mavlink_huch_lin_sen_ctrl_t packet;
	packet.msec = msec;
	packet.arg = arg;
	packet.cmd = cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 7);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_LIN_SEN_CTRL;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 7, 169);
}

/**
 * @brief Encode a huch_lin_sen_ctrl struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_lin_sen_ctrl C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_lin_sen_ctrl_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_lin_sen_ctrl_t* huch_lin_sen_ctrl)
{
	return mavlink_msg_huch_lin_sen_ctrl_pack(system_id, component_id, msg, huch_lin_sen_ctrl->msec, huch_lin_sen_ctrl->cmd, huch_lin_sen_ctrl->arg);
}

/**
 * @brief Send a huch_lin_sen_ctrl message
 * @param chan MAVLink channel to send the message
 *
 * @param msec timestamp in milliseconds
 * @param cmd command
 * @param arg command argument
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_lin_sen_ctrl_send(mavlink_channel_t chan, uint32_t msec, uint8_t cmd, uint16_t arg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[7];
	_mav_put_uint32_t(buf, 0, msec);
	_mav_put_uint16_t(buf, 4, arg);
	_mav_put_uint8_t(buf, 6, cmd);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_LIN_SEN_CTRL, buf, 7, 169);
#else
	mavlink_huch_lin_sen_ctrl_t packet;
	packet.msec = msec;
	packet.arg = arg;
	packet.cmd = cmd;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_LIN_SEN_CTRL, (const char *)&packet, 7, 169);
#endif
}

#endif

// MESSAGE HUCH_LIN_SEN_CTRL UNPACKING


/**
 * @brief Get field msec from huch_lin_sen_ctrl message
 *
 * @return timestamp in milliseconds
 */
static inline uint32_t mavlink_msg_huch_lin_sen_ctrl_get_msec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field cmd from huch_lin_sen_ctrl message
 *
 * @return command
 */
static inline uint8_t mavlink_msg_huch_lin_sen_ctrl_get_cmd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field arg from huch_lin_sen_ctrl message
 *
 * @return command argument
 */
static inline uint16_t mavlink_msg_huch_lin_sen_ctrl_get_arg(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a huch_lin_sen_ctrl message into a struct
 *
 * @param msg The message to decode
 * @param huch_lin_sen_ctrl C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_lin_sen_ctrl_decode(const mavlink_message_t* msg, mavlink_huch_lin_sen_ctrl_t* huch_lin_sen_ctrl)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_lin_sen_ctrl->msec = mavlink_msg_huch_lin_sen_ctrl_get_msec(msg);
	huch_lin_sen_ctrl->arg = mavlink_msg_huch_lin_sen_ctrl_get_arg(msg);
	huch_lin_sen_ctrl->cmd = mavlink_msg_huch_lin_sen_ctrl_get_cmd(msg);
#else
	memcpy(huch_lin_sen_ctrl, _MAV_PAYLOAD(msg), 7);
#endif
}
