// MESSAGE HUCH_SIM_CTRL PACKING

#define MAVLINK_MSG_ID_HUCH_SIM_CTRL 222

typedef struct __mavlink_huch_sim_ctrl_t
{
 uint64_t usec; ///< timestamp in microseconds
 float arg; ///< command argument
 int16_t cmd; ///< command
} mavlink_huch_sim_ctrl_t;

#define MAVLINK_MSG_ID_HUCH_SIM_CTRL_LEN 14
#define MAVLINK_MSG_ID_222_LEN 14



#define MAVLINK_MESSAGE_INFO_HUCH_SIM_CTRL { \
	"HUCH_SIM_CTRL", \
	3, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_sim_ctrl_t, usec) }, \
         { "arg", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_huch_sim_ctrl_t, arg) }, \
         { "cmd", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_huch_sim_ctrl_t, cmd) }, \
         } \
}


/**
 * @brief Pack a huch_sim_ctrl message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec timestamp in microseconds
 * @param cmd command
 * @param arg command argument
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_sim_ctrl_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, int16_t cmd, float arg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, arg);
	_mav_put_int16_t(buf, 12, cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 14);
#else
	mavlink_huch_sim_ctrl_t packet;
	packet.usec = usec;
	packet.arg = arg;
	packet.cmd = cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 14);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_SIM_CTRL;
	return mavlink_finalize_message(msg, system_id, component_id, 14, 69);
}

/**
 * @brief Pack a huch_sim_ctrl message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec timestamp in microseconds
 * @param cmd command
 * @param arg command argument
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_sim_ctrl_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,int16_t cmd,float arg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, arg);
	_mav_put_int16_t(buf, 12, cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 14);
#else
	mavlink_huch_sim_ctrl_t packet;
	packet.usec = usec;
	packet.arg = arg;
	packet.cmd = cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 14);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_SIM_CTRL;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14, 69);
}

/**
 * @brief Encode a huch_sim_ctrl struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_sim_ctrl C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_sim_ctrl_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_sim_ctrl_t* huch_sim_ctrl)
{
	return mavlink_msg_huch_sim_ctrl_pack(system_id, component_id, msg, huch_sim_ctrl->usec, huch_sim_ctrl->cmd, huch_sim_ctrl->arg);
}

/**
 * @brief Send a huch_sim_ctrl message
 * @param chan MAVLink channel to send the message
 *
 * @param usec timestamp in microseconds
 * @param cmd command
 * @param arg command argument
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_sim_ctrl_send(mavlink_channel_t chan, uint64_t usec, int16_t cmd, float arg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, arg);
	_mav_put_int16_t(buf, 12, cmd);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_SIM_CTRL, buf, 14, 69);
#else
	mavlink_huch_sim_ctrl_t packet;
	packet.usec = usec;
	packet.arg = arg;
	packet.cmd = cmd;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_SIM_CTRL, (const char *)&packet, 14, 69);
#endif
}

#endif

// MESSAGE HUCH_SIM_CTRL UNPACKING


/**
 * @brief Get field usec from huch_sim_ctrl message
 *
 * @return timestamp in microseconds
 */
static inline uint64_t mavlink_msg_huch_sim_ctrl_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field cmd from huch_sim_ctrl message
 *
 * @return command
 */
static inline int16_t mavlink_msg_huch_sim_ctrl_get_cmd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field arg from huch_sim_ctrl message
 *
 * @return command argument
 */
static inline float mavlink_msg_huch_sim_ctrl_get_arg(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a huch_sim_ctrl message into a struct
 *
 * @param msg The message to decode
 * @param huch_sim_ctrl C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_sim_ctrl_decode(const mavlink_message_t* msg, mavlink_huch_sim_ctrl_t* huch_sim_ctrl)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_sim_ctrl->usec = mavlink_msg_huch_sim_ctrl_get_usec(msg);
	huch_sim_ctrl->arg = mavlink_msg_huch_sim_ctrl_get_arg(msg);
	huch_sim_ctrl->cmd = mavlink_msg_huch_sim_ctrl_get_cmd(msg);
#else
	memcpy(huch_sim_ctrl, _MAV_PAYLOAD(msg), 14);
#endif
}
