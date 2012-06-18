// MESSAGE HUCH_EXP_CTRL_RX PACKING

#define MAVLINK_MSG_ID_HUCH_EXP_CTRL_RX 207

typedef struct __mavlink_huch_exp_ctrl_rx_t
{
 uint16_t value0; ///< 
 uint16_t value1; ///< 
 uint16_t value2; ///< 
 uint16_t value3; ///< 
 uint8_t version; ///< 
} mavlink_huch_exp_ctrl_rx_t;

#define MAVLINK_MSG_ID_HUCH_EXP_CTRL_RX_LEN 9
#define MAVLINK_MSG_ID_207_LEN 9



#define MAVLINK_MESSAGE_INFO_HUCH_EXP_CTRL_RX { \
	"HUCH_EXP_CTRL_RX", \
	5, \
	{  { "value0", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_huch_exp_ctrl_rx_t, value0) }, \
         { "value1", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_huch_exp_ctrl_rx_t, value1) }, \
         { "value2", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_huch_exp_ctrl_rx_t, value2) }, \
         { "value3", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_huch_exp_ctrl_rx_t, value3) }, \
         { "version", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_huch_exp_ctrl_rx_t, version) }, \
         } \
}


/**
 * @brief Pack a huch_exp_ctrl_rx message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param version 
 * @param value0 
 * @param value1 
 * @param value2 
 * @param value3 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_exp_ctrl_rx_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t version, uint16_t value0, uint16_t value1, uint16_t value2, uint16_t value3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_uint16_t(buf, 0, value0);
	_mav_put_uint16_t(buf, 2, value1);
	_mav_put_uint16_t(buf, 4, value2);
	_mav_put_uint16_t(buf, 6, value3);
	_mav_put_uint8_t(buf, 8, version);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 9);
#else
	mavlink_huch_exp_ctrl_rx_t packet;
	packet.value0 = value0;
	packet.value1 = value1;
	packet.value2 = value2;
	packet.value3 = value3;
	packet.version = version;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 9);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_EXP_CTRL_RX;
	return mavlink_finalize_message(msg, system_id, component_id, 9, 126);
}

/**
 * @brief Pack a huch_exp_ctrl_rx message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param version 
 * @param value0 
 * @param value1 
 * @param value2 
 * @param value3 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_exp_ctrl_rx_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t version,uint16_t value0,uint16_t value1,uint16_t value2,uint16_t value3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_uint16_t(buf, 0, value0);
	_mav_put_uint16_t(buf, 2, value1);
	_mav_put_uint16_t(buf, 4, value2);
	_mav_put_uint16_t(buf, 6, value3);
	_mav_put_uint8_t(buf, 8, version);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 9);
#else
	mavlink_huch_exp_ctrl_rx_t packet;
	packet.value0 = value0;
	packet.value1 = value1;
	packet.value2 = value2;
	packet.value3 = value3;
	packet.version = version;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 9);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_EXP_CTRL_RX;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 9, 126);
}

/**
 * @brief Encode a huch_exp_ctrl_rx struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_exp_ctrl_rx C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_exp_ctrl_rx_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_exp_ctrl_rx_t* huch_exp_ctrl_rx)
{
	return mavlink_msg_huch_exp_ctrl_rx_pack(system_id, component_id, msg, huch_exp_ctrl_rx->version, huch_exp_ctrl_rx->value0, huch_exp_ctrl_rx->value1, huch_exp_ctrl_rx->value2, huch_exp_ctrl_rx->value3);
}

/**
 * @brief Send a huch_exp_ctrl_rx message
 * @param chan MAVLink channel to send the message
 *
 * @param version 
 * @param value0 
 * @param value1 
 * @param value2 
 * @param value3 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_exp_ctrl_rx_send(mavlink_channel_t chan, uint8_t version, uint16_t value0, uint16_t value1, uint16_t value2, uint16_t value3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_uint16_t(buf, 0, value0);
	_mav_put_uint16_t(buf, 2, value1);
	_mav_put_uint16_t(buf, 4, value2);
	_mav_put_uint16_t(buf, 6, value3);
	_mav_put_uint8_t(buf, 8, version);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_EXP_CTRL_RX, buf, 9, 126);
#else
	mavlink_huch_exp_ctrl_rx_t packet;
	packet.value0 = value0;
	packet.value1 = value1;
	packet.value2 = value2;
	packet.value3 = value3;
	packet.version = version;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_EXP_CTRL_RX, (const char *)&packet, 9, 126);
#endif
}

#endif

// MESSAGE HUCH_EXP_CTRL_RX UNPACKING


/**
 * @brief Get field version from huch_exp_ctrl_rx message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_huch_exp_ctrl_rx_get_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field value0 from huch_exp_ctrl_rx message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_huch_exp_ctrl_rx_get_value0(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field value1 from huch_exp_ctrl_rx message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_huch_exp_ctrl_rx_get_value1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field value2 from huch_exp_ctrl_rx message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_huch_exp_ctrl_rx_get_value2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field value3 from huch_exp_ctrl_rx message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_huch_exp_ctrl_rx_get_value3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Decode a huch_exp_ctrl_rx message into a struct
 *
 * @param msg The message to decode
 * @param huch_exp_ctrl_rx C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_exp_ctrl_rx_decode(const mavlink_message_t* msg, mavlink_huch_exp_ctrl_rx_t* huch_exp_ctrl_rx)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_exp_ctrl_rx->value0 = mavlink_msg_huch_exp_ctrl_rx_get_value0(msg);
	huch_exp_ctrl_rx->value1 = mavlink_msg_huch_exp_ctrl_rx_get_value1(msg);
	huch_exp_ctrl_rx->value2 = mavlink_msg_huch_exp_ctrl_rx_get_value2(msg);
	huch_exp_ctrl_rx->value3 = mavlink_msg_huch_exp_ctrl_rx_get_value3(msg);
	huch_exp_ctrl_rx->version = mavlink_msg_huch_exp_ctrl_rx_get_version(msg);
#else
	memcpy(huch_exp_ctrl_rx, _MAV_PAYLOAD(msg), 9);
#endif
}
