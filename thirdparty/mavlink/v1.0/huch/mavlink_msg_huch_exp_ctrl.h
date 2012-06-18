// MESSAGE HUCH_EXP_CTRL PACKING

#define MAVLINK_MSG_ID_HUCH_EXP_CTRL 202

typedef struct __mavlink_huch_exp_ctrl_t
{
 uint8_t status; ///< 
 int8_t rx[9]; ///< 
 int8_t tx[2]; ///< 
} mavlink_huch_exp_ctrl_t;

#define MAVLINK_MSG_ID_HUCH_EXP_CTRL_LEN 12
#define MAVLINK_MSG_ID_202_LEN 12

#define MAVLINK_MSG_HUCH_EXP_CTRL_FIELD_RX_LEN 9
#define MAVLINK_MSG_HUCH_EXP_CTRL_FIELD_TX_LEN 2

#define MAVLINK_MESSAGE_INFO_HUCH_EXP_CTRL { \
	"HUCH_EXP_CTRL", \
	3, \
	{  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_huch_exp_ctrl_t, status) }, \
         { "rx", NULL, MAVLINK_TYPE_INT8_T, 9, 1, offsetof(mavlink_huch_exp_ctrl_t, rx) }, \
         { "tx", NULL, MAVLINK_TYPE_INT8_T, 2, 10, offsetof(mavlink_huch_exp_ctrl_t, tx) }, \
         } \
}


/**
 * @brief Pack a huch_exp_ctrl message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status 
 * @param rx 
 * @param tx 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_exp_ctrl_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t status, const int8_t *rx, const int8_t *tx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint8_t(buf, 0, status);
	_mav_put_int8_t_array(buf, 1, rx, 9);
	_mav_put_int8_t_array(buf, 10, tx, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_exp_ctrl_t packet;
	packet.status = status;
	mav_array_memcpy(packet.rx, rx, sizeof(int8_t)*9);
	mav_array_memcpy(packet.tx, tx, sizeof(int8_t)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_EXP_CTRL;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 197);
}

/**
 * @brief Pack a huch_exp_ctrl message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param status 
 * @param rx 
 * @param tx 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_exp_ctrl_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t status,const int8_t *rx,const int8_t *tx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint8_t(buf, 0, status);
	_mav_put_int8_t_array(buf, 1, rx, 9);
	_mav_put_int8_t_array(buf, 10, tx, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_exp_ctrl_t packet;
	packet.status = status;
	mav_array_memcpy(packet.rx, rx, sizeof(int8_t)*9);
	mav_array_memcpy(packet.tx, tx, sizeof(int8_t)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_EXP_CTRL;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 197);
}

/**
 * @brief Encode a huch_exp_ctrl struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_exp_ctrl C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_exp_ctrl_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_exp_ctrl_t* huch_exp_ctrl)
{
	return mavlink_msg_huch_exp_ctrl_pack(system_id, component_id, msg, huch_exp_ctrl->status, huch_exp_ctrl->rx, huch_exp_ctrl->tx);
}

/**
 * @brief Send a huch_exp_ctrl message
 * @param chan MAVLink channel to send the message
 *
 * @param status 
 * @param rx 
 * @param tx 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_exp_ctrl_send(mavlink_channel_t chan, uint8_t status, const int8_t *rx, const int8_t *tx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint8_t(buf, 0, status);
	_mav_put_int8_t_array(buf, 1, rx, 9);
	_mav_put_int8_t_array(buf, 10, tx, 2);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_EXP_CTRL, buf, 12, 197);
#else
	mavlink_huch_exp_ctrl_t packet;
	packet.status = status;
	mav_array_memcpy(packet.rx, rx, sizeof(int8_t)*9);
	mav_array_memcpy(packet.tx, tx, sizeof(int8_t)*2);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_EXP_CTRL, (const char *)&packet, 12, 197);
#endif
}

#endif

// MESSAGE HUCH_EXP_CTRL UNPACKING


/**
 * @brief Get field status from huch_exp_ctrl message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_huch_exp_ctrl_get_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field rx from huch_exp_ctrl message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_huch_exp_ctrl_get_rx(const mavlink_message_t* msg, int8_t *rx)
{
	return _MAV_RETURN_int8_t_array(msg, rx, 9,  1);
}

/**
 * @brief Get field tx from huch_exp_ctrl message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_huch_exp_ctrl_get_tx(const mavlink_message_t* msg, int8_t *tx)
{
	return _MAV_RETURN_int8_t_array(msg, tx, 2,  10);
}

/**
 * @brief Decode a huch_exp_ctrl message into a struct
 *
 * @param msg The message to decode
 * @param huch_exp_ctrl C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_exp_ctrl_decode(const mavlink_message_t* msg, mavlink_huch_exp_ctrl_t* huch_exp_ctrl)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_exp_ctrl->status = mavlink_msg_huch_exp_ctrl_get_status(msg);
	mavlink_msg_huch_exp_ctrl_get_rx(msg, huch_exp_ctrl->rx);
	mavlink_msg_huch_exp_ctrl_get_tx(msg, huch_exp_ctrl->tx);
#else
	memcpy(huch_exp_ctrl, _MAV_PAYLOAD(msg), 12);
#endif
}
