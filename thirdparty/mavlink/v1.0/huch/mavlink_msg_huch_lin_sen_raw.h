// MESSAGE HUCH_LIN_SEN_RAW PACKING

#define MAVLINK_MSG_ID_HUCH_LIN_SEN_RAW 224

typedef struct __mavlink_huch_lin_sen_raw_t
{
 uint32_t msec; ///< timestamp in milliseconds
 uint16_t data[96]; ///< data
} mavlink_huch_lin_sen_raw_t;

#define MAVLINK_MSG_ID_HUCH_LIN_SEN_RAW_LEN 196
#define MAVLINK_MSG_ID_224_LEN 196

#define MAVLINK_MSG_HUCH_LIN_SEN_RAW_FIELD_DATA_LEN 96

#define MAVLINK_MESSAGE_INFO_HUCH_LIN_SEN_RAW { \
	"HUCH_LIN_SEN_RAW", \
	2, \
	{  { "msec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_huch_lin_sen_raw_t, msec) }, \
         { "data", NULL, MAVLINK_TYPE_UINT16_T, 96, 4, offsetof(mavlink_huch_lin_sen_raw_t, data) }, \
         } \
}


/**
 * @brief Pack a huch_lin_sen_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param msec timestamp in milliseconds
 * @param data data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_lin_sen_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t msec, const uint16_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[196];
	_mav_put_uint32_t(buf, 0, msec);
	_mav_put_uint16_t_array(buf, 4, data, 96);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 196);
#else
	mavlink_huch_lin_sen_raw_t packet;
	packet.msec = msec;
	mav_array_memcpy(packet.data, data, sizeof(uint16_t)*96);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 196);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_LIN_SEN_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 196, 160);
}

/**
 * @brief Pack a huch_lin_sen_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param msec timestamp in milliseconds
 * @param data data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_lin_sen_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t msec,const uint16_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[196];
	_mav_put_uint32_t(buf, 0, msec);
	_mav_put_uint16_t_array(buf, 4, data, 96);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 196);
#else
	mavlink_huch_lin_sen_raw_t packet;
	packet.msec = msec;
	mav_array_memcpy(packet.data, data, sizeof(uint16_t)*96);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 196);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_LIN_SEN_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 196, 160);
}

/**
 * @brief Encode a huch_lin_sen_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_lin_sen_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_lin_sen_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_lin_sen_raw_t* huch_lin_sen_raw)
{
	return mavlink_msg_huch_lin_sen_raw_pack(system_id, component_id, msg, huch_lin_sen_raw->msec, huch_lin_sen_raw->data);
}

/**
 * @brief Send a huch_lin_sen_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param msec timestamp in milliseconds
 * @param data data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_lin_sen_raw_send(mavlink_channel_t chan, uint32_t msec, const uint16_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[196];
	_mav_put_uint32_t(buf, 0, msec);
	_mav_put_uint16_t_array(buf, 4, data, 96);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_LIN_SEN_RAW, buf, 196, 160);
#else
	mavlink_huch_lin_sen_raw_t packet;
	packet.msec = msec;
	mav_array_memcpy(packet.data, data, sizeof(uint16_t)*96);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_LIN_SEN_RAW, (const char *)&packet, 196, 160);
#endif
}

#endif

// MESSAGE HUCH_LIN_SEN_RAW UNPACKING


/**
 * @brief Get field msec from huch_lin_sen_raw message
 *
 * @return timestamp in milliseconds
 */
static inline uint32_t mavlink_msg_huch_lin_sen_raw_get_msec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field data from huch_lin_sen_raw message
 *
 * @return data
 */
static inline uint16_t mavlink_msg_huch_lin_sen_raw_get_data(const mavlink_message_t* msg, uint16_t *data)
{
	return _MAV_RETURN_uint16_t_array(msg, data, 96,  4);
}

/**
 * @brief Decode a huch_lin_sen_raw message into a struct
 *
 * @param msg The message to decode
 * @param huch_lin_sen_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_lin_sen_raw_decode(const mavlink_message_t* msg, mavlink_huch_lin_sen_raw_t* huch_lin_sen_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_lin_sen_raw->msec = mavlink_msg_huch_lin_sen_raw_get_msec(msg);
	mavlink_msg_huch_lin_sen_raw_get_data(msg, huch_lin_sen_raw->data);
#else
	memcpy(huch_lin_sen_raw, _MAV_PAYLOAD(msg), 196);
#endif
}
