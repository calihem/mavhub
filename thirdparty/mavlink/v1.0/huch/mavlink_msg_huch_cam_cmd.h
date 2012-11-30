// MESSAGE HUCH_CAM_CMD PACKING

#define MAVLINK_MSG_ID_HUCH_CAM_CMD 232

typedef struct __mavlink_huch_cam_cmd_t
{
 uint8_t cam_index; ///< The camera index
 uint8_t exposure; ///< Desired camera exposure setting.
 uint8_t contrast; ///< Desired camera contrast setting.
 uint8_t gain; ///< Desired camera gain setting.
} mavlink_huch_cam_cmd_t;

#define MAVLINK_MSG_ID_HUCH_CAM_CMD_LEN 4
#define MAVLINK_MSG_ID_232_LEN 4



#define MAVLINK_MESSAGE_INFO_HUCH_CAM_CMD { \
	"HUCH_CAM_CMD", \
	4, \
	{  { "cam_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_huch_cam_cmd_t, cam_index) }, \
         { "exposure", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_huch_cam_cmd_t, exposure) }, \
         { "contrast", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_huch_cam_cmd_t, contrast) }, \
         { "gain", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_huch_cam_cmd_t, gain) }, \
         } \
}


/**
 * @brief Pack a huch_cam_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cam_index The camera index
 * @param exposure Desired camera exposure setting.
 * @param contrast Desired camera contrast setting.
 * @param gain Desired camera gain setting.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_cam_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t cam_index, uint8_t exposure, uint8_t contrast, uint8_t gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint8_t(buf, 0, cam_index);
	_mav_put_uint8_t(buf, 1, exposure);
	_mav_put_uint8_t(buf, 2, contrast);
	_mav_put_uint8_t(buf, 3, gain);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_huch_cam_cmd_t packet;
	packet.cam_index = cam_index;
	packet.exposure = exposure;
	packet.contrast = contrast;
	packet.gain = gain;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_CAM_CMD;
	return mavlink_finalize_message(msg, system_id, component_id, 4, 49);
}

/**
 * @brief Pack a huch_cam_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param cam_index The camera index
 * @param exposure Desired camera exposure setting.
 * @param contrast Desired camera contrast setting.
 * @param gain Desired camera gain setting.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_cam_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t cam_index,uint8_t exposure,uint8_t contrast,uint8_t gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint8_t(buf, 0, cam_index);
	_mav_put_uint8_t(buf, 1, exposure);
	_mav_put_uint8_t(buf, 2, contrast);
	_mav_put_uint8_t(buf, 3, gain);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_huch_cam_cmd_t packet;
	packet.cam_index = cam_index;
	packet.exposure = exposure;
	packet.contrast = contrast;
	packet.gain = gain;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_CAM_CMD;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 49);
}

/**
 * @brief Encode a huch_cam_cmd struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_cam_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_cam_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_cam_cmd_t* huch_cam_cmd)
{
	return mavlink_msg_huch_cam_cmd_pack(system_id, component_id, msg, huch_cam_cmd->cam_index, huch_cam_cmd->exposure, huch_cam_cmd->contrast, huch_cam_cmd->gain);
}

/**
 * @brief Send a huch_cam_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param cam_index The camera index
 * @param exposure Desired camera exposure setting.
 * @param contrast Desired camera contrast setting.
 * @param gain Desired camera gain setting.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_cam_cmd_send(mavlink_channel_t chan, uint8_t cam_index, uint8_t exposure, uint8_t contrast, uint8_t gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint8_t(buf, 0, cam_index);
	_mav_put_uint8_t(buf, 1, exposure);
	_mav_put_uint8_t(buf, 2, contrast);
	_mav_put_uint8_t(buf, 3, gain);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_CAM_CMD, buf, 4, 49);
#else
	mavlink_huch_cam_cmd_t packet;
	packet.cam_index = cam_index;
	packet.exposure = exposure;
	packet.contrast = contrast;
	packet.gain = gain;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_CAM_CMD, (const char *)&packet, 4, 49);
#endif
}

#endif

// MESSAGE HUCH_CAM_CMD UNPACKING


/**
 * @brief Get field cam_index from huch_cam_cmd message
 *
 * @return The camera index
 */
static inline uint8_t mavlink_msg_huch_cam_cmd_get_cam_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field exposure from huch_cam_cmd message
 *
 * @return Desired camera exposure setting.
 */
static inline uint8_t mavlink_msg_huch_cam_cmd_get_exposure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field contrast from huch_cam_cmd message
 *
 * @return Desired camera contrast setting.
 */
static inline uint8_t mavlink_msg_huch_cam_cmd_get_contrast(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field gain from huch_cam_cmd message
 *
 * @return Desired camera gain setting.
 */
static inline uint8_t mavlink_msg_huch_cam_cmd_get_gain(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a huch_cam_cmd message into a struct
 *
 * @param msg The message to decode
 * @param huch_cam_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_cam_cmd_decode(const mavlink_message_t* msg, mavlink_huch_cam_cmd_t* huch_cam_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_cam_cmd->cam_index = mavlink_msg_huch_cam_cmd_get_cam_index(msg);
	huch_cam_cmd->exposure = mavlink_msg_huch_cam_cmd_get_exposure(msg);
	huch_cam_cmd->contrast = mavlink_msg_huch_cam_cmd_get_contrast(msg);
	huch_cam_cmd->gain = mavlink_msg_huch_cam_cmd_get_gain(msg);
#else
	memcpy(huch_cam_cmd, _MAV_PAYLOAD(msg), 4);
#endif
}
