// MESSAGE HUCH_CAM_STATE PACKING

#define MAVLINK_MSG_ID_HUCH_CAM_STATE 231

typedef struct __mavlink_huch_cam_state_t
{
 float hist1; ///< Eight bin histogram field 1.
 float hist2; ///< Eight bin histogram field 2.
 float hist3; ///< Eight bin histogram field 3.
 float hist4; ///< Eight bin histogram field 4.
 float hist5; ///< Eight bin histogram field 5.
 float hist6; ///< Eight bin histogram field 6.
 float hist7; ///< Eight bin histogram field 7.
 float hist8; ///< Eight bin histogram field 8.
 uint8_t cam_index; ///< The camera index
 uint8_t exposure; ///< Current camera exposure setting.
 uint8_t contrast; ///< Current camera contrast setting.
 uint8_t gain; ///< Current camera gain setting.
} mavlink_huch_cam_state_t;

#define MAVLINK_MSG_ID_HUCH_CAM_STATE_LEN 36
#define MAVLINK_MSG_ID_231_LEN 36



#define MAVLINK_MESSAGE_INFO_HUCH_CAM_STATE { \
	"HUCH_CAM_STATE", \
	12, \
	{  { "hist1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_huch_cam_state_t, hist1) }, \
         { "hist2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_huch_cam_state_t, hist2) }, \
         { "hist3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_huch_cam_state_t, hist3) }, \
         { "hist4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_huch_cam_state_t, hist4) }, \
         { "hist5", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_huch_cam_state_t, hist5) }, \
         { "hist6", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_huch_cam_state_t, hist6) }, \
         { "hist7", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_huch_cam_state_t, hist7) }, \
         { "hist8", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_huch_cam_state_t, hist8) }, \
         { "cam_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_huch_cam_state_t, cam_index) }, \
         { "exposure", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_huch_cam_state_t, exposure) }, \
         { "contrast", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_huch_cam_state_t, contrast) }, \
         { "gain", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_huch_cam_state_t, gain) }, \
         } \
}


/**
 * @brief Pack a huch_cam_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cam_index The camera index
 * @param exposure Current camera exposure setting.
 * @param contrast Current camera contrast setting.
 * @param gain Current camera gain setting.
 * @param hist1 Eight bin histogram field 1.
 * @param hist2 Eight bin histogram field 2.
 * @param hist3 Eight bin histogram field 3.
 * @param hist4 Eight bin histogram field 4.
 * @param hist5 Eight bin histogram field 5.
 * @param hist6 Eight bin histogram field 6.
 * @param hist7 Eight bin histogram field 7.
 * @param hist8 Eight bin histogram field 8.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_cam_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t cam_index, uint8_t exposure, uint8_t contrast, uint8_t gain, float hist1, float hist2, float hist3, float hist4, float hist5, float hist6, float hist7, float hist8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_float(buf, 0, hist1);
	_mav_put_float(buf, 4, hist2);
	_mav_put_float(buf, 8, hist3);
	_mav_put_float(buf, 12, hist4);
	_mav_put_float(buf, 16, hist5);
	_mav_put_float(buf, 20, hist6);
	_mav_put_float(buf, 24, hist7);
	_mav_put_float(buf, 28, hist8);
	_mav_put_uint8_t(buf, 32, cam_index);
	_mav_put_uint8_t(buf, 33, exposure);
	_mav_put_uint8_t(buf, 34, contrast);
	_mav_put_uint8_t(buf, 35, gain);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 36);
#else
	mavlink_huch_cam_state_t packet;
	packet.hist1 = hist1;
	packet.hist2 = hist2;
	packet.hist3 = hist3;
	packet.hist4 = hist4;
	packet.hist5 = hist5;
	packet.hist6 = hist6;
	packet.hist7 = hist7;
	packet.hist8 = hist8;
	packet.cam_index = cam_index;
	packet.exposure = exposure;
	packet.contrast = contrast;
	packet.gain = gain;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_CAM_STATE;
	return mavlink_finalize_message(msg, system_id, component_id, 36, 175);
}

/**
 * @brief Pack a huch_cam_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param cam_index The camera index
 * @param exposure Current camera exposure setting.
 * @param contrast Current camera contrast setting.
 * @param gain Current camera gain setting.
 * @param hist1 Eight bin histogram field 1.
 * @param hist2 Eight bin histogram field 2.
 * @param hist3 Eight bin histogram field 3.
 * @param hist4 Eight bin histogram field 4.
 * @param hist5 Eight bin histogram field 5.
 * @param hist6 Eight bin histogram field 6.
 * @param hist7 Eight bin histogram field 7.
 * @param hist8 Eight bin histogram field 8.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_cam_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t cam_index,uint8_t exposure,uint8_t contrast,uint8_t gain,float hist1,float hist2,float hist3,float hist4,float hist5,float hist6,float hist7,float hist8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_float(buf, 0, hist1);
	_mav_put_float(buf, 4, hist2);
	_mav_put_float(buf, 8, hist3);
	_mav_put_float(buf, 12, hist4);
	_mav_put_float(buf, 16, hist5);
	_mav_put_float(buf, 20, hist6);
	_mav_put_float(buf, 24, hist7);
	_mav_put_float(buf, 28, hist8);
	_mav_put_uint8_t(buf, 32, cam_index);
	_mav_put_uint8_t(buf, 33, exposure);
	_mav_put_uint8_t(buf, 34, contrast);
	_mav_put_uint8_t(buf, 35, gain);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 36);
#else
	mavlink_huch_cam_state_t packet;
	packet.hist1 = hist1;
	packet.hist2 = hist2;
	packet.hist3 = hist3;
	packet.hist4 = hist4;
	packet.hist5 = hist5;
	packet.hist6 = hist6;
	packet.hist7 = hist7;
	packet.hist8 = hist8;
	packet.cam_index = cam_index;
	packet.exposure = exposure;
	packet.contrast = contrast;
	packet.gain = gain;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_CAM_STATE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36, 175);
}

/**
 * @brief Encode a huch_cam_state struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_cam_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_cam_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_cam_state_t* huch_cam_state)
{
	return mavlink_msg_huch_cam_state_pack(system_id, component_id, msg, huch_cam_state->cam_index, huch_cam_state->exposure, huch_cam_state->contrast, huch_cam_state->gain, huch_cam_state->hist1, huch_cam_state->hist2, huch_cam_state->hist3, huch_cam_state->hist4, huch_cam_state->hist5, huch_cam_state->hist6, huch_cam_state->hist7, huch_cam_state->hist8);
}

/**
 * @brief Send a huch_cam_state message
 * @param chan MAVLink channel to send the message
 *
 * @param cam_index The camera index
 * @param exposure Current camera exposure setting.
 * @param contrast Current camera contrast setting.
 * @param gain Current camera gain setting.
 * @param hist1 Eight bin histogram field 1.
 * @param hist2 Eight bin histogram field 2.
 * @param hist3 Eight bin histogram field 3.
 * @param hist4 Eight bin histogram field 4.
 * @param hist5 Eight bin histogram field 5.
 * @param hist6 Eight bin histogram field 6.
 * @param hist7 Eight bin histogram field 7.
 * @param hist8 Eight bin histogram field 8.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_cam_state_send(mavlink_channel_t chan, uint8_t cam_index, uint8_t exposure, uint8_t contrast, uint8_t gain, float hist1, float hist2, float hist3, float hist4, float hist5, float hist6, float hist7, float hist8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_float(buf, 0, hist1);
	_mav_put_float(buf, 4, hist2);
	_mav_put_float(buf, 8, hist3);
	_mav_put_float(buf, 12, hist4);
	_mav_put_float(buf, 16, hist5);
	_mav_put_float(buf, 20, hist6);
	_mav_put_float(buf, 24, hist7);
	_mav_put_float(buf, 28, hist8);
	_mav_put_uint8_t(buf, 32, cam_index);
	_mav_put_uint8_t(buf, 33, exposure);
	_mav_put_uint8_t(buf, 34, contrast);
	_mav_put_uint8_t(buf, 35, gain);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_CAM_STATE, buf, 36, 175);
#else
	mavlink_huch_cam_state_t packet;
	packet.hist1 = hist1;
	packet.hist2 = hist2;
	packet.hist3 = hist3;
	packet.hist4 = hist4;
	packet.hist5 = hist5;
	packet.hist6 = hist6;
	packet.hist7 = hist7;
	packet.hist8 = hist8;
	packet.cam_index = cam_index;
	packet.exposure = exposure;
	packet.contrast = contrast;
	packet.gain = gain;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_CAM_STATE, (const char *)&packet, 36, 175);
#endif
}

#endif

// MESSAGE HUCH_CAM_STATE UNPACKING


/**
 * @brief Get field cam_index from huch_cam_state message
 *
 * @return The camera index
 */
static inline uint8_t mavlink_msg_huch_cam_state_get_cam_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field exposure from huch_cam_state message
 *
 * @return Current camera exposure setting.
 */
static inline uint8_t mavlink_msg_huch_cam_state_get_exposure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field contrast from huch_cam_state message
 *
 * @return Current camera contrast setting.
 */
static inline uint8_t mavlink_msg_huch_cam_state_get_contrast(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field gain from huch_cam_state message
 *
 * @return Current camera gain setting.
 */
static inline uint8_t mavlink_msg_huch_cam_state_get_gain(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field hist1 from huch_cam_state message
 *
 * @return Eight bin histogram field 1.
 */
static inline float mavlink_msg_huch_cam_state_get_hist1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field hist2 from huch_cam_state message
 *
 * @return Eight bin histogram field 2.
 */
static inline float mavlink_msg_huch_cam_state_get_hist2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field hist3 from huch_cam_state message
 *
 * @return Eight bin histogram field 3.
 */
static inline float mavlink_msg_huch_cam_state_get_hist3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field hist4 from huch_cam_state message
 *
 * @return Eight bin histogram field 4.
 */
static inline float mavlink_msg_huch_cam_state_get_hist4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field hist5 from huch_cam_state message
 *
 * @return Eight bin histogram field 5.
 */
static inline float mavlink_msg_huch_cam_state_get_hist5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field hist6 from huch_cam_state message
 *
 * @return Eight bin histogram field 6.
 */
static inline float mavlink_msg_huch_cam_state_get_hist6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field hist7 from huch_cam_state message
 *
 * @return Eight bin histogram field 7.
 */
static inline float mavlink_msg_huch_cam_state_get_hist7(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field hist8 from huch_cam_state message
 *
 * @return Eight bin histogram field 8.
 */
static inline float mavlink_msg_huch_cam_state_get_hist8(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a huch_cam_state message into a struct
 *
 * @param msg The message to decode
 * @param huch_cam_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_cam_state_decode(const mavlink_message_t* msg, mavlink_huch_cam_state_t* huch_cam_state)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_cam_state->hist1 = mavlink_msg_huch_cam_state_get_hist1(msg);
	huch_cam_state->hist2 = mavlink_msg_huch_cam_state_get_hist2(msg);
	huch_cam_state->hist3 = mavlink_msg_huch_cam_state_get_hist3(msg);
	huch_cam_state->hist4 = mavlink_msg_huch_cam_state_get_hist4(msg);
	huch_cam_state->hist5 = mavlink_msg_huch_cam_state_get_hist5(msg);
	huch_cam_state->hist6 = mavlink_msg_huch_cam_state_get_hist6(msg);
	huch_cam_state->hist7 = mavlink_msg_huch_cam_state_get_hist7(msg);
	huch_cam_state->hist8 = mavlink_msg_huch_cam_state_get_hist8(msg);
	huch_cam_state->cam_index = mavlink_msg_huch_cam_state_get_cam_index(msg);
	huch_cam_state->exposure = mavlink_msg_huch_cam_state_get_exposure(msg);
	huch_cam_state->contrast = mavlink_msg_huch_cam_state_get_contrast(msg);
	huch_cam_state->gain = mavlink_msg_huch_cam_state_get_gain(msg);
#else
	memcpy(huch_cam_state, _MAV_PAYLOAD(msg), 36);
#endif
}
