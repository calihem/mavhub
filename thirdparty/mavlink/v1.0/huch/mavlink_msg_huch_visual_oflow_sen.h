// MESSAGE HUCH_VISUAL_OFLOW_SEN PACKING

#define MAVLINK_MSG_ID_HUCH_VISUAL_OFLOW_SEN 233

typedef struct __mavlink_huch_visual_oflow_sen_t
{
 float u; ///< x flow (rel velocity)
 float v; ///< y flow (rel velocity)
 float squal; ///< surface quality, number of features reported by sensor
 uint8_t id; ///< x flow integral (rel pos)
} mavlink_huch_visual_oflow_sen_t;

#define MAVLINK_MSG_ID_HUCH_VISUAL_OFLOW_SEN_LEN 13
#define MAVLINK_MSG_ID_233_LEN 13



#define MAVLINK_MESSAGE_INFO_HUCH_VISUAL_OFLOW_SEN { \
	"HUCH_VISUAL_OFLOW_SEN", \
	4, \
	{  { "u", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_huch_visual_oflow_sen_t, u) }, \
         { "v", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_huch_visual_oflow_sen_t, v) }, \
         { "squal", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_huch_visual_oflow_sen_t, squal) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_huch_visual_oflow_sen_t, id) }, \
         } \
}


/**
 * @brief Pack a huch_visual_oflow_sen message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id x flow integral (rel pos)
 * @param u x flow (rel velocity)
 * @param v y flow (rel velocity)
 * @param squal surface quality, number of features reported by sensor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_visual_oflow_sen_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t id, float u, float v, float squal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[13];
	_mav_put_float(buf, 0, u);
	_mav_put_float(buf, 4, v);
	_mav_put_float(buf, 8, squal);
	_mav_put_uint8_t(buf, 12, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 13);
#else
	mavlink_huch_visual_oflow_sen_t packet;
	packet.u = u;
	packet.v = v;
	packet.squal = squal;
	packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 13);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_VISUAL_OFLOW_SEN;
	return mavlink_finalize_message(msg, system_id, component_id, 13, 86);
}

/**
 * @brief Pack a huch_visual_oflow_sen message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param id x flow integral (rel pos)
 * @param u x flow (rel velocity)
 * @param v y flow (rel velocity)
 * @param squal surface quality, number of features reported by sensor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_visual_oflow_sen_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t id,float u,float v,float squal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[13];
	_mav_put_float(buf, 0, u);
	_mav_put_float(buf, 4, v);
	_mav_put_float(buf, 8, squal);
	_mav_put_uint8_t(buf, 12, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 13);
#else
	mavlink_huch_visual_oflow_sen_t packet;
	packet.u = u;
	packet.v = v;
	packet.squal = squal;
	packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 13);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_VISUAL_OFLOW_SEN;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 13, 86);
}

/**
 * @brief Encode a huch_visual_oflow_sen struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_visual_oflow_sen C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_visual_oflow_sen_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_visual_oflow_sen_t* huch_visual_oflow_sen)
{
	return mavlink_msg_huch_visual_oflow_sen_pack(system_id, component_id, msg, huch_visual_oflow_sen->id, huch_visual_oflow_sen->u, huch_visual_oflow_sen->v, huch_visual_oflow_sen->squal);
}

/**
 * @brief Send a huch_visual_oflow_sen message
 * @param chan MAVLink channel to send the message
 *
 * @param id x flow integral (rel pos)
 * @param u x flow (rel velocity)
 * @param v y flow (rel velocity)
 * @param squal surface quality, number of features reported by sensor
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_visual_oflow_sen_send(mavlink_channel_t chan, uint8_t id, float u, float v, float squal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[13];
	_mav_put_float(buf, 0, u);
	_mav_put_float(buf, 4, v);
	_mav_put_float(buf, 8, squal);
	_mav_put_uint8_t(buf, 12, id);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_VISUAL_OFLOW_SEN, buf, 13, 86);
#else
	mavlink_huch_visual_oflow_sen_t packet;
	packet.u = u;
	packet.v = v;
	packet.squal = squal;
	packet.id = id;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_VISUAL_OFLOW_SEN, (const char *)&packet, 13, 86);
#endif
}

#endif

// MESSAGE HUCH_VISUAL_OFLOW_SEN UNPACKING


/**
 * @brief Get field id from huch_visual_oflow_sen message
 *
 * @return x flow integral (rel pos)
 */
static inline uint8_t mavlink_msg_huch_visual_oflow_sen_get_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field u from huch_visual_oflow_sen message
 *
 * @return x flow (rel velocity)
 */
static inline float mavlink_msg_huch_visual_oflow_sen_get_u(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field v from huch_visual_oflow_sen message
 *
 * @return y flow (rel velocity)
 */
static inline float mavlink_msg_huch_visual_oflow_sen_get_v(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field squal from huch_visual_oflow_sen message
 *
 * @return surface quality, number of features reported by sensor
 */
static inline float mavlink_msg_huch_visual_oflow_sen_get_squal(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a huch_visual_oflow_sen message into a struct
 *
 * @param msg The message to decode
 * @param huch_visual_oflow_sen C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_visual_oflow_sen_decode(const mavlink_message_t* msg, mavlink_huch_visual_oflow_sen_t* huch_visual_oflow_sen)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_visual_oflow_sen->u = mavlink_msg_huch_visual_oflow_sen_get_u(msg);
	huch_visual_oflow_sen->v = mavlink_msg_huch_visual_oflow_sen_get_v(msg);
	huch_visual_oflow_sen->squal = mavlink_msg_huch_visual_oflow_sen_get_squal(msg);
	huch_visual_oflow_sen->id = mavlink_msg_huch_visual_oflow_sen_get_id(msg);
#else
	memcpy(huch_visual_oflow_sen, _MAV_PAYLOAD(msg), 13);
#endif
}
