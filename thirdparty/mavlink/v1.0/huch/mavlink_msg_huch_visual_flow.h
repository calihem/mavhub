// MESSAGE HUCH_VISUAL_FLOW PACKING

#define MAVLINK_MSG_ID_HUCH_VISUAL_FLOW 226

typedef struct __mavlink_huch_visual_flow_t
{
 uint64_t usec; ///< timestamp in microseconds
 float u; ///< x flow (rel velocity)
 float v; ///< y flow (rel velocity)
 float u_i; ///< x flow integral (rel pos)
 float v_i; ///< y flow integral (rel pos)
} mavlink_huch_visual_flow_t;

#define MAVLINK_MSG_ID_HUCH_VISUAL_FLOW_LEN 24
#define MAVLINK_MSG_ID_226_LEN 24



#define MAVLINK_MESSAGE_INFO_HUCH_VISUAL_FLOW { \
	"HUCH_VISUAL_FLOW", \
	5, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_visual_flow_t, usec) }, \
         { "u", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_huch_visual_flow_t, u) }, \
         { "v", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_huch_visual_flow_t, v) }, \
         { "u_i", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_huch_visual_flow_t, u_i) }, \
         { "v_i", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_huch_visual_flow_t, v_i) }, \
         } \
}


/**
 * @brief Pack a huch_visual_flow message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec timestamp in microseconds
 * @param u x flow (rel velocity)
 * @param v y flow (rel velocity)
 * @param u_i x flow integral (rel pos)
 * @param v_i y flow integral (rel pos)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_visual_flow_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, float u, float v, float u_i, float v_i)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, u);
	_mav_put_float(buf, 12, v);
	_mav_put_float(buf, 16, u_i);
	_mav_put_float(buf, 20, v_i);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 24);
#else
	mavlink_huch_visual_flow_t packet;
	packet.usec = usec;
	packet.u = u;
	packet.v = v;
	packet.u_i = u_i;
	packet.v_i = v_i;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 24);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_VISUAL_FLOW;
	return mavlink_finalize_message(msg, system_id, component_id, 24, 118);
}

/**
 * @brief Pack a huch_visual_flow message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec timestamp in microseconds
 * @param u x flow (rel velocity)
 * @param v y flow (rel velocity)
 * @param u_i x flow integral (rel pos)
 * @param v_i y flow integral (rel pos)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_visual_flow_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,float u,float v,float u_i,float v_i)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, u);
	_mav_put_float(buf, 12, v);
	_mav_put_float(buf, 16, u_i);
	_mav_put_float(buf, 20, v_i);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 24);
#else
	mavlink_huch_visual_flow_t packet;
	packet.usec = usec;
	packet.u = u;
	packet.v = v;
	packet.u_i = u_i;
	packet.v_i = v_i;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 24);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_VISUAL_FLOW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 24, 118);
}

/**
 * @brief Encode a huch_visual_flow struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_visual_flow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_visual_flow_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_visual_flow_t* huch_visual_flow)
{
	return mavlink_msg_huch_visual_flow_pack(system_id, component_id, msg, huch_visual_flow->usec, huch_visual_flow->u, huch_visual_flow->v, huch_visual_flow->u_i, huch_visual_flow->v_i);
}

/**
 * @brief Send a huch_visual_flow message
 * @param chan MAVLink channel to send the message
 *
 * @param usec timestamp in microseconds
 * @param u x flow (rel velocity)
 * @param v y flow (rel velocity)
 * @param u_i x flow integral (rel pos)
 * @param v_i y flow integral (rel pos)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_visual_flow_send(mavlink_channel_t chan, uint64_t usec, float u, float v, float u_i, float v_i)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, u);
	_mav_put_float(buf, 12, v);
	_mav_put_float(buf, 16, u_i);
	_mav_put_float(buf, 20, v_i);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_VISUAL_FLOW, buf, 24, 118);
#else
	mavlink_huch_visual_flow_t packet;
	packet.usec = usec;
	packet.u = u;
	packet.v = v;
	packet.u_i = u_i;
	packet.v_i = v_i;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_VISUAL_FLOW, (const char *)&packet, 24, 118);
#endif
}

#endif

// MESSAGE HUCH_VISUAL_FLOW UNPACKING


/**
 * @brief Get field usec from huch_visual_flow message
 *
 * @return timestamp in microseconds
 */
static inline uint64_t mavlink_msg_huch_visual_flow_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field u from huch_visual_flow message
 *
 * @return x flow (rel velocity)
 */
static inline float mavlink_msg_huch_visual_flow_get_u(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field v from huch_visual_flow message
 *
 * @return y flow (rel velocity)
 */
static inline float mavlink_msg_huch_visual_flow_get_v(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field u_i from huch_visual_flow message
 *
 * @return x flow integral (rel pos)
 */
static inline float mavlink_msg_huch_visual_flow_get_u_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field v_i from huch_visual_flow message
 *
 * @return y flow integral (rel pos)
 */
static inline float mavlink_msg_huch_visual_flow_get_v_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a huch_visual_flow message into a struct
 *
 * @param msg The message to decode
 * @param huch_visual_flow C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_visual_flow_decode(const mavlink_message_t* msg, mavlink_huch_visual_flow_t* huch_visual_flow)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_visual_flow->usec = mavlink_msg_huch_visual_flow_get_usec(msg);
	huch_visual_flow->u = mavlink_msg_huch_visual_flow_get_u(msg);
	huch_visual_flow->v = mavlink_msg_huch_visual_flow_get_v(msg);
	huch_visual_flow->u_i = mavlink_msg_huch_visual_flow_get_u_i(msg);
	huch_visual_flow->v_i = mavlink_msg_huch_visual_flow_get_v_i(msg);
#else
	memcpy(huch_visual_flow, _MAV_PAYLOAD(msg), 24);
#endif
}
