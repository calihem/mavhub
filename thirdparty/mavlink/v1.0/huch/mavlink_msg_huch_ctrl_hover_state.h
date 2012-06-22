// MESSAGE HUCH_CTRL_HOVER_STATE PACKING

#define MAVLINK_MSG_ID_HUCH_CTRL_HOVER_STATE 210

typedef struct __mavlink_huch_ctrl_hover_state_t
{
 float uss; ///< Ultrasonic range measurement in mm
 float baro; ///< Barometric measurement in mm
 float accz; ///< Accelerometer measurement in mm/s^2
 float ir1; ///< Infrared range measurement in mm
 float ir2; ///< Infrared range measurement in mm
 float kal_s0; ///< Kalman state component 0 (pos)
 float kal_s1; ///< Kalman state component 1 (vel)
 float kal_s2; ///< Kalman state component 2 (acc)
} mavlink_huch_ctrl_hover_state_t;

#define MAVLINK_MSG_ID_HUCH_CTRL_HOVER_STATE_LEN 32
#define MAVLINK_MSG_ID_210_LEN 32



#define MAVLINK_MESSAGE_INFO_HUCH_CTRL_HOVER_STATE { \
	"HUCH_CTRL_HOVER_STATE", \
	8, \
	{  { "uss", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_huch_ctrl_hover_state_t, uss) }, \
         { "baro", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_huch_ctrl_hover_state_t, baro) }, \
         { "accz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_huch_ctrl_hover_state_t, accz) }, \
         { "ir1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_huch_ctrl_hover_state_t, ir1) }, \
         { "ir2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_huch_ctrl_hover_state_t, ir2) }, \
         { "kal_s0", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_huch_ctrl_hover_state_t, kal_s0) }, \
         { "kal_s1", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_huch_ctrl_hover_state_t, kal_s1) }, \
         { "kal_s2", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_huch_ctrl_hover_state_t, kal_s2) }, \
         } \
}


/**
 * @brief Pack a huch_ctrl_hover_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param uss Ultrasonic range measurement in mm
 * @param baro Barometric measurement in mm
 * @param accz Accelerometer measurement in mm/s^2
 * @param ir1 Infrared range measurement in mm
 * @param ir2 Infrared range measurement in mm
 * @param kal_s0 Kalman state component 0 (pos)
 * @param kal_s1 Kalman state component 1 (vel)
 * @param kal_s2 Kalman state component 2 (acc)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_ctrl_hover_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float uss, float baro, float accz, float ir1, float ir2, float kal_s0, float kal_s1, float kal_s2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[32];
	_mav_put_float(buf, 0, uss);
	_mav_put_float(buf, 4, baro);
	_mav_put_float(buf, 8, accz);
	_mav_put_float(buf, 12, ir1);
	_mav_put_float(buf, 16, ir2);
	_mav_put_float(buf, 20, kal_s0);
	_mav_put_float(buf, 24, kal_s1);
	_mav_put_float(buf, 28, kal_s2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 32);
#else
	mavlink_huch_ctrl_hover_state_t packet;
	packet.uss = uss;
	packet.baro = baro;
	packet.accz = accz;
	packet.ir1 = ir1;
	packet.ir2 = ir2;
	packet.kal_s0 = kal_s0;
	packet.kal_s1 = kal_s1;
	packet.kal_s2 = kal_s2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 32);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_CTRL_HOVER_STATE;
	return mavlink_finalize_message(msg, system_id, component_id, 32, 30);
}

/**
 * @brief Pack a huch_ctrl_hover_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param uss Ultrasonic range measurement in mm
 * @param baro Barometric measurement in mm
 * @param accz Accelerometer measurement in mm/s^2
 * @param ir1 Infrared range measurement in mm
 * @param ir2 Infrared range measurement in mm
 * @param kal_s0 Kalman state component 0 (pos)
 * @param kal_s1 Kalman state component 1 (vel)
 * @param kal_s2 Kalman state component 2 (acc)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_ctrl_hover_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float uss,float baro,float accz,float ir1,float ir2,float kal_s0,float kal_s1,float kal_s2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[32];
	_mav_put_float(buf, 0, uss);
	_mav_put_float(buf, 4, baro);
	_mav_put_float(buf, 8, accz);
	_mav_put_float(buf, 12, ir1);
	_mav_put_float(buf, 16, ir2);
	_mav_put_float(buf, 20, kal_s0);
	_mav_put_float(buf, 24, kal_s1);
	_mav_put_float(buf, 28, kal_s2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 32);
#else
	mavlink_huch_ctrl_hover_state_t packet;
	packet.uss = uss;
	packet.baro = baro;
	packet.accz = accz;
	packet.ir1 = ir1;
	packet.ir2 = ir2;
	packet.kal_s0 = kal_s0;
	packet.kal_s1 = kal_s1;
	packet.kal_s2 = kal_s2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 32);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_CTRL_HOVER_STATE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32, 30);
}

/**
 * @brief Encode a huch_ctrl_hover_state struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_ctrl_hover_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_ctrl_hover_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_ctrl_hover_state_t* huch_ctrl_hover_state)
{
	return mavlink_msg_huch_ctrl_hover_state_pack(system_id, component_id, msg, huch_ctrl_hover_state->uss, huch_ctrl_hover_state->baro, huch_ctrl_hover_state->accz, huch_ctrl_hover_state->ir1, huch_ctrl_hover_state->ir2, huch_ctrl_hover_state->kal_s0, huch_ctrl_hover_state->kal_s1, huch_ctrl_hover_state->kal_s2);
}

/**
 * @brief Send a huch_ctrl_hover_state message
 * @param chan MAVLink channel to send the message
 *
 * @param uss Ultrasonic range measurement in mm
 * @param baro Barometric measurement in mm
 * @param accz Accelerometer measurement in mm/s^2
 * @param ir1 Infrared range measurement in mm
 * @param ir2 Infrared range measurement in mm
 * @param kal_s0 Kalman state component 0 (pos)
 * @param kal_s1 Kalman state component 1 (vel)
 * @param kal_s2 Kalman state component 2 (acc)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_ctrl_hover_state_send(mavlink_channel_t chan, float uss, float baro, float accz, float ir1, float ir2, float kal_s0, float kal_s1, float kal_s2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[32];
	_mav_put_float(buf, 0, uss);
	_mav_put_float(buf, 4, baro);
	_mav_put_float(buf, 8, accz);
	_mav_put_float(buf, 12, ir1);
	_mav_put_float(buf, 16, ir2);
	_mav_put_float(buf, 20, kal_s0);
	_mav_put_float(buf, 24, kal_s1);
	_mav_put_float(buf, 28, kal_s2);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_CTRL_HOVER_STATE, buf, 32, 30);
#else
	mavlink_huch_ctrl_hover_state_t packet;
	packet.uss = uss;
	packet.baro = baro;
	packet.accz = accz;
	packet.ir1 = ir1;
	packet.ir2 = ir2;
	packet.kal_s0 = kal_s0;
	packet.kal_s1 = kal_s1;
	packet.kal_s2 = kal_s2;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_CTRL_HOVER_STATE, (const char *)&packet, 32, 30);
#endif
}

#endif

// MESSAGE HUCH_CTRL_HOVER_STATE UNPACKING


/**
 * @brief Get field uss from huch_ctrl_hover_state message
 *
 * @return Ultrasonic range measurement in mm
 */
static inline float mavlink_msg_huch_ctrl_hover_state_get_uss(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field baro from huch_ctrl_hover_state message
 *
 * @return Barometric measurement in mm
 */
static inline float mavlink_msg_huch_ctrl_hover_state_get_baro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field accz from huch_ctrl_hover_state message
 *
 * @return Accelerometer measurement in mm/s^2
 */
static inline float mavlink_msg_huch_ctrl_hover_state_get_accz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field ir1 from huch_ctrl_hover_state message
 *
 * @return Infrared range measurement in mm
 */
static inline float mavlink_msg_huch_ctrl_hover_state_get_ir1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field ir2 from huch_ctrl_hover_state message
 *
 * @return Infrared range measurement in mm
 */
static inline float mavlink_msg_huch_ctrl_hover_state_get_ir2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field kal_s0 from huch_ctrl_hover_state message
 *
 * @return Kalman state component 0 (pos)
 */
static inline float mavlink_msg_huch_ctrl_hover_state_get_kal_s0(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field kal_s1 from huch_ctrl_hover_state message
 *
 * @return Kalman state component 1 (vel)
 */
static inline float mavlink_msg_huch_ctrl_hover_state_get_kal_s1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field kal_s2 from huch_ctrl_hover_state message
 *
 * @return Kalman state component 2 (acc)
 */
static inline float mavlink_msg_huch_ctrl_hover_state_get_kal_s2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a huch_ctrl_hover_state message into a struct
 *
 * @param msg The message to decode
 * @param huch_ctrl_hover_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_ctrl_hover_state_decode(const mavlink_message_t* msg, mavlink_huch_ctrl_hover_state_t* huch_ctrl_hover_state)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_ctrl_hover_state->uss = mavlink_msg_huch_ctrl_hover_state_get_uss(msg);
	huch_ctrl_hover_state->baro = mavlink_msg_huch_ctrl_hover_state_get_baro(msg);
	huch_ctrl_hover_state->accz = mavlink_msg_huch_ctrl_hover_state_get_accz(msg);
	huch_ctrl_hover_state->ir1 = mavlink_msg_huch_ctrl_hover_state_get_ir1(msg);
	huch_ctrl_hover_state->ir2 = mavlink_msg_huch_ctrl_hover_state_get_ir2(msg);
	huch_ctrl_hover_state->kal_s0 = mavlink_msg_huch_ctrl_hover_state_get_kal_s0(msg);
	huch_ctrl_hover_state->kal_s1 = mavlink_msg_huch_ctrl_hover_state_get_kal_s1(msg);
	huch_ctrl_hover_state->kal_s2 = mavlink_msg_huch_ctrl_hover_state_get_kal_s2(msg);
#else
	memcpy(huch_ctrl_hover_state, _MAV_PAYLOAD(msg), 32);
#endif
}
