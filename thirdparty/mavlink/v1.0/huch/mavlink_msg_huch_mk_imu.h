// MESSAGE HUCH_MK_IMU PACKING

#define MAVLINK_MSG_ID_HUCH_MK_IMU 212

typedef struct __mavlink_huch_mk_imu_t
{
 uint64_t usec; ///< Timestamp (microseconds since UNIX epoch)
 int16_t xacc; ///< Calibrated X acceleration (mg)
 int16_t yacc; ///< Calibrated Y acceleration (mg)
 int16_t zacc; ///< Calibrated Z acceleration (mg)
 int16_t xgyro; ///< Uncalibrated angular speed around X axis (0.1 deg/sec)
 int16_t ygyro; ///< Uncalibrated angular speed around Y axis (0.1 deg/sec)
 int16_t zgyro; ///< Uncalibrated angular speed around Z axis (0.1 deg/sec)
} mavlink_huch_mk_imu_t;

#define MAVLINK_MSG_ID_HUCH_MK_IMU_LEN 20
#define MAVLINK_MSG_ID_212_LEN 20



#define MAVLINK_MESSAGE_INFO_HUCH_MK_IMU { \
	"HUCH_MK_IMU", \
	7, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_mk_imu_t, usec) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_huch_mk_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_huch_mk_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_huch_mk_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_huch_mk_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_huch_mk_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_huch_mk_imu_t, zgyro) }, \
         } \
}


/**
 * @brief Pack a huch_mk_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch)
 * @param xacc Calibrated X acceleration (mg)
 * @param yacc Calibrated Y acceleration (mg)
 * @param zacc Calibrated Z acceleration (mg)
 * @param xgyro Uncalibrated angular speed around X axis (0.1 deg/sec)
 * @param ygyro Uncalibrated angular speed around Y axis (0.1 deg/sec)
 * @param zgyro Uncalibrated angular speed around Z axis (0.1 deg/sec)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_mk_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int16_t(buf, 8, xacc);
	_mav_put_int16_t(buf, 10, yacc);
	_mav_put_int16_t(buf, 12, zacc);
	_mav_put_int16_t(buf, 14, xgyro);
	_mav_put_int16_t(buf, 16, ygyro);
	_mav_put_int16_t(buf, 18, zgyro);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 20);
#else
	mavlink_huch_mk_imu_t packet;
	packet.usec = usec;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_MK_IMU;
	return mavlink_finalize_message(msg, system_id, component_id, 20, 122);
}

/**
 * @brief Pack a huch_mk_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch)
 * @param xacc Calibrated X acceleration (mg)
 * @param yacc Calibrated Y acceleration (mg)
 * @param zacc Calibrated Z acceleration (mg)
 * @param xgyro Uncalibrated angular speed around X axis (0.1 deg/sec)
 * @param ygyro Uncalibrated angular speed around Y axis (0.1 deg/sec)
 * @param zgyro Uncalibrated angular speed around Z axis (0.1 deg/sec)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_mk_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,int16_t xacc,int16_t yacc,int16_t zacc,int16_t xgyro,int16_t ygyro,int16_t zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int16_t(buf, 8, xacc);
	_mav_put_int16_t(buf, 10, yacc);
	_mav_put_int16_t(buf, 12, zacc);
	_mav_put_int16_t(buf, 14, xgyro);
	_mav_put_int16_t(buf, 16, ygyro);
	_mav_put_int16_t(buf, 18, zgyro);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 20);
#else
	mavlink_huch_mk_imu_t packet;
	packet.usec = usec;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_MK_IMU;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20, 122);
}

/**
 * @brief Encode a huch_mk_imu struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_mk_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_mk_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_mk_imu_t* huch_mk_imu)
{
	return mavlink_msg_huch_mk_imu_pack(system_id, component_id, msg, huch_mk_imu->usec, huch_mk_imu->xacc, huch_mk_imu->yacc, huch_mk_imu->zacc, huch_mk_imu->xgyro, huch_mk_imu->ygyro, huch_mk_imu->zgyro);
}

/**
 * @brief Send a huch_mk_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch)
 * @param xacc Calibrated X acceleration (mg)
 * @param yacc Calibrated Y acceleration (mg)
 * @param zacc Calibrated Z acceleration (mg)
 * @param xgyro Uncalibrated angular speed around X axis (0.1 deg/sec)
 * @param ygyro Uncalibrated angular speed around Y axis (0.1 deg/sec)
 * @param zgyro Uncalibrated angular speed around Z axis (0.1 deg/sec)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_mk_imu_send(mavlink_channel_t chan, uint64_t usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_int16_t(buf, 8, xacc);
	_mav_put_int16_t(buf, 10, yacc);
	_mav_put_int16_t(buf, 12, zacc);
	_mav_put_int16_t(buf, 14, xgyro);
	_mav_put_int16_t(buf, 16, ygyro);
	_mav_put_int16_t(buf, 18, zgyro);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_MK_IMU, buf, 20, 122);
#else
	mavlink_huch_mk_imu_t packet;
	packet.usec = usec;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_MK_IMU, (const char *)&packet, 20, 122);
#endif
}

#endif

// MESSAGE HUCH_MK_IMU UNPACKING


/**
 * @brief Get field usec from huch_mk_imu message
 *
 * @return Timestamp (microseconds since UNIX epoch)
 */
static inline uint64_t mavlink_msg_huch_mk_imu_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field xacc from huch_mk_imu message
 *
 * @return Calibrated X acceleration (mg)
 */
static inline int16_t mavlink_msg_huch_mk_imu_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field yacc from huch_mk_imu message
 *
 * @return Calibrated Y acceleration (mg)
 */
static inline int16_t mavlink_msg_huch_mk_imu_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field zacc from huch_mk_imu message
 *
 * @return Calibrated Z acceleration (mg)
 */
static inline int16_t mavlink_msg_huch_mk_imu_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field xgyro from huch_mk_imu message
 *
 * @return Uncalibrated angular speed around X axis (0.1 deg/sec)
 */
static inline int16_t mavlink_msg_huch_mk_imu_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field ygyro from huch_mk_imu message
 *
 * @return Uncalibrated angular speed around Y axis (0.1 deg/sec)
 */
static inline int16_t mavlink_msg_huch_mk_imu_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field zgyro from huch_mk_imu message
 *
 * @return Uncalibrated angular speed around Z axis (0.1 deg/sec)
 */
static inline int16_t mavlink_msg_huch_mk_imu_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Decode a huch_mk_imu message into a struct
 *
 * @param msg The message to decode
 * @param huch_mk_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_mk_imu_decode(const mavlink_message_t* msg, mavlink_huch_mk_imu_t* huch_mk_imu)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_mk_imu->usec = mavlink_msg_huch_mk_imu_get_usec(msg);
	huch_mk_imu->xacc = mavlink_msg_huch_mk_imu_get_xacc(msg);
	huch_mk_imu->yacc = mavlink_msg_huch_mk_imu_get_yacc(msg);
	huch_mk_imu->zacc = mavlink_msg_huch_mk_imu_get_zacc(msg);
	huch_mk_imu->xgyro = mavlink_msg_huch_mk_imu_get_xgyro(msg);
	huch_mk_imu->ygyro = mavlink_msg_huch_mk_imu_get_ygyro(msg);
	huch_mk_imu->zgyro = mavlink_msg_huch_mk_imu_get_zgyro(msg);
#else
	memcpy(huch_mk_imu, _MAV_PAYLOAD(msg), 20);
#endif
}
