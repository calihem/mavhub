// MESSAGE HUCH_IMU_RAW_ADC PACKING

#define MAVLINK_MSG_ID_HUCH_IMU_RAW_ADC 211

typedef struct __mavlink_huch_imu_raw_adc_t
{
 uint16_t xacc; ///< X acceleration (adc units)
 uint16_t yacc; ///< Y acceleration (adc units)
 uint16_t zacc; ///< Z acceleration (adc units)
 uint16_t xgyro; ///< Angular speed around X axis (adc units)
 uint16_t ygyro; ///< Angular speed around Y axis (adc units)
 uint16_t zgyro; ///< Angular speed around Z axis (adc units)
} mavlink_huch_imu_raw_adc_t;

#define MAVLINK_MSG_ID_HUCH_IMU_RAW_ADC_LEN 12
#define MAVLINK_MSG_ID_211_LEN 12



#define MAVLINK_MESSAGE_INFO_HUCH_IMU_RAW_ADC { \
	"HUCH_IMU_RAW_ADC", \
	6, \
	{  { "xacc", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_huch_imu_raw_adc_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_huch_imu_raw_adc_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_huch_imu_raw_adc_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_huch_imu_raw_adc_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_huch_imu_raw_adc_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_huch_imu_raw_adc_t, zgyro) }, \
         } \
}


/**
 * @brief Pack a huch_imu_raw_adc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param xacc X acceleration (adc units)
 * @param yacc Y acceleration (adc units)
 * @param zacc Z acceleration (adc units)
 * @param xgyro Angular speed around X axis (adc units)
 * @param ygyro Angular speed around Y axis (adc units)
 * @param zgyro Angular speed around Z axis (adc units)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_imu_raw_adc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t xacc, uint16_t yacc, uint16_t zacc, uint16_t xgyro, uint16_t ygyro, uint16_t zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint16_t(buf, 0, xacc);
	_mav_put_uint16_t(buf, 2, yacc);
	_mav_put_uint16_t(buf, 4, zacc);
	_mav_put_uint16_t(buf, 6, xgyro);
	_mav_put_uint16_t(buf, 8, ygyro);
	_mav_put_uint16_t(buf, 10, zgyro);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_imu_raw_adc_t packet;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_IMU_RAW_ADC;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 115);
}

/**
 * @brief Pack a huch_imu_raw_adc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param xacc X acceleration (adc units)
 * @param yacc Y acceleration (adc units)
 * @param zacc Z acceleration (adc units)
 * @param xgyro Angular speed around X axis (adc units)
 * @param ygyro Angular speed around Y axis (adc units)
 * @param zgyro Angular speed around Z axis (adc units)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_imu_raw_adc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t xacc,uint16_t yacc,uint16_t zacc,uint16_t xgyro,uint16_t ygyro,uint16_t zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint16_t(buf, 0, xacc);
	_mav_put_uint16_t(buf, 2, yacc);
	_mav_put_uint16_t(buf, 4, zacc);
	_mav_put_uint16_t(buf, 6, xgyro);
	_mav_put_uint16_t(buf, 8, ygyro);
	_mav_put_uint16_t(buf, 10, zgyro);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_huch_imu_raw_adc_t packet;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_IMU_RAW_ADC;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 115);
}

/**
 * @brief Encode a huch_imu_raw_adc struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_imu_raw_adc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_imu_raw_adc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_imu_raw_adc_t* huch_imu_raw_adc)
{
	return mavlink_msg_huch_imu_raw_adc_pack(system_id, component_id, msg, huch_imu_raw_adc->xacc, huch_imu_raw_adc->yacc, huch_imu_raw_adc->zacc, huch_imu_raw_adc->xgyro, huch_imu_raw_adc->ygyro, huch_imu_raw_adc->zgyro);
}

/**
 * @brief Send a huch_imu_raw_adc message
 * @param chan MAVLink channel to send the message
 *
 * @param xacc X acceleration (adc units)
 * @param yacc Y acceleration (adc units)
 * @param zacc Z acceleration (adc units)
 * @param xgyro Angular speed around X axis (adc units)
 * @param ygyro Angular speed around Y axis (adc units)
 * @param zgyro Angular speed around Z axis (adc units)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_imu_raw_adc_send(mavlink_channel_t chan, uint16_t xacc, uint16_t yacc, uint16_t zacc, uint16_t xgyro, uint16_t ygyro, uint16_t zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint16_t(buf, 0, xacc);
	_mav_put_uint16_t(buf, 2, yacc);
	_mav_put_uint16_t(buf, 4, zacc);
	_mav_put_uint16_t(buf, 6, xgyro);
	_mav_put_uint16_t(buf, 8, ygyro);
	_mav_put_uint16_t(buf, 10, zgyro);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_IMU_RAW_ADC, buf, 12, 115);
#else
	mavlink_huch_imu_raw_adc_t packet;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_IMU_RAW_ADC, (const char *)&packet, 12, 115);
#endif
}

#endif

// MESSAGE HUCH_IMU_RAW_ADC UNPACKING


/**
 * @brief Get field xacc from huch_imu_raw_adc message
 *
 * @return X acceleration (adc units)
 */
static inline uint16_t mavlink_msg_huch_imu_raw_adc_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field yacc from huch_imu_raw_adc message
 *
 * @return Y acceleration (adc units)
 */
static inline uint16_t mavlink_msg_huch_imu_raw_adc_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field zacc from huch_imu_raw_adc message
 *
 * @return Z acceleration (adc units)
 */
static inline uint16_t mavlink_msg_huch_imu_raw_adc_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field xgyro from huch_imu_raw_adc message
 *
 * @return Angular speed around X axis (adc units)
 */
static inline uint16_t mavlink_msg_huch_imu_raw_adc_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field ygyro from huch_imu_raw_adc message
 *
 * @return Angular speed around Y axis (adc units)
 */
static inline uint16_t mavlink_msg_huch_imu_raw_adc_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field zgyro from huch_imu_raw_adc message
 *
 * @return Angular speed around Z axis (adc units)
 */
static inline uint16_t mavlink_msg_huch_imu_raw_adc_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Decode a huch_imu_raw_adc message into a struct
 *
 * @param msg The message to decode
 * @param huch_imu_raw_adc C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_imu_raw_adc_decode(const mavlink_message_t* msg, mavlink_huch_imu_raw_adc_t* huch_imu_raw_adc)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_imu_raw_adc->xacc = mavlink_msg_huch_imu_raw_adc_get_xacc(msg);
	huch_imu_raw_adc->yacc = mavlink_msg_huch_imu_raw_adc_get_yacc(msg);
	huch_imu_raw_adc->zacc = mavlink_msg_huch_imu_raw_adc_get_zacc(msg);
	huch_imu_raw_adc->xgyro = mavlink_msg_huch_imu_raw_adc_get_xgyro(msg);
	huch_imu_raw_adc->ygyro = mavlink_msg_huch_imu_raw_adc_get_ygyro(msg);
	huch_imu_raw_adc->zgyro = mavlink_msg_huch_imu_raw_adc_get_zgyro(msg);
#else
	memcpy(huch_imu_raw_adc, _MAV_PAYLOAD(msg), 12);
#endif
}
