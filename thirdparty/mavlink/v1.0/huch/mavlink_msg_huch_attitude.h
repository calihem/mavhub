// MESSAGE HUCH_ATTITUDE PACKING

#define MAVLINK_MSG_ID_HUCH_ATTITUDE 203

typedef struct __mavlink_huch_attitude_t
{
 int32_t xgyroint; ///< 
 int32_t ygyroint; ///< 
 int32_t zgyroint; ///< 
 int16_t xacc; ///< 
 int16_t yacc; ///< 
 int16_t zacc; ///< 
 int16_t zaccraw; ///< 
 int16_t xaccmean; ///< 
 int16_t yaccmean; ///< 
 int16_t zaccmean; ///< 
 int16_t xgyro; ///< 
 int16_t ygyro; ///< 
 int16_t zgyro; ///< 
 int16_t xmag; ///< 
 int16_t ymag; ///< 
 int16_t zmag; ///< 
} mavlink_huch_attitude_t;

#define MAVLINK_MSG_ID_HUCH_ATTITUDE_LEN 38
#define MAVLINK_MSG_ID_203_LEN 38



#define MAVLINK_MESSAGE_INFO_HUCH_ATTITUDE { \
	"HUCH_ATTITUDE", \
	16, \
	{  { "xgyroint", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_huch_attitude_t, xgyroint) }, \
         { "ygyroint", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_huch_attitude_t, ygyroint) }, \
         { "zgyroint", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_huch_attitude_t, zgyroint) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_huch_attitude_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_huch_attitude_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_huch_attitude_t, zacc) }, \
         { "zaccraw", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_huch_attitude_t, zaccraw) }, \
         { "xaccmean", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_huch_attitude_t, xaccmean) }, \
         { "yaccmean", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_huch_attitude_t, yaccmean) }, \
         { "zaccmean", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_huch_attitude_t, zaccmean) }, \
         { "xgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_huch_attitude_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_huch_attitude_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_huch_attitude_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_huch_attitude_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_huch_attitude_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_INT16_T, 0, 36, offsetof(mavlink_huch_attitude_t, zmag) }, \
         } \
}


/**
 * @brief Pack a huch_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param xacc 
 * @param yacc 
 * @param zacc 
 * @param zaccraw 
 * @param xaccmean 
 * @param yaccmean 
 * @param zaccmean 
 * @param xgyro 
 * @param ygyro 
 * @param zgyro 
 * @param xgyroint 
 * @param ygyroint 
 * @param zgyroint 
 * @param xmag 
 * @param ymag 
 * @param zmag 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t xacc, int16_t yacc, int16_t zacc, int16_t zaccraw, int16_t xaccmean, int16_t yaccmean, int16_t zaccmean, int16_t xgyro, int16_t ygyro, int16_t zgyro, int32_t xgyroint, int32_t ygyroint, int32_t zgyroint, int16_t xmag, int16_t ymag, int16_t zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[38];
	_mav_put_int32_t(buf, 0, xgyroint);
	_mav_put_int32_t(buf, 4, ygyroint);
	_mav_put_int32_t(buf, 8, zgyroint);
	_mav_put_int16_t(buf, 12, xacc);
	_mav_put_int16_t(buf, 14, yacc);
	_mav_put_int16_t(buf, 16, zacc);
	_mav_put_int16_t(buf, 18, zaccraw);
	_mav_put_int16_t(buf, 20, xaccmean);
	_mav_put_int16_t(buf, 22, yaccmean);
	_mav_put_int16_t(buf, 24, zaccmean);
	_mav_put_int16_t(buf, 26, xgyro);
	_mav_put_int16_t(buf, 28, ygyro);
	_mav_put_int16_t(buf, 30, zgyro);
	_mav_put_int16_t(buf, 32, xmag);
	_mav_put_int16_t(buf, 34, ymag);
	_mav_put_int16_t(buf, 36, zmag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 38);
#else
	mavlink_huch_attitude_t packet;
	packet.xgyroint = xgyroint;
	packet.ygyroint = ygyroint;
	packet.zgyroint = zgyroint;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.zaccraw = zaccraw;
	packet.xaccmean = xaccmean;
	packet.yaccmean = yaccmean;
	packet.zaccmean = zaccmean;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 38);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_ATTITUDE;
	return mavlink_finalize_message(msg, system_id, component_id, 38, 223);
}

/**
 * @brief Pack a huch_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param xacc 
 * @param yacc 
 * @param zacc 
 * @param zaccraw 
 * @param xaccmean 
 * @param yaccmean 
 * @param zaccmean 
 * @param xgyro 
 * @param ygyro 
 * @param zgyro 
 * @param xgyroint 
 * @param ygyroint 
 * @param zgyroint 
 * @param xmag 
 * @param ymag 
 * @param zmag 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t xacc,int16_t yacc,int16_t zacc,int16_t zaccraw,int16_t xaccmean,int16_t yaccmean,int16_t zaccmean,int16_t xgyro,int16_t ygyro,int16_t zgyro,int32_t xgyroint,int32_t ygyroint,int32_t zgyroint,int16_t xmag,int16_t ymag,int16_t zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[38];
	_mav_put_int32_t(buf, 0, xgyroint);
	_mav_put_int32_t(buf, 4, ygyroint);
	_mav_put_int32_t(buf, 8, zgyroint);
	_mav_put_int16_t(buf, 12, xacc);
	_mav_put_int16_t(buf, 14, yacc);
	_mav_put_int16_t(buf, 16, zacc);
	_mav_put_int16_t(buf, 18, zaccraw);
	_mav_put_int16_t(buf, 20, xaccmean);
	_mav_put_int16_t(buf, 22, yaccmean);
	_mav_put_int16_t(buf, 24, zaccmean);
	_mav_put_int16_t(buf, 26, xgyro);
	_mav_put_int16_t(buf, 28, ygyro);
	_mav_put_int16_t(buf, 30, zgyro);
	_mav_put_int16_t(buf, 32, xmag);
	_mav_put_int16_t(buf, 34, ymag);
	_mav_put_int16_t(buf, 36, zmag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 38);
#else
	mavlink_huch_attitude_t packet;
	packet.xgyroint = xgyroint;
	packet.ygyroint = ygyroint;
	packet.zgyroint = zgyroint;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.zaccraw = zaccraw;
	packet.xaccmean = xaccmean;
	packet.yaccmean = yaccmean;
	packet.zaccmean = zaccmean;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 38);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_ATTITUDE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 38, 223);
}

/**
 * @brief Encode a huch_attitude struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_attitude_t* huch_attitude)
{
	return mavlink_msg_huch_attitude_pack(system_id, component_id, msg, huch_attitude->xacc, huch_attitude->yacc, huch_attitude->zacc, huch_attitude->zaccraw, huch_attitude->xaccmean, huch_attitude->yaccmean, huch_attitude->zaccmean, huch_attitude->xgyro, huch_attitude->ygyro, huch_attitude->zgyro, huch_attitude->xgyroint, huch_attitude->ygyroint, huch_attitude->zgyroint, huch_attitude->xmag, huch_attitude->ymag, huch_attitude->zmag);
}

/**
 * @brief Send a huch_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param xacc 
 * @param yacc 
 * @param zacc 
 * @param zaccraw 
 * @param xaccmean 
 * @param yaccmean 
 * @param zaccmean 
 * @param xgyro 
 * @param ygyro 
 * @param zgyro 
 * @param xgyroint 
 * @param ygyroint 
 * @param zgyroint 
 * @param xmag 
 * @param ymag 
 * @param zmag 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_attitude_send(mavlink_channel_t chan, int16_t xacc, int16_t yacc, int16_t zacc, int16_t zaccraw, int16_t xaccmean, int16_t yaccmean, int16_t zaccmean, int16_t xgyro, int16_t ygyro, int16_t zgyro, int32_t xgyroint, int32_t ygyroint, int32_t zgyroint, int16_t xmag, int16_t ymag, int16_t zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[38];
	_mav_put_int32_t(buf, 0, xgyroint);
	_mav_put_int32_t(buf, 4, ygyroint);
	_mav_put_int32_t(buf, 8, zgyroint);
	_mav_put_int16_t(buf, 12, xacc);
	_mav_put_int16_t(buf, 14, yacc);
	_mav_put_int16_t(buf, 16, zacc);
	_mav_put_int16_t(buf, 18, zaccraw);
	_mav_put_int16_t(buf, 20, xaccmean);
	_mav_put_int16_t(buf, 22, yaccmean);
	_mav_put_int16_t(buf, 24, zaccmean);
	_mav_put_int16_t(buf, 26, xgyro);
	_mav_put_int16_t(buf, 28, ygyro);
	_mav_put_int16_t(buf, 30, zgyro);
	_mav_put_int16_t(buf, 32, xmag);
	_mav_put_int16_t(buf, 34, ymag);
	_mav_put_int16_t(buf, 36, zmag);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_ATTITUDE, buf, 38, 223);
#else
	mavlink_huch_attitude_t packet;
	packet.xgyroint = xgyroint;
	packet.ygyroint = ygyroint;
	packet.zgyroint = zgyroint;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.zaccraw = zaccraw;
	packet.xaccmean = xaccmean;
	packet.yaccmean = yaccmean;
	packet.zaccmean = zaccmean;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_ATTITUDE, (const char *)&packet, 38, 223);
#endif
}

#endif

// MESSAGE HUCH_ATTITUDE UNPACKING


/**
 * @brief Get field xacc from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field yacc from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field zacc from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field zaccraw from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_zaccraw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field xaccmean from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_xaccmean(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field yaccmean from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_yaccmean(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field zaccmean from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_zaccmean(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field xgyro from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field ygyro from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field zgyro from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field xgyroint from huch_attitude message
 *
 * @return 
 */
static inline int32_t mavlink_msg_huch_attitude_get_xgyroint(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field ygyroint from huch_attitude message
 *
 * @return 
 */
static inline int32_t mavlink_msg_huch_attitude_get_ygyroint(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field zgyroint from huch_attitude message
 *
 * @return 
 */
static inline int32_t mavlink_msg_huch_attitude_get_zgyroint(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field xmag from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_xmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field ymag from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_ymag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  34);
}

/**
 * @brief Get field zmag from huch_attitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_attitude_get_zmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  36);
}

/**
 * @brief Decode a huch_attitude message into a struct
 *
 * @param msg The message to decode
 * @param huch_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_attitude_decode(const mavlink_message_t* msg, mavlink_huch_attitude_t* huch_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_attitude->xgyroint = mavlink_msg_huch_attitude_get_xgyroint(msg);
	huch_attitude->ygyroint = mavlink_msg_huch_attitude_get_ygyroint(msg);
	huch_attitude->zgyroint = mavlink_msg_huch_attitude_get_zgyroint(msg);
	huch_attitude->xacc = mavlink_msg_huch_attitude_get_xacc(msg);
	huch_attitude->yacc = mavlink_msg_huch_attitude_get_yacc(msg);
	huch_attitude->zacc = mavlink_msg_huch_attitude_get_zacc(msg);
	huch_attitude->zaccraw = mavlink_msg_huch_attitude_get_zaccraw(msg);
	huch_attitude->xaccmean = mavlink_msg_huch_attitude_get_xaccmean(msg);
	huch_attitude->yaccmean = mavlink_msg_huch_attitude_get_yaccmean(msg);
	huch_attitude->zaccmean = mavlink_msg_huch_attitude_get_zaccmean(msg);
	huch_attitude->xgyro = mavlink_msg_huch_attitude_get_xgyro(msg);
	huch_attitude->ygyro = mavlink_msg_huch_attitude_get_ygyro(msg);
	huch_attitude->zgyro = mavlink_msg_huch_attitude_get_zgyro(msg);
	huch_attitude->xmag = mavlink_msg_huch_attitude_get_xmag(msg);
	huch_attitude->ymag = mavlink_msg_huch_attitude_get_ymag(msg);
	huch_attitude->zmag = mavlink_msg_huch_attitude_get_zmag(msg);
#else
	memcpy(huch_attitude, _MAV_PAYLOAD(msg), 38);
#endif
}
