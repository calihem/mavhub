// MESSAGE HUCH_FC_ALTITUDE PACKING

#define MAVLINK_MSG_ID_HUCH_FC_ALTITUDE 204

typedef struct __mavlink_huch_fc_altitude_t
{
 int16_t baro; ///< 
 int16_t baroref; ///< 
} mavlink_huch_fc_altitude_t;

#define MAVLINK_MSG_ID_HUCH_FC_ALTITUDE_LEN 4
#define MAVLINK_MSG_ID_204_LEN 4



#define MAVLINK_MESSAGE_INFO_HUCH_FC_ALTITUDE { \
	"HUCH_FC_ALTITUDE", \
	2, \
	{  { "baro", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_huch_fc_altitude_t, baro) }, \
         { "baroref", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_huch_fc_altitude_t, baroref) }, \
         } \
}


/**
 * @brief Pack a huch_fc_altitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param baro 
 * @param baroref 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_fc_altitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t baro, int16_t baroref)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_int16_t(buf, 0, baro);
	_mav_put_int16_t(buf, 2, baroref);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_huch_fc_altitude_t packet;
	packet.baro = baro;
	packet.baroref = baroref;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_FC_ALTITUDE;
	return mavlink_finalize_message(msg, system_id, component_id, 4, 186);
}

/**
 * @brief Pack a huch_fc_altitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param baro 
 * @param baroref 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_fc_altitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t baro,int16_t baroref)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_int16_t(buf, 0, baro);
	_mav_put_int16_t(buf, 2, baroref);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_huch_fc_altitude_t packet;
	packet.baro = baro;
	packet.baroref = baroref;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_FC_ALTITUDE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 186);
}

/**
 * @brief Encode a huch_fc_altitude struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_fc_altitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_fc_altitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_fc_altitude_t* huch_fc_altitude)
{
	return mavlink_msg_huch_fc_altitude_pack(system_id, component_id, msg, huch_fc_altitude->baro, huch_fc_altitude->baroref);
}

/**
 * @brief Send a huch_fc_altitude message
 * @param chan MAVLink channel to send the message
 *
 * @param baro 
 * @param baroref 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_fc_altitude_send(mavlink_channel_t chan, int16_t baro, int16_t baroref)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_int16_t(buf, 0, baro);
	_mav_put_int16_t(buf, 2, baroref);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_FC_ALTITUDE, buf, 4, 186);
#else
	mavlink_huch_fc_altitude_t packet;
	packet.baro = baro;
	packet.baroref = baroref;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_FC_ALTITUDE, (const char *)&packet, 4, 186);
#endif
}

#endif

// MESSAGE HUCH_FC_ALTITUDE UNPACKING


/**
 * @brief Get field baro from huch_fc_altitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_fc_altitude_get_baro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field baroref from huch_fc_altitude message
 *
 * @return 
 */
static inline int16_t mavlink_msg_huch_fc_altitude_get_baroref(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Decode a huch_fc_altitude message into a struct
 *
 * @param msg The message to decode
 * @param huch_fc_altitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_fc_altitude_decode(const mavlink_message_t* msg, mavlink_huch_fc_altitude_t* huch_fc_altitude)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_fc_altitude->baro = mavlink_msg_huch_fc_altitude_get_baro(msg);
	huch_fc_altitude->baroref = mavlink_msg_huch_fc_altitude_get_baroref(msg);
#else
	memcpy(huch_fc_altitude, _MAV_PAYLOAD(msg), 4);
#endif
}
