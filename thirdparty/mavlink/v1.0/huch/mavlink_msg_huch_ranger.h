// MESSAGE HUCH_RANGER PACKING

#define MAVLINK_MSG_ID_HUCH_RANGER 205

typedef struct __mavlink_huch_ranger_t
{
 uint16_t ranger1; ///< 
 uint16_t ranger2; ///< 
 uint16_t ranger3; ///< 
} mavlink_huch_ranger_t;

#define MAVLINK_MSG_ID_HUCH_RANGER_LEN 6
#define MAVLINK_MSG_ID_205_LEN 6



#define MAVLINK_MESSAGE_INFO_HUCH_RANGER { \
	"HUCH_RANGER", \
	3, \
	{  { "ranger1", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_huch_ranger_t, ranger1) }, \
         { "ranger2", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_huch_ranger_t, ranger2) }, \
         { "ranger3", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_huch_ranger_t, ranger3) }, \
         } \
}


/**
 * @brief Pack a huch_ranger message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ranger1 
 * @param ranger2 
 * @param ranger3 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_ranger_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t ranger1, uint16_t ranger2, uint16_t ranger3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_uint16_t(buf, 0, ranger1);
	_mav_put_uint16_t(buf, 2, ranger2);
	_mav_put_uint16_t(buf, 4, ranger3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_huch_ranger_t packet;
	packet.ranger1 = ranger1;
	packet.ranger2 = ranger2;
	packet.ranger3 = ranger3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_RANGER;
	return mavlink_finalize_message(msg, system_id, component_id, 6, 47);
}

/**
 * @brief Pack a huch_ranger message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param ranger1 
 * @param ranger2 
 * @param ranger3 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_ranger_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t ranger1,uint16_t ranger2,uint16_t ranger3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_uint16_t(buf, 0, ranger1);
	_mav_put_uint16_t(buf, 2, ranger2);
	_mav_put_uint16_t(buf, 4, ranger3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_huch_ranger_t packet;
	packet.ranger1 = ranger1;
	packet.ranger2 = ranger2;
	packet.ranger3 = ranger3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_RANGER;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 6, 47);
}

/**
 * @brief Encode a huch_ranger struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_ranger C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_ranger_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_ranger_t* huch_ranger)
{
	return mavlink_msg_huch_ranger_pack(system_id, component_id, msg, huch_ranger->ranger1, huch_ranger->ranger2, huch_ranger->ranger3);
}

/**
 * @brief Send a huch_ranger message
 * @param chan MAVLink channel to send the message
 *
 * @param ranger1 
 * @param ranger2 
 * @param ranger3 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_ranger_send(mavlink_channel_t chan, uint16_t ranger1, uint16_t ranger2, uint16_t ranger3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_uint16_t(buf, 0, ranger1);
	_mav_put_uint16_t(buf, 2, ranger2);
	_mav_put_uint16_t(buf, 4, ranger3);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_RANGER, buf, 6, 47);
#else
	mavlink_huch_ranger_t packet;
	packet.ranger1 = ranger1;
	packet.ranger2 = ranger2;
	packet.ranger3 = ranger3;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_RANGER, (const char *)&packet, 6, 47);
#endif
}

#endif

// MESSAGE HUCH_RANGER UNPACKING


/**
 * @brief Get field ranger1 from huch_ranger message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_huch_ranger_get_ranger1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field ranger2 from huch_ranger message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_huch_ranger_get_ranger2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field ranger3 from huch_ranger message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_huch_ranger_get_ranger3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a huch_ranger message into a struct
 *
 * @param msg The message to decode
 * @param huch_ranger C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_ranger_decode(const mavlink_message_t* msg, mavlink_huch_ranger_t* huch_ranger)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_ranger->ranger1 = mavlink_msg_huch_ranger_get_ranger1(msg);
	huch_ranger->ranger2 = mavlink_msg_huch_ranger_get_ranger2(msg);
	huch_ranger->ranger3 = mavlink_msg_huch_ranger_get_ranger3(msg);
#else
	memcpy(huch_ranger, _MAV_PAYLOAD(msg), 6);
#endif
}
