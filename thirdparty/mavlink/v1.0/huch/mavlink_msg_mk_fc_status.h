// MESSAGE MK_FC_STATUS PACKING

#define MAVLINK_MSG_ID_MK_FC_STATUS 209

typedef struct __mavlink_mk_fc_status_t
{
 int16_t rssi; ///< RC Control RSSI on FlightCtrl
 int16_t batt; ///< Battery Voltage Level
 int16_t nick; ///< FC internal nick value (steering.StickNick)
 int16_t roll; ///< FC internal roll value (steering.StickRoll)
 int16_t yaw; ///< FC internal yaw value (steering.StickYaw)
 int16_t gas; ///< FC internal gas value (GasMixFraction)
} mavlink_mk_fc_status_t;

#define MAVLINK_MSG_ID_MK_FC_STATUS_LEN 12
#define MAVLINK_MSG_ID_209_LEN 12



#define MAVLINK_MESSAGE_INFO_MK_FC_STATUS { \
	"MK_FC_STATUS", \
	6, \
	{  { "rssi", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_mk_fc_status_t, rssi) }, \
         { "batt", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_mk_fc_status_t, batt) }, \
         { "nick", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_mk_fc_status_t, nick) }, \
         { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_mk_fc_status_t, roll) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_mk_fc_status_t, yaw) }, \
         { "gas", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_mk_fc_status_t, gas) }, \
         } \
}


/**
 * @brief Pack a mk_fc_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rssi RC Control RSSI on FlightCtrl
 * @param batt Battery Voltage Level
 * @param nick FC internal nick value (steering.StickNick)
 * @param roll FC internal roll value (steering.StickRoll)
 * @param yaw FC internal yaw value (steering.StickYaw)
 * @param gas FC internal gas value (GasMixFraction)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mk_fc_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t rssi, int16_t batt, int16_t nick, int16_t roll, int16_t yaw, int16_t gas)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_int16_t(buf, 0, rssi);
	_mav_put_int16_t(buf, 2, batt);
	_mav_put_int16_t(buf, 4, nick);
	_mav_put_int16_t(buf, 6, roll);
	_mav_put_int16_t(buf, 8, yaw);
	_mav_put_int16_t(buf, 10, gas);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_mk_fc_status_t packet;
	packet.rssi = rssi;
	packet.batt = batt;
	packet.nick = nick;
	packet.roll = roll;
	packet.yaw = yaw;
	packet.gas = gas;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_MK_FC_STATUS;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 245);
}

/**
 * @brief Pack a mk_fc_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param rssi RC Control RSSI on FlightCtrl
 * @param batt Battery Voltage Level
 * @param nick FC internal nick value (steering.StickNick)
 * @param roll FC internal roll value (steering.StickRoll)
 * @param yaw FC internal yaw value (steering.StickYaw)
 * @param gas FC internal gas value (GasMixFraction)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mk_fc_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t rssi,int16_t batt,int16_t nick,int16_t roll,int16_t yaw,int16_t gas)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_int16_t(buf, 0, rssi);
	_mav_put_int16_t(buf, 2, batt);
	_mav_put_int16_t(buf, 4, nick);
	_mav_put_int16_t(buf, 6, roll);
	_mav_put_int16_t(buf, 8, yaw);
	_mav_put_int16_t(buf, 10, gas);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_mk_fc_status_t packet;
	packet.rssi = rssi;
	packet.batt = batt;
	packet.nick = nick;
	packet.roll = roll;
	packet.yaw = yaw;
	packet.gas = gas;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_MK_FC_STATUS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 245);
}

/**
 * @brief Encode a mk_fc_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mk_fc_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mk_fc_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mk_fc_status_t* mk_fc_status)
{
	return mavlink_msg_mk_fc_status_pack(system_id, component_id, msg, mk_fc_status->rssi, mk_fc_status->batt, mk_fc_status->nick, mk_fc_status->roll, mk_fc_status->yaw, mk_fc_status->gas);
}

/**
 * @brief Send a mk_fc_status message
 * @param chan MAVLink channel to send the message
 *
 * @param rssi RC Control RSSI on FlightCtrl
 * @param batt Battery Voltage Level
 * @param nick FC internal nick value (steering.StickNick)
 * @param roll FC internal roll value (steering.StickRoll)
 * @param yaw FC internal yaw value (steering.StickYaw)
 * @param gas FC internal gas value (GasMixFraction)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mk_fc_status_send(mavlink_channel_t chan, int16_t rssi, int16_t batt, int16_t nick, int16_t roll, int16_t yaw, int16_t gas)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_int16_t(buf, 0, rssi);
	_mav_put_int16_t(buf, 2, batt);
	_mav_put_int16_t(buf, 4, nick);
	_mav_put_int16_t(buf, 6, roll);
	_mav_put_int16_t(buf, 8, yaw);
	_mav_put_int16_t(buf, 10, gas);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MK_FC_STATUS, buf, 12, 245);
#else
	mavlink_mk_fc_status_t packet;
	packet.rssi = rssi;
	packet.batt = batt;
	packet.nick = nick;
	packet.roll = roll;
	packet.yaw = yaw;
	packet.gas = gas;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MK_FC_STATUS, (const char *)&packet, 12, 245);
#endif
}

#endif

// MESSAGE MK_FC_STATUS UNPACKING


/**
 * @brief Get field rssi from mk_fc_status message
 *
 * @return RC Control RSSI on FlightCtrl
 */
static inline int16_t mavlink_msg_mk_fc_status_get_rssi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field batt from mk_fc_status message
 *
 * @return Battery Voltage Level
 */
static inline int16_t mavlink_msg_mk_fc_status_get_batt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field nick from mk_fc_status message
 *
 * @return FC internal nick value (steering.StickNick)
 */
static inline int16_t mavlink_msg_mk_fc_status_get_nick(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field roll from mk_fc_status message
 *
 * @return FC internal roll value (steering.StickRoll)
 */
static inline int16_t mavlink_msg_mk_fc_status_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field yaw from mk_fc_status message
 *
 * @return FC internal yaw value (steering.StickYaw)
 */
static inline int16_t mavlink_msg_mk_fc_status_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field gas from mk_fc_status message
 *
 * @return FC internal gas value (GasMixFraction)
 */
static inline int16_t mavlink_msg_mk_fc_status_get_gas(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Decode a mk_fc_status message into a struct
 *
 * @param msg The message to decode
 * @param mk_fc_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_mk_fc_status_decode(const mavlink_message_t* msg, mavlink_mk_fc_status_t* mk_fc_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	mk_fc_status->rssi = mavlink_msg_mk_fc_status_get_rssi(msg);
	mk_fc_status->batt = mavlink_msg_mk_fc_status_get_batt(msg);
	mk_fc_status->nick = mavlink_msg_mk_fc_status_get_nick(msg);
	mk_fc_status->roll = mavlink_msg_mk_fc_status_get_roll(msg);
	mk_fc_status->yaw = mavlink_msg_mk_fc_status_get_yaw(msg);
	mk_fc_status->gas = mavlink_msg_mk_fc_status_get_gas(msg);
#else
	memcpy(mk_fc_status, _MAV_PAYLOAD(msg), 12);
#endif
}
