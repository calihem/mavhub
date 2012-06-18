// MESSAGE HUCH_VISUAL_NAVIGATION PACKING

#define MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION 218

typedef struct __mavlink_huch_visual_navigation_t
{
 float alt_velocity; ///< altitude velocity
 float alt_absolute; ///< altitude absolute value
 float home_beta; ///< home direction
 float home_distance; ///< home distance
 float visual_compass; ///< visual compass
 float ego_beta; ///< lateral movement direction
 float ego_speed; ///< lateral movement speed
 float debug; ///< debug information
 int16_t keypoints; ///< tracked keypoints
 int16_t error; ///< error code
} mavlink_huch_visual_navigation_t;

#define MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION_LEN 36
#define MAVLINK_MSG_ID_218_LEN 36



#define MAVLINK_MESSAGE_INFO_HUCH_VISUAL_NAVIGATION { \
	"HUCH_VISUAL_NAVIGATION", \
	10, \
	{  { "alt_velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_huch_visual_navigation_t, alt_velocity) }, \
         { "alt_absolute", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_huch_visual_navigation_t, alt_absolute) }, \
         { "home_beta", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_huch_visual_navigation_t, home_beta) }, \
         { "home_distance", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_huch_visual_navigation_t, home_distance) }, \
         { "visual_compass", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_huch_visual_navigation_t, visual_compass) }, \
         { "ego_beta", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_huch_visual_navigation_t, ego_beta) }, \
         { "ego_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_huch_visual_navigation_t, ego_speed) }, \
         { "debug", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_huch_visual_navigation_t, debug) }, \
         { "keypoints", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_huch_visual_navigation_t, keypoints) }, \
         { "error", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_huch_visual_navigation_t, error) }, \
         } \
}


/**
 * @brief Pack a huch_visual_navigation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param alt_velocity altitude velocity
 * @param alt_absolute altitude absolute value
 * @param home_beta home direction
 * @param home_distance home distance
 * @param visual_compass visual compass
 * @param ego_beta lateral movement direction
 * @param ego_speed lateral movement speed
 * @param keypoints tracked keypoints
 * @param error error code
 * @param debug debug information
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_visual_navigation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float alt_velocity, float alt_absolute, float home_beta, float home_distance, float visual_compass, float ego_beta, float ego_speed, int16_t keypoints, int16_t error, float debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_float(buf, 0, alt_velocity);
	_mav_put_float(buf, 4, alt_absolute);
	_mav_put_float(buf, 8, home_beta);
	_mav_put_float(buf, 12, home_distance);
	_mav_put_float(buf, 16, visual_compass);
	_mav_put_float(buf, 20, ego_beta);
	_mav_put_float(buf, 24, ego_speed);
	_mav_put_float(buf, 28, debug);
	_mav_put_int16_t(buf, 32, keypoints);
	_mav_put_int16_t(buf, 34, error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 36);
#else
	mavlink_huch_visual_navigation_t packet;
	packet.alt_velocity = alt_velocity;
	packet.alt_absolute = alt_absolute;
	packet.home_beta = home_beta;
	packet.home_distance = home_distance;
	packet.visual_compass = visual_compass;
	packet.ego_beta = ego_beta;
	packet.ego_speed = ego_speed;
	packet.debug = debug;
	packet.keypoints = keypoints;
	packet.error = error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION;
	return mavlink_finalize_message(msg, system_id, component_id, 36, 52);
}

/**
 * @brief Pack a huch_visual_navigation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param alt_velocity altitude velocity
 * @param alt_absolute altitude absolute value
 * @param home_beta home direction
 * @param home_distance home distance
 * @param visual_compass visual compass
 * @param ego_beta lateral movement direction
 * @param ego_speed lateral movement speed
 * @param keypoints tracked keypoints
 * @param error error code
 * @param debug debug information
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_visual_navigation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float alt_velocity,float alt_absolute,float home_beta,float home_distance,float visual_compass,float ego_beta,float ego_speed,int16_t keypoints,int16_t error,float debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_float(buf, 0, alt_velocity);
	_mav_put_float(buf, 4, alt_absolute);
	_mav_put_float(buf, 8, home_beta);
	_mav_put_float(buf, 12, home_distance);
	_mav_put_float(buf, 16, visual_compass);
	_mav_put_float(buf, 20, ego_beta);
	_mav_put_float(buf, 24, ego_speed);
	_mav_put_float(buf, 28, debug);
	_mav_put_int16_t(buf, 32, keypoints);
	_mav_put_int16_t(buf, 34, error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 36);
#else
	mavlink_huch_visual_navigation_t packet;
	packet.alt_velocity = alt_velocity;
	packet.alt_absolute = alt_absolute;
	packet.home_beta = home_beta;
	packet.home_distance = home_distance;
	packet.visual_compass = visual_compass;
	packet.ego_beta = ego_beta;
	packet.ego_speed = ego_speed;
	packet.debug = debug;
	packet.keypoints = keypoints;
	packet.error = error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36, 52);
}

/**
 * @brief Encode a huch_visual_navigation struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_visual_navigation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_visual_navigation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_visual_navigation_t* huch_visual_navigation)
{
	return mavlink_msg_huch_visual_navigation_pack(system_id, component_id, msg, huch_visual_navigation->alt_velocity, huch_visual_navigation->alt_absolute, huch_visual_navigation->home_beta, huch_visual_navigation->home_distance, huch_visual_navigation->visual_compass, huch_visual_navigation->ego_beta, huch_visual_navigation->ego_speed, huch_visual_navigation->keypoints, huch_visual_navigation->error, huch_visual_navigation->debug);
}

/**
 * @brief Send a huch_visual_navigation message
 * @param chan MAVLink channel to send the message
 *
 * @param alt_velocity altitude velocity
 * @param alt_absolute altitude absolute value
 * @param home_beta home direction
 * @param home_distance home distance
 * @param visual_compass visual compass
 * @param ego_beta lateral movement direction
 * @param ego_speed lateral movement speed
 * @param keypoints tracked keypoints
 * @param error error code
 * @param debug debug information
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_visual_navigation_send(mavlink_channel_t chan, float alt_velocity, float alt_absolute, float home_beta, float home_distance, float visual_compass, float ego_beta, float ego_speed, int16_t keypoints, int16_t error, float debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_float(buf, 0, alt_velocity);
	_mav_put_float(buf, 4, alt_absolute);
	_mav_put_float(buf, 8, home_beta);
	_mav_put_float(buf, 12, home_distance);
	_mav_put_float(buf, 16, visual_compass);
	_mav_put_float(buf, 20, ego_beta);
	_mav_put_float(buf, 24, ego_speed);
	_mav_put_float(buf, 28, debug);
	_mav_put_int16_t(buf, 32, keypoints);
	_mav_put_int16_t(buf, 34, error);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION, buf, 36, 52);
#else
	mavlink_huch_visual_navigation_t packet;
	packet.alt_velocity = alt_velocity;
	packet.alt_absolute = alt_absolute;
	packet.home_beta = home_beta;
	packet.home_distance = home_distance;
	packet.visual_compass = visual_compass;
	packet.ego_beta = ego_beta;
	packet.ego_speed = ego_speed;
	packet.debug = debug;
	packet.keypoints = keypoints;
	packet.error = error;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_VISUAL_NAVIGATION, (const char *)&packet, 36, 52);
#endif
}

#endif

// MESSAGE HUCH_VISUAL_NAVIGATION UNPACKING


/**
 * @brief Get field alt_velocity from huch_visual_navigation message
 *
 * @return altitude velocity
 */
static inline float mavlink_msg_huch_visual_navigation_get_alt_velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field alt_absolute from huch_visual_navigation message
 *
 * @return altitude absolute value
 */
static inline float mavlink_msg_huch_visual_navigation_get_alt_absolute(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field home_beta from huch_visual_navigation message
 *
 * @return home direction
 */
static inline float mavlink_msg_huch_visual_navigation_get_home_beta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field home_distance from huch_visual_navigation message
 *
 * @return home distance
 */
static inline float mavlink_msg_huch_visual_navigation_get_home_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field visual_compass from huch_visual_navigation message
 *
 * @return visual compass
 */
static inline float mavlink_msg_huch_visual_navigation_get_visual_compass(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field ego_beta from huch_visual_navigation message
 *
 * @return lateral movement direction
 */
static inline float mavlink_msg_huch_visual_navigation_get_ego_beta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ego_speed from huch_visual_navigation message
 *
 * @return lateral movement speed
 */
static inline float mavlink_msg_huch_visual_navigation_get_ego_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field keypoints from huch_visual_navigation message
 *
 * @return tracked keypoints
 */
static inline int16_t mavlink_msg_huch_visual_navigation_get_keypoints(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field error from huch_visual_navigation message
 *
 * @return error code
 */
static inline int16_t mavlink_msg_huch_visual_navigation_get_error(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  34);
}

/**
 * @brief Get field debug from huch_visual_navigation message
 *
 * @return debug information
 */
static inline float mavlink_msg_huch_visual_navigation_get_debug(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a huch_visual_navigation message into a struct
 *
 * @param msg The message to decode
 * @param huch_visual_navigation C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_visual_navigation_decode(const mavlink_message_t* msg, mavlink_huch_visual_navigation_t* huch_visual_navigation)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_visual_navigation->alt_velocity = mavlink_msg_huch_visual_navigation_get_alt_velocity(msg);
	huch_visual_navigation->alt_absolute = mavlink_msg_huch_visual_navigation_get_alt_absolute(msg);
	huch_visual_navigation->home_beta = mavlink_msg_huch_visual_navigation_get_home_beta(msg);
	huch_visual_navigation->home_distance = mavlink_msg_huch_visual_navigation_get_home_distance(msg);
	huch_visual_navigation->visual_compass = mavlink_msg_huch_visual_navigation_get_visual_compass(msg);
	huch_visual_navigation->ego_beta = mavlink_msg_huch_visual_navigation_get_ego_beta(msg);
	huch_visual_navigation->ego_speed = mavlink_msg_huch_visual_navigation_get_ego_speed(msg);
	huch_visual_navigation->debug = mavlink_msg_huch_visual_navigation_get_debug(msg);
	huch_visual_navigation->keypoints = mavlink_msg_huch_visual_navigation_get_keypoints(msg);
	huch_visual_navigation->error = mavlink_msg_huch_visual_navigation_get_error(msg);
#else
	memcpy(huch_visual_navigation, _MAV_PAYLOAD(msg), 36);
#endif
}
