// MESSAGE HUCH_SENSOR_ARRAY PACKING

#define MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY 221

typedef struct __mavlink_huch_sensor_array_t
{
 uint64_t usec; ///< timestamp in microseconds
 double data[16]; ///< 16-vector of generic double
} mavlink_huch_sensor_array_t;

#define MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY_LEN 136
#define MAVLINK_MSG_ID_221_LEN 136

#define MAVLINK_MSG_HUCH_SENSOR_ARRAY_FIELD_DATA_LEN 16

#define MAVLINK_MESSAGE_INFO_HUCH_SENSOR_ARRAY { \
	"HUCH_SENSOR_ARRAY", \
	2, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_huch_sensor_array_t, usec) }, \
         { "data", NULL, MAVLINK_TYPE_DOUBLE, 16, 8, offsetof(mavlink_huch_sensor_array_t, data) }, \
         } \
}


/**
 * @brief Pack a huch_sensor_array message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec timestamp in microseconds
 * @param data 16-vector of generic double
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_sensor_array_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, const double *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[136];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_double_array(buf, 8, data, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 136);
#else
	mavlink_huch_sensor_array_t packet;
	packet.usec = usec;
	mav_array_memcpy(packet.data, data, sizeof(double)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 136);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY;
	return mavlink_finalize_message(msg, system_id, component_id, 136, 247);
}

/**
 * @brief Pack a huch_sensor_array message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec timestamp in microseconds
 * @param data 16-vector of generic double
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_huch_sensor_array_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,const double *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[136];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_double_array(buf, 8, data, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 136);
#else
	mavlink_huch_sensor_array_t packet;
	packet.usec = usec;
	mav_array_memcpy(packet.data, data, sizeof(double)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 136);
#endif

	msg->msgid = MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 136, 247);
}

/**
 * @brief Encode a huch_sensor_array struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param huch_sensor_array C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_huch_sensor_array_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_huch_sensor_array_t* huch_sensor_array)
{
	return mavlink_msg_huch_sensor_array_pack(system_id, component_id, msg, huch_sensor_array->usec, huch_sensor_array->data);
}

/**
 * @brief Send a huch_sensor_array message
 * @param chan MAVLink channel to send the message
 *
 * @param usec timestamp in microseconds
 * @param data 16-vector of generic double
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_huch_sensor_array_send(mavlink_channel_t chan, uint64_t usec, const double *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[136];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_double_array(buf, 8, data, 16);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY, buf, 136, 247);
#else
	mavlink_huch_sensor_array_t packet;
	packet.usec = usec;
	mav_array_memcpy(packet.data, data, sizeof(double)*16);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUCH_SENSOR_ARRAY, (const char *)&packet, 136, 247);
#endif
}

#endif

// MESSAGE HUCH_SENSOR_ARRAY UNPACKING


/**
 * @brief Get field usec from huch_sensor_array message
 *
 * @return timestamp in microseconds
 */
static inline uint64_t mavlink_msg_huch_sensor_array_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field data from huch_sensor_array message
 *
 * @return 16-vector of generic double
 */
static inline uint16_t mavlink_msg_huch_sensor_array_get_data(const mavlink_message_t* msg, double *data)
{
	return _MAV_RETURN_double_array(msg, data, 16,  8);
}

/**
 * @brief Decode a huch_sensor_array message into a struct
 *
 * @param msg The message to decode
 * @param huch_sensor_array C-struct to decode the message contents into
 */
static inline void mavlink_msg_huch_sensor_array_decode(const mavlink_message_t* msg, mavlink_huch_sensor_array_t* huch_sensor_array)
{
#if MAVLINK_NEED_BYTE_SWAP
	huch_sensor_array->usec = mavlink_msg_huch_sensor_array_get_usec(msg);
	mavlink_msg_huch_sensor_array_get_data(msg, huch_sensor_array->data);
#else
	memcpy(huch_sensor_array, _MAV_PAYLOAD(msg), 136);
#endif
}
