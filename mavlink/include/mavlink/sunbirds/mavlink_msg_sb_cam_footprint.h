// MESSAGE SB_CAM_FOOTPRINT PACKING

#define MAVLINK_MSG_ID_SB_CAM_FOOTPRINT 158

typedef struct __mavlink_sb_cam_footprint_t
{
 uint64_t timestamp; /*< Timestamp*/
 int32_t lat; /*< Latitude in degrees * 1E7*/
 int32_t lon; /*< Longitude in degrees * 1E7*/
 int32_t alt; /*< Altitude relative in meters * 1E3*/
 int32_t yaw; /*< Yaw in radians * 1E3*/
} mavlink_sb_cam_footprint_t;

#define MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN 24
#define MAVLINK_MSG_ID_158_LEN 24

#define MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_CRC 39
#define MAVLINK_MSG_ID_158_CRC 39



#define MAVLINK_MESSAGE_INFO_SB_CAM_FOOTPRINT { \
	"SB_CAM_FOOTPRINT", \
	5, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sb_cam_footprint_t, timestamp) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_sb_cam_footprint_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_sb_cam_footprint_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_sb_cam_footprint_t, alt) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_sb_cam_footprint_t, yaw) }, \
         } \
}


/**
 * @brief Pack a sb_cam_footprint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp
 * @param lat Latitude in degrees * 1E7
 * @param lon Longitude in degrees * 1E7
 * @param alt Altitude relative in meters * 1E3
 * @param yaw Yaw in radians * 1E3
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sb_cam_footprint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, int32_t lat, int32_t lon, int32_t alt, int32_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_int32_t(buf, 20, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN);
#else
	mavlink_sb_cam_footprint_t packet;
	packet.timestamp = timestamp;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SB_CAM_FOOTPRINT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN);
#endif
}

/**
 * @brief Pack a sb_cam_footprint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp
 * @param lat Latitude in degrees * 1E7
 * @param lon Longitude in degrees * 1E7
 * @param alt Altitude relative in meters * 1E3
 * @param yaw Yaw in radians * 1E3
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sb_cam_footprint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,int32_t lat,int32_t lon,int32_t alt,int32_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_int32_t(buf, 20, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN);
#else
	mavlink_sb_cam_footprint_t packet;
	packet.timestamp = timestamp;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SB_CAM_FOOTPRINT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN);
#endif
}

/**
 * @brief Encode a sb_cam_footprint struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sb_cam_footprint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sb_cam_footprint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sb_cam_footprint_t* sb_cam_footprint)
{
	return mavlink_msg_sb_cam_footprint_pack(system_id, component_id, msg, sb_cam_footprint->timestamp, sb_cam_footprint->lat, sb_cam_footprint->lon, sb_cam_footprint->alt, sb_cam_footprint->yaw);
}

/**
 * @brief Encode a sb_cam_footprint struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sb_cam_footprint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sb_cam_footprint_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sb_cam_footprint_t* sb_cam_footprint)
{
	return mavlink_msg_sb_cam_footprint_pack_chan(system_id, component_id, chan, msg, sb_cam_footprint->timestamp, sb_cam_footprint->lat, sb_cam_footprint->lon, sb_cam_footprint->alt, sb_cam_footprint->yaw);
}

/**
 * @brief Send a sb_cam_footprint message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp
 * @param lat Latitude in degrees * 1E7
 * @param lon Longitude in degrees * 1E7
 * @param alt Altitude relative in meters * 1E3
 * @param yaw Yaw in radians * 1E3
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sb_cam_footprint_send(mavlink_channel_t chan, uint64_t timestamp, int32_t lat, int32_t lon, int32_t alt, int32_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_int32_t(buf, 20, yaw);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT, buf, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT, buf, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN);
#endif
#else
	mavlink_sb_cam_footprint_t packet;
	packet.timestamp = timestamp;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.yaw = yaw;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT, (const char *)&packet, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT, (const char *)&packet, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sb_cam_footprint_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, int32_t lat, int32_t lon, int32_t alt, int32_t yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_int32_t(buf, 20, yaw);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT, buf, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT, buf, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN);
#endif
#else
	mavlink_sb_cam_footprint_t *packet = (mavlink_sb_cam_footprint_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;
	packet->yaw = yaw;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT, (const char *)packet, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT, (const char *)packet, MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SB_CAM_FOOTPRINT UNPACKING


/**
 * @brief Get field timestamp from sb_cam_footprint message
 *
 * @return Timestamp
 */
static inline uint64_t mavlink_msg_sb_cam_footprint_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field lat from sb_cam_footprint message
 *
 * @return Latitude in degrees * 1E7
 */
static inline int32_t mavlink_msg_sb_cam_footprint_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from sb_cam_footprint message
 *
 * @return Longitude in degrees * 1E7
 */
static inline int32_t mavlink_msg_sb_cam_footprint_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from sb_cam_footprint message
 *
 * @return Altitude relative in meters * 1E3
 */
static inline int32_t mavlink_msg_sb_cam_footprint_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field yaw from sb_cam_footprint message
 *
 * @return Yaw in radians * 1E3
 */
static inline int32_t mavlink_msg_sb_cam_footprint_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Decode a sb_cam_footprint message into a struct
 *
 * @param msg The message to decode
 * @param sb_cam_footprint C-struct to decode the message contents into
 */
static inline void mavlink_msg_sb_cam_footprint_decode(const mavlink_message_t* msg, mavlink_sb_cam_footprint_t* sb_cam_footprint)
{
#if MAVLINK_NEED_BYTE_SWAP
	sb_cam_footprint->timestamp = mavlink_msg_sb_cam_footprint_get_timestamp(msg);
	sb_cam_footprint->lat = mavlink_msg_sb_cam_footprint_get_lat(msg);
	sb_cam_footprint->lon = mavlink_msg_sb_cam_footprint_get_lon(msg);
	sb_cam_footprint->alt = mavlink_msg_sb_cam_footprint_get_alt(msg);
	sb_cam_footprint->yaw = mavlink_msg_sb_cam_footprint_get_yaw(msg);
#else
	memcpy(sb_cam_footprint, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SB_CAM_FOOTPRINT_LEN);
#endif
}
