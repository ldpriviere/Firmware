// MESSAGE MPPT_STATUS PACKING

#define MAVLINK_MSG_ID_MPPT_STATUS 157

typedef struct __mavlink_mppt_status_t
{
 uint16_t mpptTemperature; /*< MPPT temperature in C.*/
 uint16_t solarCurrent; /*< Available solar current.*/
 uint16_t totalCurrent; /*< Total current consumption.*/
 uint16_t batteryVoltage; /*< Total battery voltage.*/
 uint8_t dutyCycleMin; /*< Min duty cycle.*/
 uint8_t dutyCycleMax; /*< Max dudy cycle.*/
} mavlink_mppt_status_t;

#define MAVLINK_MSG_ID_MPPT_STATUS_LEN 10
#define MAVLINK_MSG_ID_157_LEN 10

#define MAVLINK_MSG_ID_MPPT_STATUS_CRC 184
#define MAVLINK_MSG_ID_157_CRC 184



#define MAVLINK_MESSAGE_INFO_MPPT_STATUS { \
	"MPPT_STATUS", \
	6, \
	{  { "mpptTemperature", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mppt_status_t, mpptTemperature) }, \
         { "solarCurrent", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_mppt_status_t, solarCurrent) }, \
         { "totalCurrent", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_mppt_status_t, totalCurrent) }, \
         { "batteryVoltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_mppt_status_t, batteryVoltage) }, \
         { "dutyCycleMin", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_mppt_status_t, dutyCycleMin) }, \
         { "dutyCycleMax", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_mppt_status_t, dutyCycleMax) }, \
         } \
}


/**
 * @brief Pack a mppt_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mpptTemperature MPPT temperature in C.
 * @param solarCurrent Available solar current.
 * @param totalCurrent Total current consumption.
 * @param batteryVoltage Total battery voltage.
 * @param dutyCycleMin Min duty cycle.
 * @param dutyCycleMax Max dudy cycle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mppt_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t mpptTemperature, uint16_t solarCurrent, uint16_t totalCurrent, uint16_t batteryVoltage, uint8_t dutyCycleMin, uint8_t dutyCycleMax)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MPPT_STATUS_LEN];
	_mav_put_uint16_t(buf, 0, mpptTemperature);
	_mav_put_uint16_t(buf, 2, solarCurrent);
	_mav_put_uint16_t(buf, 4, totalCurrent);
	_mav_put_uint16_t(buf, 6, batteryVoltage);
	_mav_put_uint8_t(buf, 8, dutyCycleMin);
	_mav_put_uint8_t(buf, 9, dutyCycleMax);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#else
	mavlink_mppt_status_t packet;
	packet.mpptTemperature = mpptTemperature;
	packet.solarCurrent = solarCurrent;
	packet.totalCurrent = totalCurrent;
	packet.batteryVoltage = batteryVoltage;
	packet.dutyCycleMin = dutyCycleMin;
	packet.dutyCycleMax = dutyCycleMax;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MPPT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif
}

/**
 * @brief Pack a mppt_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mpptTemperature MPPT temperature in C.
 * @param solarCurrent Available solar current.
 * @param totalCurrent Total current consumption.
 * @param batteryVoltage Total battery voltage.
 * @param dutyCycleMin Min duty cycle.
 * @param dutyCycleMax Max dudy cycle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mppt_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t mpptTemperature,uint16_t solarCurrent,uint16_t totalCurrent,uint16_t batteryVoltage,uint8_t dutyCycleMin,uint8_t dutyCycleMax)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MPPT_STATUS_LEN];
	_mav_put_uint16_t(buf, 0, mpptTemperature);
	_mav_put_uint16_t(buf, 2, solarCurrent);
	_mav_put_uint16_t(buf, 4, totalCurrent);
	_mav_put_uint16_t(buf, 6, batteryVoltage);
	_mav_put_uint8_t(buf, 8, dutyCycleMin);
	_mav_put_uint8_t(buf, 9, dutyCycleMax);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#else
	mavlink_mppt_status_t packet;
	packet.mpptTemperature = mpptTemperature;
	packet.solarCurrent = solarCurrent;
	packet.totalCurrent = totalCurrent;
	packet.batteryVoltage = batteryVoltage;
	packet.dutyCycleMin = dutyCycleMin;
	packet.dutyCycleMax = dutyCycleMax;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MPPT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif
}

/**
 * @brief Encode a mppt_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mppt_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mppt_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mppt_status_t* mppt_status)
{
	return mavlink_msg_mppt_status_pack(system_id, component_id, msg, mppt_status->mpptTemperature, mppt_status->solarCurrent, mppt_status->totalCurrent, mppt_status->batteryVoltage, mppt_status->dutyCycleMin, mppt_status->dutyCycleMax);
}

/**
 * @brief Encode a mppt_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mppt_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mppt_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mppt_status_t* mppt_status)
{
	return mavlink_msg_mppt_status_pack_chan(system_id, component_id, chan, msg, mppt_status->mpptTemperature, mppt_status->solarCurrent, mppt_status->totalCurrent, mppt_status->batteryVoltage, mppt_status->dutyCycleMin, mppt_status->dutyCycleMax);
}

/**
 * @brief Send a mppt_status message
 * @param chan MAVLink channel to send the message
 *
 * @param mpptTemperature MPPT temperature in C.
 * @param solarCurrent Available solar current.
 * @param totalCurrent Total current consumption.
 * @param batteryVoltage Total battery voltage.
 * @param dutyCycleMin Min duty cycle.
 * @param dutyCycleMax Max dudy cycle.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mppt_status_send(mavlink_channel_t chan, uint16_t mpptTemperature, uint16_t solarCurrent, uint16_t totalCurrent, uint16_t batteryVoltage, uint8_t dutyCycleMin, uint8_t dutyCycleMax)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MPPT_STATUS_LEN];
	_mav_put_uint16_t(buf, 0, mpptTemperature);
	_mav_put_uint16_t(buf, 2, solarCurrent);
	_mav_put_uint16_t(buf, 4, totalCurrent);
	_mav_put_uint16_t(buf, 6, batteryVoltage);
	_mav_put_uint8_t(buf, 8, dutyCycleMin);
	_mav_put_uint8_t(buf, 9, dutyCycleMax);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, buf, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, buf, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif
#else
	mavlink_mppt_status_t packet;
	packet.mpptTemperature = mpptTemperature;
	packet.solarCurrent = solarCurrent;
	packet.totalCurrent = totalCurrent;
	packet.batteryVoltage = batteryVoltage;
	packet.dutyCycleMin = dutyCycleMin;
	packet.dutyCycleMax = dutyCycleMax;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MPPT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mppt_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t mpptTemperature, uint16_t solarCurrent, uint16_t totalCurrent, uint16_t batteryVoltage, uint8_t dutyCycleMin, uint8_t dutyCycleMax)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, mpptTemperature);
	_mav_put_uint16_t(buf, 2, solarCurrent);
	_mav_put_uint16_t(buf, 4, totalCurrent);
	_mav_put_uint16_t(buf, 6, batteryVoltage);
	_mav_put_uint8_t(buf, 8, dutyCycleMin);
	_mav_put_uint8_t(buf, 9, dutyCycleMax);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, buf, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, buf, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif
#else
	mavlink_mppt_status_t *packet = (mavlink_mppt_status_t *)msgbuf;
	packet->mpptTemperature = mpptTemperature;
	packet->solarCurrent = solarCurrent;
	packet->totalCurrent = totalCurrent;
	packet->batteryVoltage = batteryVoltage;
	packet->dutyCycleMin = dutyCycleMin;
	packet->dutyCycleMax = dutyCycleMax;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, (const char *)packet, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, (const char *)packet, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MPPT_STATUS UNPACKING


/**
 * @brief Get field mpptTemperature from mppt_status message
 *
 * @return MPPT temperature in C.
 */
static inline uint16_t mavlink_msg_mppt_status_get_mpptTemperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field solarCurrent from mppt_status message
 *
 * @return Available solar current.
 */
static inline uint16_t mavlink_msg_mppt_status_get_solarCurrent(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field totalCurrent from mppt_status message
 *
 * @return Total current consumption.
 */
static inline uint16_t mavlink_msg_mppt_status_get_totalCurrent(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field batteryVoltage from mppt_status message
 *
 * @return Total battery voltage.
 */
static inline uint16_t mavlink_msg_mppt_status_get_batteryVoltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field dutyCycleMin from mppt_status message
 *
 * @return Min duty cycle.
 */
static inline uint8_t mavlink_msg_mppt_status_get_dutyCycleMin(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field dutyCycleMax from mppt_status message
 *
 * @return Max dudy cycle.
 */
static inline uint8_t mavlink_msg_mppt_status_get_dutyCycleMax(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Decode a mppt_status message into a struct
 *
 * @param msg The message to decode
 * @param mppt_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_mppt_status_decode(const mavlink_message_t* msg, mavlink_mppt_status_t* mppt_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	mppt_status->mpptTemperature = mavlink_msg_mppt_status_get_mpptTemperature(msg);
	mppt_status->solarCurrent = mavlink_msg_mppt_status_get_solarCurrent(msg);
	mppt_status->totalCurrent = mavlink_msg_mppt_status_get_totalCurrent(msg);
	mppt_status->batteryVoltage = mavlink_msg_mppt_status_get_batteryVoltage(msg);
	mppt_status->dutyCycleMin = mavlink_msg_mppt_status_get_dutyCycleMin(msg);
	mppt_status->dutyCycleMax = mavlink_msg_mppt_status_get_dutyCycleMax(msg);
#else
	memcpy(mppt_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif
}
