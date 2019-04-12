#pragma once
// MESSAGE DISTANCE_SENSOR PACKING

#define MACE_MSG_ID_DISTANCE_SENSOR 203

MACEPACKED(
typedef struct __mace_distance_sensor_t {
 uint32_t time_boot_ms; /*< Time since system boot*/
 uint16_t min_distance; /*< Minimum distance the sensor can measure in centimeters*/
 uint16_t max_distance; /*< Maximum distance the sensor can measure in centimeters*/
 uint16_t current_distance; /*< Current distance reading*/
 uint8_t type; /*< Type from MAV_DISTANCE_SENSOR enum.*/
 uint8_t id; /*< Onboard ID of the sensor*/
 uint8_t orientation; /*< Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.*/
 uint8_t covariance; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/
}) mace_distance_sensor_t;

#define MACE_MSG_ID_DISTANCE_SENSOR_LEN 14
#define MACE_MSG_ID_DISTANCE_SENSOR_MIN_LEN 14
#define MACE_MSG_ID_203_LEN 14
#define MACE_MSG_ID_203_MIN_LEN 14

#define MACE_MSG_ID_DISTANCE_SENSOR_CRC 85
#define MACE_MSG_ID_203_CRC 85



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_DISTANCE_SENSOR { \
    203, \
    "DISTANCE_SENSOR", \
    8, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_distance_sensor_t, time_boot_ms) }, \
         { "min_distance", NULL, MACE_TYPE_UINT16_T, 0, 4, offsetof(mace_distance_sensor_t, min_distance) }, \
         { "max_distance", NULL, MACE_TYPE_UINT16_T, 0, 6, offsetof(mace_distance_sensor_t, max_distance) }, \
         { "current_distance", NULL, MACE_TYPE_UINT16_T, 0, 8, offsetof(mace_distance_sensor_t, current_distance) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 10, offsetof(mace_distance_sensor_t, type) }, \
         { "id", NULL, MACE_TYPE_UINT8_T, 0, 11, offsetof(mace_distance_sensor_t, id) }, \
         { "orientation", NULL, MACE_TYPE_UINT8_T, 0, 12, offsetof(mace_distance_sensor_t, orientation) }, \
         { "covariance", NULL, MACE_TYPE_UINT8_T, 0, 13, offsetof(mace_distance_sensor_t, covariance) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_DISTANCE_SENSOR { \
    "DISTANCE_SENSOR", \
    8, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_distance_sensor_t, time_boot_ms) }, \
         { "min_distance", NULL, MACE_TYPE_UINT16_T, 0, 4, offsetof(mace_distance_sensor_t, min_distance) }, \
         { "max_distance", NULL, MACE_TYPE_UINT16_T, 0, 6, offsetof(mace_distance_sensor_t, max_distance) }, \
         { "current_distance", NULL, MACE_TYPE_UINT16_T, 0, 8, offsetof(mace_distance_sensor_t, current_distance) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 10, offsetof(mace_distance_sensor_t, type) }, \
         { "id", NULL, MACE_TYPE_UINT8_T, 0, 11, offsetof(mace_distance_sensor_t, id) }, \
         { "orientation", NULL, MACE_TYPE_UINT8_T, 0, 12, offsetof(mace_distance_sensor_t, orientation) }, \
         { "covariance", NULL, MACE_TYPE_UINT8_T, 0, 13, offsetof(mace_distance_sensor_t, covariance) }, \
         } \
}
#endif

/**
 * @brief Pack a distance_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Time since system boot
 * @param min_distance Minimum distance the sensor can measure in centimeters
 * @param max_distance Maximum distance the sensor can measure in centimeters
 * @param current_distance Current distance reading
 * @param type Type from MAV_DISTANCE_SENSOR enum.
 * @param id Onboard ID of the sensor
 * @param orientation Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.
 * @param covariance Measurement covariance in centimeters, 0 for unknown / invalid readings
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_distance_sensor_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_DISTANCE_SENSOR_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_uint16_t(buf, 4, min_distance);
    _mace_put_uint16_t(buf, 6, max_distance);
    _mace_put_uint16_t(buf, 8, current_distance);
    _mace_put_uint8_t(buf, 10, type);
    _mace_put_uint8_t(buf, 11, id);
    _mace_put_uint8_t(buf, 12, orientation);
    _mace_put_uint8_t(buf, 13, covariance);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_DISTANCE_SENSOR_LEN);
#else
    mace_distance_sensor_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.min_distance = min_distance;
    packet.max_distance = max_distance;
    packet.current_distance = current_distance;
    packet.type = type;
    packet.id = id;
    packet.orientation = orientation;
    packet.covariance = covariance;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_DISTANCE_SENSOR_LEN);
#endif

    msg->msgid = MACE_MSG_ID_DISTANCE_SENSOR;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MACE_MSG_ID_DISTANCE_SENSOR_LEN, MACE_MSG_ID_DISTANCE_SENSOR_CRC);
}

/**
 * @brief Pack a distance_sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Time since system boot
 * @param min_distance Minimum distance the sensor can measure in centimeters
 * @param max_distance Maximum distance the sensor can measure in centimeters
 * @param current_distance Current distance reading
 * @param type Type from MAV_DISTANCE_SENSOR enum.
 * @param id Onboard ID of the sensor
 * @param orientation Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.
 * @param covariance Measurement covariance in centimeters, 0 for unknown / invalid readings
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_distance_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint32_t time_boot_ms,uint16_t min_distance,uint16_t max_distance,uint16_t current_distance,uint8_t type,uint8_t id,uint8_t orientation,uint8_t covariance)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_DISTANCE_SENSOR_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_uint16_t(buf, 4, min_distance);
    _mace_put_uint16_t(buf, 6, max_distance);
    _mace_put_uint16_t(buf, 8, current_distance);
    _mace_put_uint8_t(buf, 10, type);
    _mace_put_uint8_t(buf, 11, id);
    _mace_put_uint8_t(buf, 12, orientation);
    _mace_put_uint8_t(buf, 13, covariance);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_DISTANCE_SENSOR_LEN);
#else
    mace_distance_sensor_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.min_distance = min_distance;
    packet.max_distance = max_distance;
    packet.current_distance = current_distance;
    packet.type = type;
    packet.id = id;
    packet.orientation = orientation;
    packet.covariance = covariance;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_DISTANCE_SENSOR_LEN);
#endif

    msg->msgid = MACE_MSG_ID_DISTANCE_SENSOR;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MACE_MSG_ID_DISTANCE_SENSOR_LEN, MACE_MSG_ID_DISTANCE_SENSOR_CRC);
}

/**
 * @brief Encode a distance_sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param distance_sensor C-struct to read the message contents from
 */
static inline uint16_t mace_msg_distance_sensor_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_distance_sensor_t* distance_sensor)
{
    return mace_msg_distance_sensor_pack(system_id, component_id, msg, distance_sensor->time_boot_ms, distance_sensor->min_distance, distance_sensor->max_distance, distance_sensor->current_distance, distance_sensor->type, distance_sensor->id, distance_sensor->orientation, distance_sensor->covariance);
}

/**
 * @brief Encode a distance_sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param distance_sensor C-struct to read the message contents from
 */
static inline uint16_t mace_msg_distance_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_distance_sensor_t* distance_sensor)
{
    return mace_msg_distance_sensor_pack_chan(system_id, component_id, chan, msg, distance_sensor->time_boot_ms, distance_sensor->min_distance, distance_sensor->max_distance, distance_sensor->current_distance, distance_sensor->type, distance_sensor->id, distance_sensor->orientation, distance_sensor->covariance);
}

/**
 * @brief Send a distance_sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Time since system boot
 * @param min_distance Minimum distance the sensor can measure in centimeters
 * @param max_distance Maximum distance the sensor can measure in centimeters
 * @param current_distance Current distance reading
 * @param type Type from MAV_DISTANCE_SENSOR enum.
 * @param id Onboard ID of the sensor
 * @param orientation Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.
 * @param covariance Measurement covariance in centimeters, 0 for unknown / invalid readings
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_distance_sensor_send(mace_channel_t chan, uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_DISTANCE_SENSOR_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_uint16_t(buf, 4, min_distance);
    _mace_put_uint16_t(buf, 6, max_distance);
    _mace_put_uint16_t(buf, 8, current_distance);
    _mace_put_uint8_t(buf, 10, type);
    _mace_put_uint8_t(buf, 11, id);
    _mace_put_uint8_t(buf, 12, orientation);
    _mace_put_uint8_t(buf, 13, covariance);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_DISTANCE_SENSOR, buf, MACE_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MACE_MSG_ID_DISTANCE_SENSOR_LEN, MACE_MSG_ID_DISTANCE_SENSOR_CRC);
#else
    mace_distance_sensor_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.min_distance = min_distance;
    packet.max_distance = max_distance;
    packet.current_distance = current_distance;
    packet.type = type;
    packet.id = id;
    packet.orientation = orientation;
    packet.covariance = covariance;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_DISTANCE_SENSOR, (const char *)&packet, MACE_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MACE_MSG_ID_DISTANCE_SENSOR_LEN, MACE_MSG_ID_DISTANCE_SENSOR_CRC);
#endif
}

/**
 * @brief Send a distance_sensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_distance_sensor_send_struct(mace_channel_t chan, const mace_distance_sensor_t* distance_sensor)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_distance_sensor_send(chan, distance_sensor->time_boot_ms, distance_sensor->min_distance, distance_sensor->max_distance, distance_sensor->current_distance, distance_sensor->type, distance_sensor->id, distance_sensor->orientation, distance_sensor->covariance);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_DISTANCE_SENSOR, (const char *)distance_sensor, MACE_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MACE_MSG_ID_DISTANCE_SENSOR_LEN, MACE_MSG_ID_DISTANCE_SENSOR_CRC);
#endif
}

#if MACE_MSG_ID_DISTANCE_SENSOR_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_distance_sensor_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_uint16_t(buf, 4, min_distance);
    _mace_put_uint16_t(buf, 6, max_distance);
    _mace_put_uint16_t(buf, 8, current_distance);
    _mace_put_uint8_t(buf, 10, type);
    _mace_put_uint8_t(buf, 11, id);
    _mace_put_uint8_t(buf, 12, orientation);
    _mace_put_uint8_t(buf, 13, covariance);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_DISTANCE_SENSOR, buf, MACE_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MACE_MSG_ID_DISTANCE_SENSOR_LEN, MACE_MSG_ID_DISTANCE_SENSOR_CRC);
#else
    mace_distance_sensor_t *packet = (mace_distance_sensor_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->min_distance = min_distance;
    packet->max_distance = max_distance;
    packet->current_distance = current_distance;
    packet->type = type;
    packet->id = id;
    packet->orientation = orientation;
    packet->covariance = covariance;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_DISTANCE_SENSOR, (const char *)packet, MACE_MSG_ID_DISTANCE_SENSOR_MIN_LEN, MACE_MSG_ID_DISTANCE_SENSOR_LEN, MACE_MSG_ID_DISTANCE_SENSOR_CRC);
#endif
}
#endif

#endif

// MESSAGE DISTANCE_SENSOR UNPACKING


/**
 * @brief Get field time_boot_ms from distance_sensor message
 *
 * @return Time since system boot
 */
static inline uint32_t mace_msg_distance_sensor_get_time_boot_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field min_distance from distance_sensor message
 *
 * @return Minimum distance the sensor can measure in centimeters
 */
static inline uint16_t mace_msg_distance_sensor_get_min_distance(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field max_distance from distance_sensor message
 *
 * @return Maximum distance the sensor can measure in centimeters
 */
static inline uint16_t mace_msg_distance_sensor_get_max_distance(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field current_distance from distance_sensor message
 *
 * @return Current distance reading
 */
static inline uint16_t mace_msg_distance_sensor_get_current_distance(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field type from distance_sensor message
 *
 * @return Type from MAV_DISTANCE_SENSOR enum.
 */
static inline uint8_t mace_msg_distance_sensor_get_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field id from distance_sensor message
 *
 * @return Onboard ID of the sensor
 */
static inline uint8_t mace_msg_distance_sensor_get_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field orientation from distance_sensor message
 *
 * @return Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.
 */
static inline uint8_t mace_msg_distance_sensor_get_orientation(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field covariance from distance_sensor message
 *
 * @return Measurement covariance in centimeters, 0 for unknown / invalid readings
 */
static inline uint8_t mace_msg_distance_sensor_get_covariance(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a distance_sensor message into a struct
 *
 * @param msg The message to decode
 * @param distance_sensor C-struct to decode the message contents into
 */
static inline void mace_msg_distance_sensor_decode(const mace_message_t* msg, mace_distance_sensor_t* distance_sensor)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    distance_sensor->time_boot_ms = mace_msg_distance_sensor_get_time_boot_ms(msg);
    distance_sensor->min_distance = mace_msg_distance_sensor_get_min_distance(msg);
    distance_sensor->max_distance = mace_msg_distance_sensor_get_max_distance(msg);
    distance_sensor->current_distance = mace_msg_distance_sensor_get_current_distance(msg);
    distance_sensor->type = mace_msg_distance_sensor_get_type(msg);
    distance_sensor->id = mace_msg_distance_sensor_get_id(msg);
    distance_sensor->orientation = mace_msg_distance_sensor_get_orientation(msg);
    distance_sensor->covariance = mace_msg_distance_sensor_get_covariance(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_DISTANCE_SENSOR_LEN? msg->len : MACE_MSG_ID_DISTANCE_SENSOR_LEN;
        memset(distance_sensor, 0, MACE_MSG_ID_DISTANCE_SENSOR_LEN);
    memcpy(distance_sensor, _MACE_PAYLOAD(msg), len);
#endif
}
