#pragma once
// MESSAGE GPS_GLOBAL_ORIGIN PACKING

#define MACE_MSG_ID_GPS_GLOBAL_ORIGIN 27

MACEPACKED(
typedef struct __mace_gps_global_origin_t {
 int32_t latitude; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t longitude; /*< Longitude (WGS84), in degrees * 1E7*/
 int32_t altitude; /*< Altitude (AMSL), in meters * 1000 (positive for up)*/
}) mace_gps_global_origin_t;

#define MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN 12
#define MACE_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN 12
#define MACE_MSG_ID_27_LEN 12
#define MACE_MSG_ID_27_MIN_LEN 12

#define MACE_MSG_ID_GPS_GLOBAL_ORIGIN_CRC 39
#define MACE_MSG_ID_27_CRC 39



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_GPS_GLOBAL_ORIGIN { \
    27, \
    "GPS_GLOBAL_ORIGIN", \
    3, \
    {  { "latitude", NULL, MACE_TYPE_INT32_T, 0, 0, offsetof(mace_gps_global_origin_t, latitude) }, \
         { "longitude", NULL, MACE_TYPE_INT32_T, 0, 4, offsetof(mace_gps_global_origin_t, longitude) }, \
         { "altitude", NULL, MACE_TYPE_INT32_T, 0, 8, offsetof(mace_gps_global_origin_t, altitude) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_GPS_GLOBAL_ORIGIN { \
    "GPS_GLOBAL_ORIGIN", \
    3, \
    {  { "latitude", NULL, MACE_TYPE_INT32_T, 0, 0, offsetof(mace_gps_global_origin_t, latitude) }, \
         { "longitude", NULL, MACE_TYPE_INT32_T, 0, 4, offsetof(mace_gps_global_origin_t, longitude) }, \
         { "altitude", NULL, MACE_TYPE_INT32_T, 0, 8, offsetof(mace_gps_global_origin_t, altitude) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_global_origin message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84), in degrees * 1E7
 * @param altitude Altitude (AMSL), in meters * 1000 (positive for up)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_gps_global_origin_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               int32_t latitude, int32_t longitude, int32_t altitude)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN];
    _mace_put_int32_t(buf, 0, latitude);
    _mace_put_int32_t(buf, 4, longitude);
    _mace_put_int32_t(buf, 8, altitude);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN);
#else
    mace_gps_global_origin_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GPS_GLOBAL_ORIGIN;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
}

/**
 * @brief Pack a gps_global_origin message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84), in degrees * 1E7
 * @param altitude Altitude (AMSL), in meters * 1000 (positive for up)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_gps_global_origin_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   int32_t latitude,int32_t longitude,int32_t altitude)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN];
    _mace_put_int32_t(buf, 0, latitude);
    _mace_put_int32_t(buf, 4, longitude);
    _mace_put_int32_t(buf, 8, altitude);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN);
#else
    mace_gps_global_origin_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GPS_GLOBAL_ORIGIN;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
}

/**
 * @brief Encode a gps_global_origin struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_global_origin C-struct to read the message contents from
 */
static inline uint16_t mace_msg_gps_global_origin_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_gps_global_origin_t* gps_global_origin)
{
    return mace_msg_gps_global_origin_pack(system_id, component_id, msg, gps_global_origin->latitude, gps_global_origin->longitude, gps_global_origin->altitude);
}

/**
 * @brief Encode a gps_global_origin struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_global_origin C-struct to read the message contents from
 */
static inline uint16_t mace_msg_gps_global_origin_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_gps_global_origin_t* gps_global_origin)
{
    return mace_msg_gps_global_origin_pack_chan(system_id, component_id, chan, msg, gps_global_origin->latitude, gps_global_origin->longitude, gps_global_origin->altitude);
}

/**
 * @brief Send a gps_global_origin message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84), in degrees * 1E7
 * @param altitude Altitude (AMSL), in meters * 1000 (positive for up)
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_gps_global_origin_send(mace_channel_t chan, int32_t latitude, int32_t longitude, int32_t altitude)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN];
    _mace_put_int32_t(buf, 0, latitude);
    _mace_put_int32_t(buf, 4, longitude);
    _mace_put_int32_t(buf, 8, altitude);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_GLOBAL_ORIGIN, buf, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
#else
    mace_gps_global_origin_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_GLOBAL_ORIGIN, (const char *)&packet, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
#endif
}

/**
 * @brief Send a gps_global_origin message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_gps_global_origin_send_struct(mace_channel_t chan, const mace_gps_global_origin_t* gps_global_origin)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_gps_global_origin_send(chan, gps_global_origin->latitude, gps_global_origin->longitude, gps_global_origin->altitude);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_GLOBAL_ORIGIN, (const char *)gps_global_origin, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
#endif
}

#if MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_gps_global_origin_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  int32_t latitude, int32_t longitude, int32_t altitude)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_int32_t(buf, 0, latitude);
    _mace_put_int32_t(buf, 4, longitude);
    _mace_put_int32_t(buf, 8, altitude);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_GLOBAL_ORIGIN, buf, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
#else
    mace_gps_global_origin_t *packet = (mace_gps_global_origin_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_GLOBAL_ORIGIN, (const char *)packet, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_GLOBAL_ORIGIN UNPACKING


/**
 * @brief Get field latitude from gps_global_origin message
 *
 * @return Latitude (WGS84), in degrees * 1E7
 */
static inline int32_t mace_msg_gps_global_origin_get_latitude(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from gps_global_origin message
 *
 * @return Longitude (WGS84), in degrees * 1E7
 */
static inline int32_t mace_msg_gps_global_origin_get_longitude(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altitude from gps_global_origin message
 *
 * @return Altitude (AMSL), in meters * 1000 (positive for up)
 */
static inline int32_t mace_msg_gps_global_origin_get_altitude(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a gps_global_origin message into a struct
 *
 * @param msg The message to decode
 * @param gps_global_origin C-struct to decode the message contents into
 */
static inline void mace_msg_gps_global_origin_decode(const mace_message_t* msg, mace_gps_global_origin_t* gps_global_origin)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    gps_global_origin->latitude = mace_msg_gps_global_origin_get_latitude(msg);
    gps_global_origin->longitude = mace_msg_gps_global_origin_get_longitude(msg);
    gps_global_origin->altitude = mace_msg_gps_global_origin_get_altitude(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN? msg->len : MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN;
        memset(gps_global_origin, 0, MACE_MSG_ID_GPS_GLOBAL_ORIGIN_LEN);
    memcpy(gps_global_origin, _MACE_PAYLOAD(msg), len);
#endif
}
