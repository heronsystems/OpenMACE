#pragma once
// MESSAGE GLOBAL_POSITION_INT PACKING

#define MACE_MSG_ID_GLOBAL_POSITION_INT 23

MACEPACKED(
typedef struct __mace_global_position_int_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 int32_t lat; /*< Latitude, expressed as degrees * 1E7*/
 int32_t lon; /*< Longitude, expressed as degrees * 1E7*/
 int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
 int32_t relative_alt; /*< Altitude above ground in meters, expressed as * 1000 (millimeters)*/
 uint16_t hdg; /*< Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
}) mace_global_position_int_t;

#define MACE_MSG_ID_GLOBAL_POSITION_INT_LEN 22
#define MACE_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN 22
#define MACE_MSG_ID_23_LEN 22
#define MACE_MSG_ID_23_MIN_LEN 22

#define MACE_MSG_ID_GLOBAL_POSITION_INT_CRC 187
#define MACE_MSG_ID_23_CRC 187



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_GLOBAL_POSITION_INT { \
    23, \
    "GLOBAL_POSITION_INT", \
    6, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_global_position_int_t, time_boot_ms) }, \
         { "lat", NULL, MACE_TYPE_INT32_T, 0, 4, offsetof(mace_global_position_int_t, lat) }, \
         { "lon", NULL, MACE_TYPE_INT32_T, 0, 8, offsetof(mace_global_position_int_t, lon) }, \
         { "alt", NULL, MACE_TYPE_INT32_T, 0, 12, offsetof(mace_global_position_int_t, alt) }, \
         { "relative_alt", NULL, MACE_TYPE_INT32_T, 0, 16, offsetof(mace_global_position_int_t, relative_alt) }, \
         { "hdg", NULL, MACE_TYPE_UINT16_T, 0, 20, offsetof(mace_global_position_int_t, hdg) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_GLOBAL_POSITION_INT { \
    "GLOBAL_POSITION_INT", \
    6, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_global_position_int_t, time_boot_ms) }, \
         { "lat", NULL, MACE_TYPE_INT32_T, 0, 4, offsetof(mace_global_position_int_t, lat) }, \
         { "lon", NULL, MACE_TYPE_INT32_T, 0, 8, offsetof(mace_global_position_int_t, lon) }, \
         { "alt", NULL, MACE_TYPE_INT32_T, 0, 12, offsetof(mace_global_position_int_t, alt) }, \
         { "relative_alt", NULL, MACE_TYPE_INT32_T, 0, 16, offsetof(mace_global_position_int_t, relative_alt) }, \
         { "hdg", NULL, MACE_TYPE_UINT16_T, 0, 20, offsetof(mace_global_position_int_t, hdg) }, \
         } \
}
#endif

/**
 * @brief Pack a global_position_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as degrees * 1E7
 * @param lon Longitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param hdg Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_global_position_int_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, uint16_t hdg)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GLOBAL_POSITION_INT_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_int32_t(buf, 4, lat);
    _mace_put_int32_t(buf, 8, lon);
    _mace_put_int32_t(buf, 12, alt);
    _mace_put_int32_t(buf, 16, relative_alt);
    _mace_put_uint16_t(buf, 20, hdg);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GLOBAL_POSITION_INT_LEN);
#else
    mace_global_position_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.hdg = hdg;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GLOBAL_POSITION_INT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GLOBAL_POSITION_INT;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_CRC);
}

/**
 * @brief Pack a global_position_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as degrees * 1E7
 * @param lon Longitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param hdg Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_global_position_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint32_t time_boot_ms,int32_t lat,int32_t lon,int32_t alt,int32_t relative_alt,uint16_t hdg)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GLOBAL_POSITION_INT_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_int32_t(buf, 4, lat);
    _mace_put_int32_t(buf, 8, lon);
    _mace_put_int32_t(buf, 12, alt);
    _mace_put_int32_t(buf, 16, relative_alt);
    _mace_put_uint16_t(buf, 20, hdg);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GLOBAL_POSITION_INT_LEN);
#else
    mace_global_position_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.hdg = hdg;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GLOBAL_POSITION_INT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GLOBAL_POSITION_INT;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_CRC);
}

/**
 * @brief Encode a global_position_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int C-struct to read the message contents from
 */
static inline uint16_t mace_msg_global_position_int_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_global_position_int_t* global_position_int)
{
    return mace_msg_global_position_int_pack(system_id, component_id, msg, global_position_int->time_boot_ms, global_position_int->lat, global_position_int->lon, global_position_int->alt, global_position_int->relative_alt, global_position_int->hdg);
}

/**
 * @brief Encode a global_position_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int C-struct to read the message contents from
 */
static inline uint16_t mace_msg_global_position_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_global_position_int_t* global_position_int)
{
    return mace_msg_global_position_int_pack_chan(system_id, component_id, chan, msg, global_position_int->time_boot_ms, global_position_int->lat, global_position_int->lon, global_position_int->alt, global_position_int->relative_alt, global_position_int->hdg);
}

/**
 * @brief Send a global_position_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as degrees * 1E7
 * @param lon Longitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param hdg Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_global_position_int_send(mace_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, uint16_t hdg)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GLOBAL_POSITION_INT_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_int32_t(buf, 4, lat);
    _mace_put_int32_t(buf, 8, lon);
    _mace_put_int32_t(buf, 12, alt);
    _mace_put_int32_t(buf, 16, relative_alt);
    _mace_put_uint16_t(buf, 20, hdg);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_POSITION_INT, buf, MACE_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_CRC);
#else
    mace_global_position_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.hdg = hdg;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_POSITION_INT, (const char *)&packet, MACE_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_CRC);
#endif
}

/**
 * @brief Send a global_position_int message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_global_position_int_send_struct(mace_channel_t chan, const mace_global_position_int_t* global_position_int)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_global_position_int_send(chan, global_position_int->time_boot_ms, global_position_int->lat, global_position_int->lon, global_position_int->alt, global_position_int->relative_alt, global_position_int->hdg);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_POSITION_INT, (const char *)global_position_int, MACE_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_CRC);
#endif
}

#if MACE_MSG_ID_GLOBAL_POSITION_INT_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_global_position_int_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, uint16_t hdg)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_int32_t(buf, 4, lat);
    _mace_put_int32_t(buf, 8, lon);
    _mace_put_int32_t(buf, 12, alt);
    _mace_put_int32_t(buf, 16, relative_alt);
    _mace_put_uint16_t(buf, 20, hdg);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_POSITION_INT, buf, MACE_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_CRC);
#else
    mace_global_position_int_t *packet = (mace_global_position_int_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->relative_alt = relative_alt;
    packet->hdg = hdg;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_POSITION_INT, (const char *)packet, MACE_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_LEN, MACE_MSG_ID_GLOBAL_POSITION_INT_CRC);
#endif
}
#endif

#endif

// MESSAGE GLOBAL_POSITION_INT UNPACKING


/**
 * @brief Get field time_boot_ms from global_position_int message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mace_msg_global_position_int_get_time_boot_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from global_position_int message
 *
 * @return Latitude, expressed as degrees * 1E7
 */
static inline int32_t mace_msg_global_position_int_get_lat(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from global_position_int message
 *
 * @return Longitude, expressed as degrees * 1E7
 */
static inline int32_t mace_msg_global_position_int_get_lon(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from global_position_int message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 */
static inline int32_t mace_msg_global_position_int_get_alt(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field relative_alt from global_position_int message
 *
 * @return Altitude above ground in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mace_msg_global_position_int_get_relative_alt(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field hdg from global_position_int message
 *
 * @return Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mace_msg_global_position_int_get_hdg(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Decode a global_position_int message into a struct
 *
 * @param msg The message to decode
 * @param global_position_int C-struct to decode the message contents into
 */
static inline void mace_msg_global_position_int_decode(const mace_message_t* msg, mace_global_position_int_t* global_position_int)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    global_position_int->time_boot_ms = mace_msg_global_position_int_get_time_boot_ms(msg);
    global_position_int->lat = mace_msg_global_position_int_get_lat(msg);
    global_position_int->lon = mace_msg_global_position_int_get_lon(msg);
    global_position_int->alt = mace_msg_global_position_int_get_alt(msg);
    global_position_int->relative_alt = mace_msg_global_position_int_get_relative_alt(msg);
    global_position_int->hdg = mace_msg_global_position_int_get_hdg(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_GLOBAL_POSITION_INT_LEN? msg->len : MACE_MSG_ID_GLOBAL_POSITION_INT_LEN;
        memset(global_position_int, 0, MACE_MSG_ID_GLOBAL_POSITION_INT_LEN);
    memcpy(global_position_int, _MACE_PAYLOAD(msg), len);
#endif
}
