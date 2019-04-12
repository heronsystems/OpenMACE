#pragma once
// MESSAGE GLOBAL_POSITION_STATE_FULL PACKING

#define MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL 25

MACEPACKED(
typedef struct __mace_global_position_state_full_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 int32_t lat; /*< Latitude, expressed as degrees * 1E7*/
 int32_t lon; /*< Longitude, expressed as degrees * 1E7*/
 int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
 int32_t relative_alt; /*< Altitude above ground in meters, expressed as * 1000 (millimeters)*/
 int16_t vx; /*< Ground X Speed (Latitude, positive north), expressed as m/s * 100*/
 int16_t vy; /*< Ground Y Speed (Longitude, positive east), expressed as m/s * 100*/
 int16_t vz; /*< Ground Z Speed (Altitude, positive down), expressed as m/s * 100*/
 uint16_t hdg; /*< Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
}) mace_global_position_state_full_t;

#define MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN 28
#define MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_MIN_LEN 28
#define MACE_MSG_ID_25_LEN 28
#define MACE_MSG_ID_25_MIN_LEN 28

#define MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_CRC 128
#define MACE_MSG_ID_25_CRC 128



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_GLOBAL_POSITION_STATE_FULL { \
    25, \
    "GLOBAL_POSITION_STATE_FULL", \
    9, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_global_position_state_full_t, time_boot_ms) }, \
         { "lat", NULL, MACE_TYPE_INT32_T, 0, 4, offsetof(mace_global_position_state_full_t, lat) }, \
         { "lon", NULL, MACE_TYPE_INT32_T, 0, 8, offsetof(mace_global_position_state_full_t, lon) }, \
         { "alt", NULL, MACE_TYPE_INT32_T, 0, 12, offsetof(mace_global_position_state_full_t, alt) }, \
         { "relative_alt", NULL, MACE_TYPE_INT32_T, 0, 16, offsetof(mace_global_position_state_full_t, relative_alt) }, \
         { "vx", NULL, MACE_TYPE_INT16_T, 0, 20, offsetof(mace_global_position_state_full_t, vx) }, \
         { "vy", NULL, MACE_TYPE_INT16_T, 0, 22, offsetof(mace_global_position_state_full_t, vy) }, \
         { "vz", NULL, MACE_TYPE_INT16_T, 0, 24, offsetof(mace_global_position_state_full_t, vz) }, \
         { "hdg", NULL, MACE_TYPE_UINT16_T, 0, 26, offsetof(mace_global_position_state_full_t, hdg) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_GLOBAL_POSITION_STATE_FULL { \
    "GLOBAL_POSITION_STATE_FULL", \
    9, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_global_position_state_full_t, time_boot_ms) }, \
         { "lat", NULL, MACE_TYPE_INT32_T, 0, 4, offsetof(mace_global_position_state_full_t, lat) }, \
         { "lon", NULL, MACE_TYPE_INT32_T, 0, 8, offsetof(mace_global_position_state_full_t, lon) }, \
         { "alt", NULL, MACE_TYPE_INT32_T, 0, 12, offsetof(mace_global_position_state_full_t, alt) }, \
         { "relative_alt", NULL, MACE_TYPE_INT32_T, 0, 16, offsetof(mace_global_position_state_full_t, relative_alt) }, \
         { "vx", NULL, MACE_TYPE_INT16_T, 0, 20, offsetof(mace_global_position_state_full_t, vx) }, \
         { "vy", NULL, MACE_TYPE_INT16_T, 0, 22, offsetof(mace_global_position_state_full_t, vy) }, \
         { "vz", NULL, MACE_TYPE_INT16_T, 0, 24, offsetof(mace_global_position_state_full_t, vz) }, \
         { "hdg", NULL, MACE_TYPE_UINT16_T, 0, 26, offsetof(mace_global_position_state_full_t, hdg) }, \
         } \
}
#endif

/**
 * @brief Pack a global_position_state_full message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as degrees * 1E7
 * @param lon Longitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param vx Ground X Speed (Latitude, positive north), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude, positive east), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude, positive down), expressed as m/s * 100
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param hdg Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_global_position_state_full_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int32_t relative_alt, uint16_t hdg)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_int32_t(buf, 4, lat);
    _mace_put_int32_t(buf, 8, lon);
    _mace_put_int32_t(buf, 12, alt);
    _mace_put_int32_t(buf, 16, relative_alt);
    _mace_put_int16_t(buf, 20, vx);
    _mace_put_int16_t(buf, 22, vy);
    _mace_put_int16_t(buf, 24, vz);
    _mace_put_uint16_t(buf, 26, hdg);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN);
#else
    mace_global_position_state_full_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.hdg = hdg;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_CRC);
}

/**
 * @brief Pack a global_position_state_full message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as degrees * 1E7
 * @param lon Longitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param vx Ground X Speed (Latitude, positive north), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude, positive east), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude, positive down), expressed as m/s * 100
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param hdg Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_global_position_state_full_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint32_t time_boot_ms,int32_t lat,int32_t lon,int32_t alt,int16_t vx,int16_t vy,int16_t vz,int32_t relative_alt,uint16_t hdg)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_int32_t(buf, 4, lat);
    _mace_put_int32_t(buf, 8, lon);
    _mace_put_int32_t(buf, 12, alt);
    _mace_put_int32_t(buf, 16, relative_alt);
    _mace_put_int16_t(buf, 20, vx);
    _mace_put_int16_t(buf, 22, vy);
    _mace_put_int16_t(buf, 24, vz);
    _mace_put_uint16_t(buf, 26, hdg);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN);
#else
    mace_global_position_state_full_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.hdg = hdg;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_CRC);
}

/**
 * @brief Encode a global_position_state_full struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_state_full C-struct to read the message contents from
 */
static inline uint16_t mace_msg_global_position_state_full_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_global_position_state_full_t* global_position_state_full)
{
    return mace_msg_global_position_state_full_pack(system_id, component_id, msg, global_position_state_full->time_boot_ms, global_position_state_full->lat, global_position_state_full->lon, global_position_state_full->alt, global_position_state_full->vx, global_position_state_full->vy, global_position_state_full->vz, global_position_state_full->relative_alt, global_position_state_full->hdg);
}

/**
 * @brief Encode a global_position_state_full struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_position_state_full C-struct to read the message contents from
 */
static inline uint16_t mace_msg_global_position_state_full_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_global_position_state_full_t* global_position_state_full)
{
    return mace_msg_global_position_state_full_pack_chan(system_id, component_id, chan, msg, global_position_state_full->time_boot_ms, global_position_state_full->lat, global_position_state_full->lon, global_position_state_full->alt, global_position_state_full->vx, global_position_state_full->vy, global_position_state_full->vz, global_position_state_full->relative_alt, global_position_state_full->hdg);
}

/**
 * @brief Send a global_position_state_full message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as degrees * 1E7
 * @param lon Longitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param vx Ground X Speed (Latitude, positive north), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude, positive east), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude, positive down), expressed as m/s * 100
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param hdg Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_global_position_state_full_send(mace_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int32_t relative_alt, uint16_t hdg)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_int32_t(buf, 4, lat);
    _mace_put_int32_t(buf, 8, lon);
    _mace_put_int32_t(buf, 12, alt);
    _mace_put_int32_t(buf, 16, relative_alt);
    _mace_put_int16_t(buf, 20, vx);
    _mace_put_int16_t(buf, 22, vy);
    _mace_put_int16_t(buf, 24, vz);
    _mace_put_uint16_t(buf, 26, hdg);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL, buf, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_CRC);
#else
    mace_global_position_state_full_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.hdg = hdg;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL, (const char *)&packet, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_CRC);
#endif
}

/**
 * @brief Send a global_position_state_full message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_global_position_state_full_send_struct(mace_channel_t chan, const mace_global_position_state_full_t* global_position_state_full)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_global_position_state_full_send(chan, global_position_state_full->time_boot_ms, global_position_state_full->lat, global_position_state_full->lon, global_position_state_full->alt, global_position_state_full->vx, global_position_state_full->vy, global_position_state_full->vz, global_position_state_full->relative_alt, global_position_state_full->hdg);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL, (const char *)global_position_state_full, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_CRC);
#endif
}

#if MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_global_position_state_full_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int32_t relative_alt, uint16_t hdg)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_int32_t(buf, 4, lat);
    _mace_put_int32_t(buf, 8, lon);
    _mace_put_int32_t(buf, 12, alt);
    _mace_put_int32_t(buf, 16, relative_alt);
    _mace_put_int16_t(buf, 20, vx);
    _mace_put_int16_t(buf, 22, vy);
    _mace_put_int16_t(buf, 24, vz);
    _mace_put_uint16_t(buf, 26, hdg);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL, buf, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_CRC);
#else
    mace_global_position_state_full_t *packet = (mace_global_position_state_full_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->relative_alt = relative_alt;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->hdg = hdg;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL, (const char *)packet, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_MIN_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_CRC);
#endif
}
#endif

#endif

// MESSAGE GLOBAL_POSITION_STATE_FULL UNPACKING


/**
 * @brief Get field time_boot_ms from global_position_state_full message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mace_msg_global_position_state_full_get_time_boot_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from global_position_state_full message
 *
 * @return Latitude, expressed as degrees * 1E7
 */
static inline int32_t mace_msg_global_position_state_full_get_lat(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from global_position_state_full message
 *
 * @return Longitude, expressed as degrees * 1E7
 */
static inline int32_t mace_msg_global_position_state_full_get_lon(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from global_position_state_full message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 */
static inline int32_t mace_msg_global_position_state_full_get_alt(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field vx from global_position_state_full message
 *
 * @return Ground X Speed (Latitude, positive north), expressed as m/s * 100
 */
static inline int16_t mace_msg_global_position_state_full_get_vx(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field vy from global_position_state_full message
 *
 * @return Ground Y Speed (Longitude, positive east), expressed as m/s * 100
 */
static inline int16_t mace_msg_global_position_state_full_get_vy(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field vz from global_position_state_full message
 *
 * @return Ground Z Speed (Altitude, positive down), expressed as m/s * 100
 */
static inline int16_t mace_msg_global_position_state_full_get_vz(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field relative_alt from global_position_state_full message
 *
 * @return Altitude above ground in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mace_msg_global_position_state_full_get_relative_alt(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field hdg from global_position_state_full message
 *
 * @return Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mace_msg_global_position_state_full_get_hdg(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Decode a global_position_state_full message into a struct
 *
 * @param msg The message to decode
 * @param global_position_state_full C-struct to decode the message contents into
 */
static inline void mace_msg_global_position_state_full_decode(const mace_message_t* msg, mace_global_position_state_full_t* global_position_state_full)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    global_position_state_full->time_boot_ms = mace_msg_global_position_state_full_get_time_boot_ms(msg);
    global_position_state_full->lat = mace_msg_global_position_state_full_get_lat(msg);
    global_position_state_full->lon = mace_msg_global_position_state_full_get_lon(msg);
    global_position_state_full->alt = mace_msg_global_position_state_full_get_alt(msg);
    global_position_state_full->relative_alt = mace_msg_global_position_state_full_get_relative_alt(msg);
    global_position_state_full->vx = mace_msg_global_position_state_full_get_vx(msg);
    global_position_state_full->vy = mace_msg_global_position_state_full_get_vy(msg);
    global_position_state_full->vz = mace_msg_global_position_state_full_get_vz(msg);
    global_position_state_full->hdg = mace_msg_global_position_state_full_get_hdg(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN? msg->len : MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN;
        memset(global_position_state_full, 0, MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_LEN);
    memcpy(global_position_state_full, _MACE_PAYLOAD(msg), len);
#endif
}
