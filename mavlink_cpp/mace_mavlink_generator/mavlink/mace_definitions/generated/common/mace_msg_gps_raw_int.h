#pragma once
// MESSAGE GPS_RAW_INT PACKING

#define MACE_MSG_ID_GPS_RAW_INT 13

MACEPACKED(
typedef struct __mace_gps_raw_int_t {
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 int32_t lat; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t lon; /*< Longitude (WGS84), in degrees * 1E7*/
 int32_t alt; /*< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.*/
 uint16_t eph; /*< GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
 uint16_t epv; /*< GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
 uint16_t vel; /*< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
 uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
 uint8_t fix_type; /*< See the GPS_FIX_TYPE enum.*/
 uint8_t satellites_visible; /*< Number of satellites visible. If unknown, set to 255*/
}) mace_gps_raw_int_t;

#define MACE_MSG_ID_GPS_RAW_INT_LEN 30
#define MACE_MSG_ID_GPS_RAW_INT_MIN_LEN 30
#define MACE_MSG_ID_13_LEN 30
#define MACE_MSG_ID_13_MIN_LEN 30

#define MACE_MSG_ID_GPS_RAW_INT_CRC 24
#define MACE_MSG_ID_13_CRC 24



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_GPS_RAW_INT { \
    13, \
    "GPS_RAW_INT", \
    10, \
    {  { "time_usec", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_gps_raw_int_t, time_usec) }, \
         { "lat", NULL, MACE_TYPE_INT32_T, 0, 8, offsetof(mace_gps_raw_int_t, lat) }, \
         { "lon", NULL, MACE_TYPE_INT32_T, 0, 12, offsetof(mace_gps_raw_int_t, lon) }, \
         { "alt", NULL, MACE_TYPE_INT32_T, 0, 16, offsetof(mace_gps_raw_int_t, alt) }, \
         { "eph", NULL, MACE_TYPE_UINT16_T, 0, 20, offsetof(mace_gps_raw_int_t, eph) }, \
         { "epv", NULL, MACE_TYPE_UINT16_T, 0, 22, offsetof(mace_gps_raw_int_t, epv) }, \
         { "vel", NULL, MACE_TYPE_UINT16_T, 0, 24, offsetof(mace_gps_raw_int_t, vel) }, \
         { "cog", NULL, MACE_TYPE_UINT16_T, 0, 26, offsetof(mace_gps_raw_int_t, cog) }, \
         { "fix_type", NULL, MACE_TYPE_UINT8_T, 0, 28, offsetof(mace_gps_raw_int_t, fix_type) }, \
         { "satellites_visible", NULL, MACE_TYPE_UINT8_T, 0, 29, offsetof(mace_gps_raw_int_t, satellites_visible) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_GPS_RAW_INT { \
    "GPS_RAW_INT", \
    10, \
    {  { "time_usec", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_gps_raw_int_t, time_usec) }, \
         { "lat", NULL, MACE_TYPE_INT32_T, 0, 8, offsetof(mace_gps_raw_int_t, lat) }, \
         { "lon", NULL, MACE_TYPE_INT32_T, 0, 12, offsetof(mace_gps_raw_int_t, lon) }, \
         { "alt", NULL, MACE_TYPE_INT32_T, 0, 16, offsetof(mace_gps_raw_int_t, alt) }, \
         { "eph", NULL, MACE_TYPE_UINT16_T, 0, 20, offsetof(mace_gps_raw_int_t, eph) }, \
         { "epv", NULL, MACE_TYPE_UINT16_T, 0, 22, offsetof(mace_gps_raw_int_t, epv) }, \
         { "vel", NULL, MACE_TYPE_UINT16_T, 0, 24, offsetof(mace_gps_raw_int_t, vel) }, \
         { "cog", NULL, MACE_TYPE_UINT16_T, 0, 26, offsetof(mace_gps_raw_int_t, cog) }, \
         { "fix_type", NULL, MACE_TYPE_UINT8_T, 0, 28, offsetof(mace_gps_raw_int_t, fix_type) }, \
         { "satellites_visible", NULL, MACE_TYPE_UINT8_T, 0, 29, offsetof(mace_gps_raw_int_t, satellites_visible) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_raw_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type See the GPS_FIX_TYPE enum.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
 * @param eph GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param epv GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_gps_raw_int_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GPS_RAW_INT_LEN];
    _mace_put_uint64_t(buf, 0, time_usec);
    _mace_put_int32_t(buf, 8, lat);
    _mace_put_int32_t(buf, 12, lon);
    _mace_put_int32_t(buf, 16, alt);
    _mace_put_uint16_t(buf, 20, eph);
    _mace_put_uint16_t(buf, 22, epv);
    _mace_put_uint16_t(buf, 24, vel);
    _mace_put_uint16_t(buf, 26, cog);
    _mace_put_uint8_t(buf, 28, fix_type);
    _mace_put_uint8_t(buf, 29, satellites_visible);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GPS_RAW_INT_LEN);
#else
    mace_gps_raw_int_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GPS_RAW_INT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GPS_RAW_INT;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_GPS_RAW_INT_MIN_LEN, MACE_MSG_ID_GPS_RAW_INT_LEN, MACE_MSG_ID_GPS_RAW_INT_CRC);
}

/**
 * @brief Pack a gps_raw_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type See the GPS_FIX_TYPE enum.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
 * @param eph GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param epv GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_gps_raw_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t time_usec,uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,uint16_t cog,uint8_t satellites_visible)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GPS_RAW_INT_LEN];
    _mace_put_uint64_t(buf, 0, time_usec);
    _mace_put_int32_t(buf, 8, lat);
    _mace_put_int32_t(buf, 12, lon);
    _mace_put_int32_t(buf, 16, alt);
    _mace_put_uint16_t(buf, 20, eph);
    _mace_put_uint16_t(buf, 22, epv);
    _mace_put_uint16_t(buf, 24, vel);
    _mace_put_uint16_t(buf, 26, cog);
    _mace_put_uint8_t(buf, 28, fix_type);
    _mace_put_uint8_t(buf, 29, satellites_visible);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GPS_RAW_INT_LEN);
#else
    mace_gps_raw_int_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GPS_RAW_INT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GPS_RAW_INT;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_GPS_RAW_INT_MIN_LEN, MACE_MSG_ID_GPS_RAW_INT_LEN, MACE_MSG_ID_GPS_RAW_INT_CRC);
}

/**
 * @brief Encode a gps_raw_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_raw_int C-struct to read the message contents from
 */
static inline uint16_t mace_msg_gps_raw_int_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_gps_raw_int_t* gps_raw_int)
{
    return mace_msg_gps_raw_int_pack(system_id, component_id, msg, gps_raw_int->time_usec, gps_raw_int->fix_type, gps_raw_int->lat, gps_raw_int->lon, gps_raw_int->alt, gps_raw_int->eph, gps_raw_int->epv, gps_raw_int->vel, gps_raw_int->cog, gps_raw_int->satellites_visible);
}

/**
 * @brief Encode a gps_raw_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_raw_int C-struct to read the message contents from
 */
static inline uint16_t mace_msg_gps_raw_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_gps_raw_int_t* gps_raw_int)
{
    return mace_msg_gps_raw_int_pack_chan(system_id, component_id, chan, msg, gps_raw_int->time_usec, gps_raw_int->fix_type, gps_raw_int->lat, gps_raw_int->lon, gps_raw_int->alt, gps_raw_int->eph, gps_raw_int->epv, gps_raw_int->vel, gps_raw_int->cog, gps_raw_int->satellites_visible);
}

/**
 * @brief Send a gps_raw_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type See the GPS_FIX_TYPE enum.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
 * @param eph GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param epv GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_gps_raw_int_send(mace_channel_t chan, uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GPS_RAW_INT_LEN];
    _mace_put_uint64_t(buf, 0, time_usec);
    _mace_put_int32_t(buf, 8, lat);
    _mace_put_int32_t(buf, 12, lon);
    _mace_put_int32_t(buf, 16, alt);
    _mace_put_uint16_t(buf, 20, eph);
    _mace_put_uint16_t(buf, 22, epv);
    _mace_put_uint16_t(buf, 24, vel);
    _mace_put_uint16_t(buf, 26, cog);
    _mace_put_uint8_t(buf, 28, fix_type);
    _mace_put_uint8_t(buf, 29, satellites_visible);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_RAW_INT, buf, MACE_MSG_ID_GPS_RAW_INT_MIN_LEN, MACE_MSG_ID_GPS_RAW_INT_LEN, MACE_MSG_ID_GPS_RAW_INT_CRC);
#else
    mace_gps_raw_int_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_RAW_INT, (const char *)&packet, MACE_MSG_ID_GPS_RAW_INT_MIN_LEN, MACE_MSG_ID_GPS_RAW_INT_LEN, MACE_MSG_ID_GPS_RAW_INT_CRC);
#endif
}

/**
 * @brief Send a gps_raw_int message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_gps_raw_int_send_struct(mace_channel_t chan, const mace_gps_raw_int_t* gps_raw_int)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_gps_raw_int_send(chan, gps_raw_int->time_usec, gps_raw_int->fix_type, gps_raw_int->lat, gps_raw_int->lon, gps_raw_int->alt, gps_raw_int->eph, gps_raw_int->epv, gps_raw_int->vel, gps_raw_int->cog, gps_raw_int->satellites_visible);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_RAW_INT, (const char *)gps_raw_int, MACE_MSG_ID_GPS_RAW_INT_MIN_LEN, MACE_MSG_ID_GPS_RAW_INT_LEN, MACE_MSG_ID_GPS_RAW_INT_CRC);
#endif
}

#if MACE_MSG_ID_GPS_RAW_INT_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_gps_raw_int_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, time_usec);
    _mace_put_int32_t(buf, 8, lat);
    _mace_put_int32_t(buf, 12, lon);
    _mace_put_int32_t(buf, 16, alt);
    _mace_put_uint16_t(buf, 20, eph);
    _mace_put_uint16_t(buf, 22, epv);
    _mace_put_uint16_t(buf, 24, vel);
    _mace_put_uint16_t(buf, 26, cog);
    _mace_put_uint8_t(buf, 28, fix_type);
    _mace_put_uint8_t(buf, 29, satellites_visible);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_RAW_INT, buf, MACE_MSG_ID_GPS_RAW_INT_MIN_LEN, MACE_MSG_ID_GPS_RAW_INT_LEN, MACE_MSG_ID_GPS_RAW_INT_CRC);
#else
    mace_gps_raw_int_t *packet = (mace_gps_raw_int_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->eph = eph;
    packet->epv = epv;
    packet->vel = vel;
    packet->cog = cog;
    packet->fix_type = fix_type;
    packet->satellites_visible = satellites_visible;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_RAW_INT, (const char *)packet, MACE_MSG_ID_GPS_RAW_INT_MIN_LEN, MACE_MSG_ID_GPS_RAW_INT_LEN, MACE_MSG_ID_GPS_RAW_INT_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_RAW_INT UNPACKING


/**
 * @brief Get field time_usec from gps_raw_int message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mace_msg_gps_raw_int_get_time_usec(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field fix_type from gps_raw_int message
 *
 * @return See the GPS_FIX_TYPE enum.
 */
static inline uint8_t mace_msg_gps_raw_int_get_fix_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field lat from gps_raw_int message
 *
 * @return Latitude (WGS84), in degrees * 1E7
 */
static inline int32_t mace_msg_gps_raw_int_get_lat(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from gps_raw_int message
 *
 * @return Longitude (WGS84), in degrees * 1E7
 */
static inline int32_t mace_msg_gps_raw_int_get_lon(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from gps_raw_int message
 *
 * @return Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
 */
static inline int32_t mace_msg_gps_raw_int_get_alt(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field eph from gps_raw_int message
 *
 * @return GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
 */
static inline uint16_t mace_msg_gps_raw_int_get_eph(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field epv from gps_raw_int message
 *
 * @return GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
 */
static inline uint16_t mace_msg_gps_raw_int_get_epv(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field vel from gps_raw_int message
 *
 * @return GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 */
static inline uint16_t mace_msg_gps_raw_int_get_vel(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field cog from gps_raw_int message
 *
 * @return Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mace_msg_gps_raw_int_get_cog(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field satellites_visible from gps_raw_int message
 *
 * @return Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t mace_msg_gps_raw_int_get_satellites_visible(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Decode a gps_raw_int message into a struct
 *
 * @param msg The message to decode
 * @param gps_raw_int C-struct to decode the message contents into
 */
static inline void mace_msg_gps_raw_int_decode(const mace_message_t* msg, mace_gps_raw_int_t* gps_raw_int)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    gps_raw_int->time_usec = mace_msg_gps_raw_int_get_time_usec(msg);
    gps_raw_int->lat = mace_msg_gps_raw_int_get_lat(msg);
    gps_raw_int->lon = mace_msg_gps_raw_int_get_lon(msg);
    gps_raw_int->alt = mace_msg_gps_raw_int_get_alt(msg);
    gps_raw_int->eph = mace_msg_gps_raw_int_get_eph(msg);
    gps_raw_int->epv = mace_msg_gps_raw_int_get_epv(msg);
    gps_raw_int->vel = mace_msg_gps_raw_int_get_vel(msg);
    gps_raw_int->cog = mace_msg_gps_raw_int_get_cog(msg);
    gps_raw_int->fix_type = mace_msg_gps_raw_int_get_fix_type(msg);
    gps_raw_int->satellites_visible = mace_msg_gps_raw_int_get_satellites_visible(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_GPS_RAW_INT_LEN? msg->len : MACE_MSG_ID_GPS_RAW_INT_LEN;
        memset(gps_raw_int, 0, MACE_MSG_ID_GPS_RAW_INT_LEN);
    memcpy(gps_raw_int, _MACE_PAYLOAD(msg), len);
#endif
}
