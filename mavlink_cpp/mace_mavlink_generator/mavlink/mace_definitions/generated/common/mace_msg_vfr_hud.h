#pragma once
// MESSAGE VFR_HUD PACKING

#define MACE_MSG_ID_VFR_HUD 28

MACEPACKED(
typedef struct __mace_vfr_hud_t {
 float airspeed; /*< Current airspeed in m/s*/
 float groundspeed; /*< Current ground speed in m/s*/
 float alt; /*< Current altitude (MSL), in meters*/
 float climb; /*< Current climb rate in meters/second*/
 int16_t heading; /*< Current heading in degrees, in compass units (0..360, 0=north)*/
 uint16_t throttle; /*< Current throttle setting in integer percent, 0 to 100*/
}) mace_vfr_hud_t;

#define MACE_MSG_ID_VFR_HUD_LEN 20
#define MACE_MSG_ID_VFR_HUD_MIN_LEN 20
#define MACE_MSG_ID_28_LEN 20
#define MACE_MSG_ID_28_MIN_LEN 20

#define MACE_MSG_ID_VFR_HUD_CRC 20
#define MACE_MSG_ID_28_CRC 20



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_VFR_HUD { \
    28, \
    "VFR_HUD", \
    6, \
    {  { "airspeed", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_vfr_hud_t, airspeed) }, \
         { "groundspeed", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_vfr_hud_t, groundspeed) }, \
         { "alt", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_vfr_hud_t, alt) }, \
         { "climb", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_vfr_hud_t, climb) }, \
         { "heading", NULL, MACE_TYPE_INT16_T, 0, 16, offsetof(mace_vfr_hud_t, heading) }, \
         { "throttle", NULL, MACE_TYPE_UINT16_T, 0, 18, offsetof(mace_vfr_hud_t, throttle) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_VFR_HUD { \
    "VFR_HUD", \
    6, \
    {  { "airspeed", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_vfr_hud_t, airspeed) }, \
         { "groundspeed", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_vfr_hud_t, groundspeed) }, \
         { "alt", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_vfr_hud_t, alt) }, \
         { "climb", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_vfr_hud_t, climb) }, \
         { "heading", NULL, MACE_TYPE_INT16_T, 0, 16, offsetof(mace_vfr_hud_t, heading) }, \
         { "throttle", NULL, MACE_TYPE_UINT16_T, 0, 18, offsetof(mace_vfr_hud_t, throttle) }, \
         } \
}
#endif

/**
 * @brief Pack a vfr_hud message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param airspeed Current airspeed in m/s
 * @param groundspeed Current ground speed in m/s
 * @param heading Current heading in degrees, in compass units (0..360, 0=north)
 * @param throttle Current throttle setting in integer percent, 0 to 100
 * @param alt Current altitude (MSL), in meters
 * @param climb Current climb rate in meters/second
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_vfr_hud_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_VFR_HUD_LEN];
    _mace_put_float(buf, 0, airspeed);
    _mace_put_float(buf, 4, groundspeed);
    _mace_put_float(buf, 8, alt);
    _mace_put_float(buf, 12, climb);
    _mace_put_int16_t(buf, 16, heading);
    _mace_put_uint16_t(buf, 18, throttle);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_VFR_HUD_LEN);
#else
    mace_vfr_hud_t packet;
    packet.airspeed = airspeed;
    packet.groundspeed = groundspeed;
    packet.alt = alt;
    packet.climb = climb;
    packet.heading = heading;
    packet.throttle = throttle;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_VFR_HUD_LEN);
#endif

    msg->msgid = MACE_MSG_ID_VFR_HUD;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_VFR_HUD_MIN_LEN, MACE_MSG_ID_VFR_HUD_LEN, MACE_MSG_ID_VFR_HUD_CRC);
}

/**
 * @brief Pack a vfr_hud message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param airspeed Current airspeed in m/s
 * @param groundspeed Current ground speed in m/s
 * @param heading Current heading in degrees, in compass units (0..360, 0=north)
 * @param throttle Current throttle setting in integer percent, 0 to 100
 * @param alt Current altitude (MSL), in meters
 * @param climb Current climb rate in meters/second
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_vfr_hud_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   float airspeed,float groundspeed,int16_t heading,uint16_t throttle,float alt,float climb)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_VFR_HUD_LEN];
    _mace_put_float(buf, 0, airspeed);
    _mace_put_float(buf, 4, groundspeed);
    _mace_put_float(buf, 8, alt);
    _mace_put_float(buf, 12, climb);
    _mace_put_int16_t(buf, 16, heading);
    _mace_put_uint16_t(buf, 18, throttle);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_VFR_HUD_LEN);
#else
    mace_vfr_hud_t packet;
    packet.airspeed = airspeed;
    packet.groundspeed = groundspeed;
    packet.alt = alt;
    packet.climb = climb;
    packet.heading = heading;
    packet.throttle = throttle;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_VFR_HUD_LEN);
#endif

    msg->msgid = MACE_MSG_ID_VFR_HUD;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_VFR_HUD_MIN_LEN, MACE_MSG_ID_VFR_HUD_LEN, MACE_MSG_ID_VFR_HUD_CRC);
}

/**
 * @brief Encode a vfr_hud struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mace_msg_vfr_hud_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_vfr_hud_t* vfr_hud)
{
    return mace_msg_vfr_hud_pack(system_id, component_id, msg, vfr_hud->airspeed, vfr_hud->groundspeed, vfr_hud->heading, vfr_hud->throttle, vfr_hud->alt, vfr_hud->climb);
}

/**
 * @brief Encode a vfr_hud struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 */
static inline uint16_t mace_msg_vfr_hud_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_vfr_hud_t* vfr_hud)
{
    return mace_msg_vfr_hud_pack_chan(system_id, component_id, chan, msg, vfr_hud->airspeed, vfr_hud->groundspeed, vfr_hud->heading, vfr_hud->throttle, vfr_hud->alt, vfr_hud->climb);
}

/**
 * @brief Send a vfr_hud message
 * @param chan MAVLink channel to send the message
 *
 * @param airspeed Current airspeed in m/s
 * @param groundspeed Current ground speed in m/s
 * @param heading Current heading in degrees, in compass units (0..360, 0=north)
 * @param throttle Current throttle setting in integer percent, 0 to 100
 * @param alt Current altitude (MSL), in meters
 * @param climb Current climb rate in meters/second
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_vfr_hud_send(mace_channel_t chan, float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_VFR_HUD_LEN];
    _mace_put_float(buf, 0, airspeed);
    _mace_put_float(buf, 4, groundspeed);
    _mace_put_float(buf, 8, alt);
    _mace_put_float(buf, 12, climb);
    _mace_put_int16_t(buf, 16, heading);
    _mace_put_uint16_t(buf, 18, throttle);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VFR_HUD, buf, MACE_MSG_ID_VFR_HUD_MIN_LEN, MACE_MSG_ID_VFR_HUD_LEN, MACE_MSG_ID_VFR_HUD_CRC);
#else
    mace_vfr_hud_t packet;
    packet.airspeed = airspeed;
    packet.groundspeed = groundspeed;
    packet.alt = alt;
    packet.climb = climb;
    packet.heading = heading;
    packet.throttle = throttle;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VFR_HUD, (const char *)&packet, MACE_MSG_ID_VFR_HUD_MIN_LEN, MACE_MSG_ID_VFR_HUD_LEN, MACE_MSG_ID_VFR_HUD_CRC);
#endif
}

/**
 * @brief Send a vfr_hud message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_vfr_hud_send_struct(mace_channel_t chan, const mace_vfr_hud_t* vfr_hud)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_vfr_hud_send(chan, vfr_hud->airspeed, vfr_hud->groundspeed, vfr_hud->heading, vfr_hud->throttle, vfr_hud->alt, vfr_hud->climb);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VFR_HUD, (const char *)vfr_hud, MACE_MSG_ID_VFR_HUD_MIN_LEN, MACE_MSG_ID_VFR_HUD_LEN, MACE_MSG_ID_VFR_HUD_CRC);
#endif
}

#if MACE_MSG_ID_VFR_HUD_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_vfr_hud_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_float(buf, 0, airspeed);
    _mace_put_float(buf, 4, groundspeed);
    _mace_put_float(buf, 8, alt);
    _mace_put_float(buf, 12, climb);
    _mace_put_int16_t(buf, 16, heading);
    _mace_put_uint16_t(buf, 18, throttle);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VFR_HUD, buf, MACE_MSG_ID_VFR_HUD_MIN_LEN, MACE_MSG_ID_VFR_HUD_LEN, MACE_MSG_ID_VFR_HUD_CRC);
#else
    mace_vfr_hud_t *packet = (mace_vfr_hud_t *)msgbuf;
    packet->airspeed = airspeed;
    packet->groundspeed = groundspeed;
    packet->alt = alt;
    packet->climb = climb;
    packet->heading = heading;
    packet->throttle = throttle;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VFR_HUD, (const char *)packet, MACE_MSG_ID_VFR_HUD_MIN_LEN, MACE_MSG_ID_VFR_HUD_LEN, MACE_MSG_ID_VFR_HUD_CRC);
#endif
}
#endif

#endif

// MESSAGE VFR_HUD UNPACKING


/**
 * @brief Get field airspeed from vfr_hud message
 *
 * @return Current airspeed in m/s
 */
static inline float mace_msg_vfr_hud_get_airspeed(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  0);
}

/**
 * @brief Get field groundspeed from vfr_hud message
 *
 * @return Current ground speed in m/s
 */
static inline float mace_msg_vfr_hud_get_groundspeed(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field heading from vfr_hud message
 *
 * @return Current heading in degrees, in compass units (0..360, 0=north)
 */
static inline int16_t mace_msg_vfr_hud_get_heading(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field throttle from vfr_hud message
 *
 * @return Current throttle setting in integer percent, 0 to 100
 */
static inline uint16_t mace_msg_vfr_hud_get_throttle(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field alt from vfr_hud message
 *
 * @return Current altitude (MSL), in meters
 */
static inline float mace_msg_vfr_hud_get_alt(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field climb from vfr_hud message
 *
 * @return Current climb rate in meters/second
 */
static inline float mace_msg_vfr_hud_get_climb(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Decode a vfr_hud message into a struct
 *
 * @param msg The message to decode
 * @param vfr_hud C-struct to decode the message contents into
 */
static inline void mace_msg_vfr_hud_decode(const mace_message_t* msg, mace_vfr_hud_t* vfr_hud)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    vfr_hud->airspeed = mace_msg_vfr_hud_get_airspeed(msg);
    vfr_hud->groundspeed = mace_msg_vfr_hud_get_groundspeed(msg);
    vfr_hud->alt = mace_msg_vfr_hud_get_alt(msg);
    vfr_hud->climb = mace_msg_vfr_hud_get_climb(msg);
    vfr_hud->heading = mace_msg_vfr_hud_get_heading(msg);
    vfr_hud->throttle = mace_msg_vfr_hud_get_throttle(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_VFR_HUD_LEN? msg->len : MACE_MSG_ID_VFR_HUD_LEN;
        memset(vfr_hud, 0, MACE_MSG_ID_VFR_HUD_LEN);
    memcpy(vfr_hud, _MACE_PAYLOAD(msg), len);
#endif
}
