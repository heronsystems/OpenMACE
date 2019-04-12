#pragma once
// MESSAGE GLOBAL_VELOCITY_INT PACKING

#define MACE_MSG_ID_GLOBAL_VELOCITY_INT 24

MACEPACKED(
typedef struct __mace_global_velocity_int_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 int16_t vx; /*< Ground X Speed (Latitude, positive north), expressed as m/s * 100*/
 int16_t vy; /*< Ground Y Speed (Longitude, positive east), expressed as m/s * 100*/
 int16_t vz; /*< Ground Z Speed (Altitude, positive down), expressed as m/s * 100*/
}) mace_global_velocity_int_t;

#define MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN 10
#define MACE_MSG_ID_GLOBAL_VELOCITY_INT_MIN_LEN 10
#define MACE_MSG_ID_24_LEN 10
#define MACE_MSG_ID_24_MIN_LEN 10

#define MACE_MSG_ID_GLOBAL_VELOCITY_INT_CRC 245
#define MACE_MSG_ID_24_CRC 245



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_GLOBAL_VELOCITY_INT { \
    24, \
    "GLOBAL_VELOCITY_INT", \
    4, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_global_velocity_int_t, time_boot_ms) }, \
         { "vx", NULL, MACE_TYPE_INT16_T, 0, 4, offsetof(mace_global_velocity_int_t, vx) }, \
         { "vy", NULL, MACE_TYPE_INT16_T, 0, 6, offsetof(mace_global_velocity_int_t, vy) }, \
         { "vz", NULL, MACE_TYPE_INT16_T, 0, 8, offsetof(mace_global_velocity_int_t, vz) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_GLOBAL_VELOCITY_INT { \
    "GLOBAL_VELOCITY_INT", \
    4, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_global_velocity_int_t, time_boot_ms) }, \
         { "vx", NULL, MACE_TYPE_INT16_T, 0, 4, offsetof(mace_global_velocity_int_t, vx) }, \
         { "vy", NULL, MACE_TYPE_INT16_T, 0, 6, offsetof(mace_global_velocity_int_t, vy) }, \
         { "vz", NULL, MACE_TYPE_INT16_T, 0, 8, offsetof(mace_global_velocity_int_t, vz) }, \
         } \
}
#endif

/**
 * @brief Pack a global_velocity_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param vx Ground X Speed (Latitude, positive north), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude, positive east), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude, positive down), expressed as m/s * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_global_velocity_int_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint32_t time_boot_ms, int16_t vx, int16_t vy, int16_t vz)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_int16_t(buf, 4, vx);
    _mace_put_int16_t(buf, 6, vy);
    _mace_put_int16_t(buf, 8, vz);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN);
#else
    mace_global_velocity_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GLOBAL_VELOCITY_INT;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_GLOBAL_VELOCITY_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_CRC);
}

/**
 * @brief Pack a global_velocity_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param vx Ground X Speed (Latitude, positive north), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude, positive east), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude, positive down), expressed as m/s * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_global_velocity_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint32_t time_boot_ms,int16_t vx,int16_t vy,int16_t vz)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_int16_t(buf, 4, vx);
    _mace_put_int16_t(buf, 6, vy);
    _mace_put_int16_t(buf, 8, vz);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN);
#else
    mace_global_velocity_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GLOBAL_VELOCITY_INT;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_GLOBAL_VELOCITY_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_CRC);
}

/**
 * @brief Encode a global_velocity_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_velocity_int C-struct to read the message contents from
 */
static inline uint16_t mace_msg_global_velocity_int_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_global_velocity_int_t* global_velocity_int)
{
    return mace_msg_global_velocity_int_pack(system_id, component_id, msg, global_velocity_int->time_boot_ms, global_velocity_int->vx, global_velocity_int->vy, global_velocity_int->vz);
}

/**
 * @brief Encode a global_velocity_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_velocity_int C-struct to read the message contents from
 */
static inline uint16_t mace_msg_global_velocity_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_global_velocity_int_t* global_velocity_int)
{
    return mace_msg_global_velocity_int_pack_chan(system_id, component_id, chan, msg, global_velocity_int->time_boot_ms, global_velocity_int->vx, global_velocity_int->vy, global_velocity_int->vz);
}

/**
 * @brief Send a global_velocity_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param vx Ground X Speed (Latitude, positive north), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude, positive east), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude, positive down), expressed as m/s * 100
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_global_velocity_int_send(mace_channel_t chan, uint32_t time_boot_ms, int16_t vx, int16_t vy, int16_t vz)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_int16_t(buf, 4, vx);
    _mace_put_int16_t(buf, 6, vy);
    _mace_put_int16_t(buf, 8, vz);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_VELOCITY_INT, buf, MACE_MSG_ID_GLOBAL_VELOCITY_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_CRC);
#else
    mace_global_velocity_int_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_VELOCITY_INT, (const char *)&packet, MACE_MSG_ID_GLOBAL_VELOCITY_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_CRC);
#endif
}

/**
 * @brief Send a global_velocity_int message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_global_velocity_int_send_struct(mace_channel_t chan, const mace_global_velocity_int_t* global_velocity_int)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_global_velocity_int_send(chan, global_velocity_int->time_boot_ms, global_velocity_int->vx, global_velocity_int->vy, global_velocity_int->vz);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_VELOCITY_INT, (const char *)global_velocity_int, MACE_MSG_ID_GLOBAL_VELOCITY_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_CRC);
#endif
}

#if MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_global_velocity_int_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint32_t time_boot_ms, int16_t vx, int16_t vy, int16_t vz)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_int16_t(buf, 4, vx);
    _mace_put_int16_t(buf, 6, vy);
    _mace_put_int16_t(buf, 8, vz);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_VELOCITY_INT, buf, MACE_MSG_ID_GLOBAL_VELOCITY_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_CRC);
#else
    mace_global_velocity_int_t *packet = (mace_global_velocity_int_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GLOBAL_VELOCITY_INT, (const char *)packet, MACE_MSG_ID_GLOBAL_VELOCITY_INT_MIN_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN, MACE_MSG_ID_GLOBAL_VELOCITY_INT_CRC);
#endif
}
#endif

#endif

// MESSAGE GLOBAL_VELOCITY_INT UNPACKING


/**
 * @brief Get field time_boot_ms from global_velocity_int message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mace_msg_global_velocity_int_get_time_boot_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field vx from global_velocity_int message
 *
 * @return Ground X Speed (Latitude, positive north), expressed as m/s * 100
 */
static inline int16_t mace_msg_global_velocity_int_get_vx(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field vy from global_velocity_int message
 *
 * @return Ground Y Speed (Longitude, positive east), expressed as m/s * 100
 */
static inline int16_t mace_msg_global_velocity_int_get_vy(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field vz from global_velocity_int message
 *
 * @return Ground Z Speed (Altitude, positive down), expressed as m/s * 100
 */
static inline int16_t mace_msg_global_velocity_int_get_vz(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  8);
}

/**
 * @brief Decode a global_velocity_int message into a struct
 *
 * @param msg The message to decode
 * @param global_velocity_int C-struct to decode the message contents into
 */
static inline void mace_msg_global_velocity_int_decode(const mace_message_t* msg, mace_global_velocity_int_t* global_velocity_int)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    global_velocity_int->time_boot_ms = mace_msg_global_velocity_int_get_time_boot_ms(msg);
    global_velocity_int->vx = mace_msg_global_velocity_int_get_vx(msg);
    global_velocity_int->vy = mace_msg_global_velocity_int_get_vy(msg);
    global_velocity_int->vz = mace_msg_global_velocity_int_get_vz(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN? msg->len : MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN;
        memset(global_velocity_int, 0, MACE_MSG_ID_GLOBAL_VELOCITY_INT_LEN);
    memcpy(global_velocity_int, _MACE_PAYLOAD(msg), len);
#endif
}
