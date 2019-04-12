#pragma once
// MESSAGE LOCAL_VELOCITY_NED PACKING

#define MACE_MSG_ID_LOCAL_VELOCITY_NED 21

MACEPACKED(
typedef struct __mace_local_velocity_ned_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float vx; /*< X Speed*/
 float vy; /*< Y Speed*/
 float vz; /*< Z Speed*/
}) mace_local_velocity_ned_t;

#define MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN 16
#define MACE_MSG_ID_LOCAL_VELOCITY_NED_MIN_LEN 16
#define MACE_MSG_ID_21_LEN 16
#define MACE_MSG_ID_21_MIN_LEN 16

#define MACE_MSG_ID_LOCAL_VELOCITY_NED_CRC 62
#define MACE_MSG_ID_21_CRC 62



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_LOCAL_VELOCITY_NED { \
    21, \
    "LOCAL_VELOCITY_NED", \
    4, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_local_velocity_ned_t, time_boot_ms) }, \
         { "vx", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_local_velocity_ned_t, vx) }, \
         { "vy", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_local_velocity_ned_t, vy) }, \
         { "vz", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_local_velocity_ned_t, vz) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_LOCAL_VELOCITY_NED { \
    "LOCAL_VELOCITY_NED", \
    4, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_local_velocity_ned_t, time_boot_ms) }, \
         { "vx", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_local_velocity_ned_t, vx) }, \
         { "vy", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_local_velocity_ned_t, vy) }, \
         { "vz", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_local_velocity_ned_t, vz) }, \
         } \
}
#endif

/**
 * @brief Pack a local_velocity_ned message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_local_velocity_ned_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint32_t time_boot_ms, float vx, float vy, float vz)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, vx);
    _mace_put_float(buf, 8, vy);
    _mace_put_float(buf, 12, vz);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN);
#else
    mace_local_velocity_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN);
#endif

    msg->msgid = MACE_MSG_ID_LOCAL_VELOCITY_NED;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_LOCAL_VELOCITY_NED_MIN_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_CRC);
}

/**
 * @brief Pack a local_velocity_ned message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_local_velocity_ned_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint32_t time_boot_ms,float vx,float vy,float vz)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, vx);
    _mace_put_float(buf, 8, vy);
    _mace_put_float(buf, 12, vz);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN);
#else
    mace_local_velocity_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN);
#endif

    msg->msgid = MACE_MSG_ID_LOCAL_VELOCITY_NED;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_LOCAL_VELOCITY_NED_MIN_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_CRC);
}

/**
 * @brief Encode a local_velocity_ned struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param local_velocity_ned C-struct to read the message contents from
 */
static inline uint16_t mace_msg_local_velocity_ned_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_local_velocity_ned_t* local_velocity_ned)
{
    return mace_msg_local_velocity_ned_pack(system_id, component_id, msg, local_velocity_ned->time_boot_ms, local_velocity_ned->vx, local_velocity_ned->vy, local_velocity_ned->vz);
}

/**
 * @brief Encode a local_velocity_ned struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param local_velocity_ned C-struct to read the message contents from
 */
static inline uint16_t mace_msg_local_velocity_ned_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_local_velocity_ned_t* local_velocity_ned)
{
    return mace_msg_local_velocity_ned_pack_chan(system_id, component_id, chan, msg, local_velocity_ned->time_boot_ms, local_velocity_ned->vx, local_velocity_ned->vy, local_velocity_ned->vz);
}

/**
 * @brief Send a local_velocity_ned message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_local_velocity_ned_send(mace_channel_t chan, uint32_t time_boot_ms, float vx, float vy, float vz)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, vx);
    _mace_put_float(buf, 8, vy);
    _mace_put_float(buf, 12, vz);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_LOCAL_VELOCITY_NED, buf, MACE_MSG_ID_LOCAL_VELOCITY_NED_MIN_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_CRC);
#else
    mace_local_velocity_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_LOCAL_VELOCITY_NED, (const char *)&packet, MACE_MSG_ID_LOCAL_VELOCITY_NED_MIN_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_CRC);
#endif
}

/**
 * @brief Send a local_velocity_ned message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_local_velocity_ned_send_struct(mace_channel_t chan, const mace_local_velocity_ned_t* local_velocity_ned)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_local_velocity_ned_send(chan, local_velocity_ned->time_boot_ms, local_velocity_ned->vx, local_velocity_ned->vy, local_velocity_ned->vz);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_LOCAL_VELOCITY_NED, (const char *)local_velocity_ned, MACE_MSG_ID_LOCAL_VELOCITY_NED_MIN_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_CRC);
#endif
}

#if MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_local_velocity_ned_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint32_t time_boot_ms, float vx, float vy, float vz)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, vx);
    _mace_put_float(buf, 8, vy);
    _mace_put_float(buf, 12, vz);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_LOCAL_VELOCITY_NED, buf, MACE_MSG_ID_LOCAL_VELOCITY_NED_MIN_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_CRC);
#else
    mace_local_velocity_ned_t *packet = (mace_local_velocity_ned_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_LOCAL_VELOCITY_NED, (const char *)packet, MACE_MSG_ID_LOCAL_VELOCITY_NED_MIN_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN, MACE_MSG_ID_LOCAL_VELOCITY_NED_CRC);
#endif
}
#endif

#endif

// MESSAGE LOCAL_VELOCITY_NED UNPACKING


/**
 * @brief Get field time_boot_ms from local_velocity_ned message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mace_msg_local_velocity_ned_get_time_boot_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field vx from local_velocity_ned message
 *
 * @return X Speed
 */
static inline float mace_msg_local_velocity_ned_get_vx(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field vy from local_velocity_ned message
 *
 * @return Y Speed
 */
static inline float mace_msg_local_velocity_ned_get_vy(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field vz from local_velocity_ned message
 *
 * @return Z Speed
 */
static inline float mace_msg_local_velocity_ned_get_vz(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Decode a local_velocity_ned message into a struct
 *
 * @param msg The message to decode
 * @param local_velocity_ned C-struct to decode the message contents into
 */
static inline void mace_msg_local_velocity_ned_decode(const mace_message_t* msg, mace_local_velocity_ned_t* local_velocity_ned)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    local_velocity_ned->time_boot_ms = mace_msg_local_velocity_ned_get_time_boot_ms(msg);
    local_velocity_ned->vx = mace_msg_local_velocity_ned_get_vx(msg);
    local_velocity_ned->vy = mace_msg_local_velocity_ned_get_vy(msg);
    local_velocity_ned->vz = mace_msg_local_velocity_ned_get_vz(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN? msg->len : MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN;
        memset(local_velocity_ned, 0, MACE_MSG_ID_LOCAL_VELOCITY_NED_LEN);
    memcpy(local_velocity_ned, _MACE_PAYLOAD(msg), len);
#endif
}
