#pragma once
// MESSAGE LOCAL_POSITION_NED PACKING

#define MACE_MSG_ID_LOCAL_POSITION_NED 20

MACEPACKED(
typedef struct __mace_local_position_ned_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float x; /*< X Position*/
 float y; /*< Y Position*/
 float z; /*< Z Position*/
}) mace_local_position_ned_t;

#define MACE_MSG_ID_LOCAL_POSITION_NED_LEN 16
#define MACE_MSG_ID_LOCAL_POSITION_NED_MIN_LEN 16
#define MACE_MSG_ID_20_LEN 16
#define MACE_MSG_ID_20_MIN_LEN 16

#define MACE_MSG_ID_LOCAL_POSITION_NED_CRC 121
#define MACE_MSG_ID_20_CRC 121



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_LOCAL_POSITION_NED { \
    20, \
    "LOCAL_POSITION_NED", \
    4, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_local_position_ned_t, time_boot_ms) }, \
         { "x", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_local_position_ned_t, x) }, \
         { "y", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_local_position_ned_t, y) }, \
         { "z", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_local_position_ned_t, z) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_LOCAL_POSITION_NED { \
    "LOCAL_POSITION_NED", \
    4, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_local_position_ned_t, time_boot_ms) }, \
         { "x", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_local_position_ned_t, x) }, \
         { "y", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_local_position_ned_t, y) }, \
         { "z", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_local_position_ned_t, z) }, \
         } \
}
#endif

/**
 * @brief Pack a local_position_ned message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_local_position_ned_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint32_t time_boot_ms, float x, float y, float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_LOCAL_POSITION_NED_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, x);
    _mace_put_float(buf, 8, y);
    _mace_put_float(buf, 12, z);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_LOCAL_POSITION_NED_LEN);
#else
    mace_local_position_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_LOCAL_POSITION_NED_LEN);
#endif

    msg->msgid = MACE_MSG_ID_LOCAL_POSITION_NED;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_CRC);
}

/**
 * @brief Pack a local_position_ned message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_local_position_ned_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint32_t time_boot_ms,float x,float y,float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_LOCAL_POSITION_NED_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, x);
    _mace_put_float(buf, 8, y);
    _mace_put_float(buf, 12, z);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_LOCAL_POSITION_NED_LEN);
#else
    mace_local_position_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_LOCAL_POSITION_NED_LEN);
#endif

    msg->msgid = MACE_MSG_ID_LOCAL_POSITION_NED;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_CRC);
}

/**
 * @brief Encode a local_position_ned struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param local_position_ned C-struct to read the message contents from
 */
static inline uint16_t mace_msg_local_position_ned_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_local_position_ned_t* local_position_ned)
{
    return mace_msg_local_position_ned_pack(system_id, component_id, msg, local_position_ned->time_boot_ms, local_position_ned->x, local_position_ned->y, local_position_ned->z);
}

/**
 * @brief Encode a local_position_ned struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param local_position_ned C-struct to read the message contents from
 */
static inline uint16_t mace_msg_local_position_ned_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_local_position_ned_t* local_position_ned)
{
    return mace_msg_local_position_ned_pack_chan(system_id, component_id, chan, msg, local_position_ned->time_boot_ms, local_position_ned->x, local_position_ned->y, local_position_ned->z);
}

/**
 * @brief Send a local_position_ned message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_local_position_ned_send(mace_channel_t chan, uint32_t time_boot_ms, float x, float y, float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_LOCAL_POSITION_NED_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, x);
    _mace_put_float(buf, 8, y);
    _mace_put_float(buf, 12, z);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_LOCAL_POSITION_NED, buf, MACE_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_CRC);
#else
    mace_local_position_ned_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.x = x;
    packet.y = y;
    packet.z = z;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_LOCAL_POSITION_NED, (const char *)&packet, MACE_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_CRC);
#endif
}

/**
 * @brief Send a local_position_ned message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_local_position_ned_send_struct(mace_channel_t chan, const mace_local_position_ned_t* local_position_ned)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_local_position_ned_send(chan, local_position_ned->time_boot_ms, local_position_ned->x, local_position_ned->y, local_position_ned->z);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_LOCAL_POSITION_NED, (const char *)local_position_ned, MACE_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_CRC);
#endif
}

#if MACE_MSG_ID_LOCAL_POSITION_NED_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_local_position_ned_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint32_t time_boot_ms, float x, float y, float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, x);
    _mace_put_float(buf, 8, y);
    _mace_put_float(buf, 12, z);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_LOCAL_POSITION_NED, buf, MACE_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_CRC);
#else
    mace_local_position_ned_t *packet = (mace_local_position_ned_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->x = x;
    packet->y = y;
    packet->z = z;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_LOCAL_POSITION_NED, (const char *)packet, MACE_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_LEN, MACE_MSG_ID_LOCAL_POSITION_NED_CRC);
#endif
}
#endif

#endif

// MESSAGE LOCAL_POSITION_NED UNPACKING


/**
 * @brief Get field time_boot_ms from local_position_ned message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mace_msg_local_position_ned_get_time_boot_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field x from local_position_ned message
 *
 * @return X Position
 */
static inline float mace_msg_local_position_ned_get_x(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from local_position_ned message
 *
 * @return Y Position
 */
static inline float mace_msg_local_position_ned_get_y(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from local_position_ned message
 *
 * @return Z Position
 */
static inline float mace_msg_local_position_ned_get_z(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Decode a local_position_ned message into a struct
 *
 * @param msg The message to decode
 * @param local_position_ned C-struct to decode the message contents into
 */
static inline void mace_msg_local_position_ned_decode(const mace_message_t* msg, mace_local_position_ned_t* local_position_ned)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    local_position_ned->time_boot_ms = mace_msg_local_position_ned_get_time_boot_ms(msg);
    local_position_ned->x = mace_msg_local_position_ned_get_x(msg);
    local_position_ned->y = mace_msg_local_position_ned_get_y(msg);
    local_position_ned->z = mace_msg_local_position_ned_get_z(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_LOCAL_POSITION_NED_LEN? msg->len : MACE_MSG_ID_LOCAL_POSITION_NED_LEN;
        memset(local_position_ned, 0, MACE_MSG_ID_LOCAL_POSITION_NED_LEN);
    memcpy(local_position_ned, _MACE_PAYLOAD(msg), len);
#endif
}
