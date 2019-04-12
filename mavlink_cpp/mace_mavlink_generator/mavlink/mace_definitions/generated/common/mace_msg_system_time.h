#pragma once
// MESSAGE SYSTEM_TIME PACKING

#define MACE_MSG_ID_SYSTEM_TIME 4

MACEPACKED(
typedef struct __mace_system_time_t {
 uint64_t time_unix_usec; /*< Timestamp of the master clock in microseconds since UNIX epoch.*/
 uint32_t time_boot_ms; /*< Timestamp of the component clock since boot time in milliseconds.*/
}) mace_system_time_t;

#define MACE_MSG_ID_SYSTEM_TIME_LEN 12
#define MACE_MSG_ID_SYSTEM_TIME_MIN_LEN 12
#define MACE_MSG_ID_4_LEN 12
#define MACE_MSG_ID_4_MIN_LEN 12

#define MACE_MSG_ID_SYSTEM_TIME_CRC 137
#define MACE_MSG_ID_4_CRC 137



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_SYSTEM_TIME { \
    4, \
    "SYSTEM_TIME", \
    2, \
    {  { "time_unix_usec", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_system_time_t, time_unix_usec) }, \
         { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 8, offsetof(mace_system_time_t, time_boot_ms) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_SYSTEM_TIME { \
    "SYSTEM_TIME", \
    2, \
    {  { "time_unix_usec", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_system_time_t, time_unix_usec) }, \
         { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 8, offsetof(mace_system_time_t, time_boot_ms) }, \
         } \
}
#endif

/**
 * @brief Pack a system_time message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_system_time_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t time_unix_usec, uint32_t time_boot_ms)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_SYSTEM_TIME_LEN];
    _mace_put_uint64_t(buf, 0, time_unix_usec);
    _mace_put_uint32_t(buf, 8, time_boot_ms);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_SYSTEM_TIME_LEN);
#else
    mace_system_time_t packet;
    packet.time_unix_usec = time_unix_usec;
    packet.time_boot_ms = time_boot_ms;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_SYSTEM_TIME_LEN);
#endif

    msg->msgid = MACE_MSG_ID_SYSTEM_TIME;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_SYSTEM_TIME_MIN_LEN, MACE_MSG_ID_SYSTEM_TIME_LEN, MACE_MSG_ID_SYSTEM_TIME_CRC);
}

/**
 * @brief Pack a system_time message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_system_time_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t time_unix_usec,uint32_t time_boot_ms)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_SYSTEM_TIME_LEN];
    _mace_put_uint64_t(buf, 0, time_unix_usec);
    _mace_put_uint32_t(buf, 8, time_boot_ms);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_SYSTEM_TIME_LEN);
#else
    mace_system_time_t packet;
    packet.time_unix_usec = time_unix_usec;
    packet.time_boot_ms = time_boot_ms;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_SYSTEM_TIME_LEN);
#endif

    msg->msgid = MACE_MSG_ID_SYSTEM_TIME;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_SYSTEM_TIME_MIN_LEN, MACE_MSG_ID_SYSTEM_TIME_LEN, MACE_MSG_ID_SYSTEM_TIME_CRC);
}

/**
 * @brief Encode a system_time struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param system_time C-struct to read the message contents from
 */
static inline uint16_t mace_msg_system_time_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_system_time_t* system_time)
{
    return mace_msg_system_time_pack(system_id, component_id, msg, system_time->time_unix_usec, system_time->time_boot_ms);
}

/**
 * @brief Encode a system_time struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param system_time C-struct to read the message contents from
 */
static inline uint16_t mace_msg_system_time_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_system_time_t* system_time)
{
    return mace_msg_system_time_pack_chan(system_id, component_id, chan, msg, system_time->time_unix_usec, system_time->time_boot_ms);
}

/**
 * @brief Send a system_time message
 * @param chan MAVLink channel to send the message
 *
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_system_time_send(mace_channel_t chan, uint64_t time_unix_usec, uint32_t time_boot_ms)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_SYSTEM_TIME_LEN];
    _mace_put_uint64_t(buf, 0, time_unix_usec);
    _mace_put_uint32_t(buf, 8, time_boot_ms);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SYSTEM_TIME, buf, MACE_MSG_ID_SYSTEM_TIME_MIN_LEN, MACE_MSG_ID_SYSTEM_TIME_LEN, MACE_MSG_ID_SYSTEM_TIME_CRC);
#else
    mace_system_time_t packet;
    packet.time_unix_usec = time_unix_usec;
    packet.time_boot_ms = time_boot_ms;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SYSTEM_TIME, (const char *)&packet, MACE_MSG_ID_SYSTEM_TIME_MIN_LEN, MACE_MSG_ID_SYSTEM_TIME_LEN, MACE_MSG_ID_SYSTEM_TIME_CRC);
#endif
}

/**
 * @brief Send a system_time message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_system_time_send_struct(mace_channel_t chan, const mace_system_time_t* system_time)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_system_time_send(chan, system_time->time_unix_usec, system_time->time_boot_ms);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SYSTEM_TIME, (const char *)system_time, MACE_MSG_ID_SYSTEM_TIME_MIN_LEN, MACE_MSG_ID_SYSTEM_TIME_LEN, MACE_MSG_ID_SYSTEM_TIME_CRC);
#endif
}

#if MACE_MSG_ID_SYSTEM_TIME_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_system_time_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t time_unix_usec, uint32_t time_boot_ms)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, time_unix_usec);
    _mace_put_uint32_t(buf, 8, time_boot_ms);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SYSTEM_TIME, buf, MACE_MSG_ID_SYSTEM_TIME_MIN_LEN, MACE_MSG_ID_SYSTEM_TIME_LEN, MACE_MSG_ID_SYSTEM_TIME_CRC);
#else
    mace_system_time_t *packet = (mace_system_time_t *)msgbuf;
    packet->time_unix_usec = time_unix_usec;
    packet->time_boot_ms = time_boot_ms;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SYSTEM_TIME, (const char *)packet, MACE_MSG_ID_SYSTEM_TIME_MIN_LEN, MACE_MSG_ID_SYSTEM_TIME_LEN, MACE_MSG_ID_SYSTEM_TIME_CRC);
#endif
}
#endif

#endif

// MESSAGE SYSTEM_TIME UNPACKING


/**
 * @brief Get field time_unix_usec from system_time message
 *
 * @return Timestamp of the master clock in microseconds since UNIX epoch.
 */
static inline uint64_t mace_msg_system_time_get_time_unix_usec(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_boot_ms from system_time message
 *
 * @return Timestamp of the component clock since boot time in milliseconds.
 */
static inline uint32_t mace_msg_system_time_get_time_boot_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Decode a system_time message into a struct
 *
 * @param msg The message to decode
 * @param system_time C-struct to decode the message contents into
 */
static inline void mace_msg_system_time_decode(const mace_message_t* msg, mace_system_time_t* system_time)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    system_time->time_unix_usec = mace_msg_system_time_get_time_unix_usec(msg);
    system_time->time_boot_ms = mace_msg_system_time_get_time_boot_ms(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_SYSTEM_TIME_LEN? msg->len : MACE_MSG_ID_SYSTEM_TIME_LEN;
        memset(system_time, 0, MACE_MSG_ID_SYSTEM_TIME_LEN);
    memcpy(system_time, _MACE_PAYLOAD(msg), len);
#endif
}
