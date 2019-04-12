#pragma once
// MESSAGE TIMESYNC PACKING

#define MACE_MSG_ID_TIMESYNC 201

MACEPACKED(
typedef struct __mace_timesync_t {
 int64_t tc1; /*< Time sync timestamp 1*/
 int64_t ts1; /*< Time sync timestamp 2*/
}) mace_timesync_t;

#define MACE_MSG_ID_TIMESYNC_LEN 16
#define MACE_MSG_ID_TIMESYNC_MIN_LEN 16
#define MACE_MSG_ID_201_LEN 16
#define MACE_MSG_ID_201_MIN_LEN 16

#define MACE_MSG_ID_TIMESYNC_CRC 34
#define MACE_MSG_ID_201_CRC 34



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_TIMESYNC { \
    201, \
    "TIMESYNC", \
    2, \
    {  { "tc1", NULL, MACE_TYPE_INT64_T, 0, 0, offsetof(mace_timesync_t, tc1) }, \
         { "ts1", NULL, MACE_TYPE_INT64_T, 0, 8, offsetof(mace_timesync_t, ts1) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_TIMESYNC { \
    "TIMESYNC", \
    2, \
    {  { "tc1", NULL, MACE_TYPE_INT64_T, 0, 0, offsetof(mace_timesync_t, tc1) }, \
         { "ts1", NULL, MACE_TYPE_INT64_T, 0, 8, offsetof(mace_timesync_t, ts1) }, \
         } \
}
#endif

/**
 * @brief Pack a timesync message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param tc1 Time sync timestamp 1
 * @param ts1 Time sync timestamp 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_timesync_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               int64_t tc1, int64_t ts1)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_TIMESYNC_LEN];
    _mace_put_int64_t(buf, 0, tc1);
    _mace_put_int64_t(buf, 8, ts1);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_TIMESYNC_LEN);
#else
    mace_timesync_t packet;
    packet.tc1 = tc1;
    packet.ts1 = ts1;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_TIMESYNC_LEN);
#endif

    msg->msgid = MACE_MSG_ID_TIMESYNC;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_TIMESYNC_MIN_LEN, MACE_MSG_ID_TIMESYNC_LEN, MACE_MSG_ID_TIMESYNC_CRC);
}

/**
 * @brief Pack a timesync message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tc1 Time sync timestamp 1
 * @param ts1 Time sync timestamp 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_timesync_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   int64_t tc1,int64_t ts1)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_TIMESYNC_LEN];
    _mace_put_int64_t(buf, 0, tc1);
    _mace_put_int64_t(buf, 8, ts1);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_TIMESYNC_LEN);
#else
    mace_timesync_t packet;
    packet.tc1 = tc1;
    packet.ts1 = ts1;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_TIMESYNC_LEN);
#endif

    msg->msgid = MACE_MSG_ID_TIMESYNC;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_TIMESYNC_MIN_LEN, MACE_MSG_ID_TIMESYNC_LEN, MACE_MSG_ID_TIMESYNC_CRC);
}

/**
 * @brief Encode a timesync struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param timesync C-struct to read the message contents from
 */
static inline uint16_t mace_msg_timesync_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_timesync_t* timesync)
{
    return mace_msg_timesync_pack(system_id, component_id, msg, timesync->tc1, timesync->ts1);
}

/**
 * @brief Encode a timesync struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timesync C-struct to read the message contents from
 */
static inline uint16_t mace_msg_timesync_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_timesync_t* timesync)
{
    return mace_msg_timesync_pack_chan(system_id, component_id, chan, msg, timesync->tc1, timesync->ts1);
}

/**
 * @brief Send a timesync message
 * @param chan MAVLink channel to send the message
 *
 * @param tc1 Time sync timestamp 1
 * @param ts1 Time sync timestamp 2
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_timesync_send(mace_channel_t chan, int64_t tc1, int64_t ts1)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_TIMESYNC_LEN];
    _mace_put_int64_t(buf, 0, tc1);
    _mace_put_int64_t(buf, 8, ts1);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_TIMESYNC, buf, MACE_MSG_ID_TIMESYNC_MIN_LEN, MACE_MSG_ID_TIMESYNC_LEN, MACE_MSG_ID_TIMESYNC_CRC);
#else
    mace_timesync_t packet;
    packet.tc1 = tc1;
    packet.ts1 = ts1;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_TIMESYNC, (const char *)&packet, MACE_MSG_ID_TIMESYNC_MIN_LEN, MACE_MSG_ID_TIMESYNC_LEN, MACE_MSG_ID_TIMESYNC_CRC);
#endif
}

/**
 * @brief Send a timesync message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_timesync_send_struct(mace_channel_t chan, const mace_timesync_t* timesync)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_timesync_send(chan, timesync->tc1, timesync->ts1);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_TIMESYNC, (const char *)timesync, MACE_MSG_ID_TIMESYNC_MIN_LEN, MACE_MSG_ID_TIMESYNC_LEN, MACE_MSG_ID_TIMESYNC_CRC);
#endif
}

#if MACE_MSG_ID_TIMESYNC_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_timesync_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  int64_t tc1, int64_t ts1)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_int64_t(buf, 0, tc1);
    _mace_put_int64_t(buf, 8, ts1);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_TIMESYNC, buf, MACE_MSG_ID_TIMESYNC_MIN_LEN, MACE_MSG_ID_TIMESYNC_LEN, MACE_MSG_ID_TIMESYNC_CRC);
#else
    mace_timesync_t *packet = (mace_timesync_t *)msgbuf;
    packet->tc1 = tc1;
    packet->ts1 = ts1;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_TIMESYNC, (const char *)packet, MACE_MSG_ID_TIMESYNC_MIN_LEN, MACE_MSG_ID_TIMESYNC_LEN, MACE_MSG_ID_TIMESYNC_CRC);
#endif
}
#endif

#endif

// MESSAGE TIMESYNC UNPACKING


/**
 * @brief Get field tc1 from timesync message
 *
 * @return Time sync timestamp 1
 */
static inline int64_t mace_msg_timesync_get_tc1(const mace_message_t* msg)
{
    return _MACE_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field ts1 from timesync message
 *
 * @return Time sync timestamp 2
 */
static inline int64_t mace_msg_timesync_get_ts1(const mace_message_t* msg)
{
    return _MACE_RETURN_int64_t(msg,  8);
}

/**
 * @brief Decode a timesync message into a struct
 *
 * @param msg The message to decode
 * @param timesync C-struct to decode the message contents into
 */
static inline void mace_msg_timesync_decode(const mace_message_t* msg, mace_timesync_t* timesync)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    timesync->tc1 = mace_msg_timesync_get_tc1(msg);
    timesync->ts1 = mace_msg_timesync_get_ts1(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_TIMESYNC_LEN? msg->len : MACE_MSG_ID_TIMESYNC_LEN;
        memset(timesync, 0, MACE_MSG_ID_TIMESYNC_LEN);
    memcpy(timesync, _MACE_PAYLOAD(msg), len);
#endif
}
