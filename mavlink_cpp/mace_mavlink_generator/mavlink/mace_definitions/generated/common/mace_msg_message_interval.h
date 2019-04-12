#pragma once
// MESSAGE MESSAGE_INTERVAL PACKING

#define MACE_MSG_ID_MESSAGE_INTERVAL 244

MACEPACKED(
typedef struct __mace_message_interval_t {
 int32_t interval_us; /*< The interval between two messages, in microseconds. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.*/
 uint16_t message_id; /*< The ID of the requested MAVLink message. v1.0 is limited to 254 messages.*/
}) mace_message_interval_t;

#define MACE_MSG_ID_MESSAGE_INTERVAL_LEN 6
#define MACE_MSG_ID_MESSAGE_INTERVAL_MIN_LEN 6
#define MACE_MSG_ID_244_LEN 6
#define MACE_MSG_ID_244_MIN_LEN 6

#define MACE_MSG_ID_MESSAGE_INTERVAL_CRC 95
#define MACE_MSG_ID_244_CRC 95



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_MESSAGE_INTERVAL { \
    244, \
    "MESSAGE_INTERVAL", \
    2, \
    {  { "interval_us", NULL, MACE_TYPE_INT32_T, 0, 0, offsetof(mace_message_interval_t, interval_us) }, \
         { "message_id", NULL, MACE_TYPE_UINT16_T, 0, 4, offsetof(mace_message_interval_t, message_id) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_MESSAGE_INTERVAL { \
    "MESSAGE_INTERVAL", \
    2, \
    {  { "interval_us", NULL, MACE_TYPE_INT32_T, 0, 0, offsetof(mace_message_interval_t, interval_us) }, \
         { "message_id", NULL, MACE_TYPE_UINT16_T, 0, 4, offsetof(mace_message_interval_t, message_id) }, \
         } \
}
#endif

/**
 * @brief Pack a message_interval message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param message_id The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
 * @param interval_us The interval between two messages, in microseconds. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_message_interval_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint16_t message_id, int32_t interval_us)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MESSAGE_INTERVAL_LEN];
    _mace_put_int32_t(buf, 0, interval_us);
    _mace_put_uint16_t(buf, 4, message_id);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MESSAGE_INTERVAL_LEN);
#else
    mace_message_interval_t packet;
    packet.interval_us = interval_us;
    packet.message_id = message_id;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MESSAGE_INTERVAL_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MESSAGE_INTERVAL;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_CRC);
}

/**
 * @brief Pack a message_interval message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param message_id The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
 * @param interval_us The interval between two messages, in microseconds. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_message_interval_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint16_t message_id,int32_t interval_us)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MESSAGE_INTERVAL_LEN];
    _mace_put_int32_t(buf, 0, interval_us);
    _mace_put_uint16_t(buf, 4, message_id);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MESSAGE_INTERVAL_LEN);
#else
    mace_message_interval_t packet;
    packet.interval_us = interval_us;
    packet.message_id = message_id;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MESSAGE_INTERVAL_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MESSAGE_INTERVAL;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_CRC);
}

/**
 * @brief Encode a message_interval struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param message_interval C-struct to read the message contents from
 */
static inline uint16_t mace_msg_message_interval_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_message_interval_t* message_interval)
{
    return mace_msg_message_interval_pack(system_id, component_id, msg, message_interval->message_id, message_interval->interval_us);
}

/**
 * @brief Encode a message_interval struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param message_interval C-struct to read the message contents from
 */
static inline uint16_t mace_msg_message_interval_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_message_interval_t* message_interval)
{
    return mace_msg_message_interval_pack_chan(system_id, component_id, chan, msg, message_interval->message_id, message_interval->interval_us);
}

/**
 * @brief Send a message_interval message
 * @param chan MAVLink channel to send the message
 *
 * @param message_id The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
 * @param interval_us The interval between two messages, in microseconds. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_message_interval_send(mace_channel_t chan, uint16_t message_id, int32_t interval_us)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MESSAGE_INTERVAL_LEN];
    _mace_put_int32_t(buf, 0, interval_us);
    _mace_put_uint16_t(buf, 4, message_id);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MESSAGE_INTERVAL, buf, MACE_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_CRC);
#else
    mace_message_interval_t packet;
    packet.interval_us = interval_us;
    packet.message_id = message_id;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MESSAGE_INTERVAL, (const char *)&packet, MACE_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_CRC);
#endif
}

/**
 * @brief Send a message_interval message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_message_interval_send_struct(mace_channel_t chan, const mace_message_interval_t* message_interval)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_message_interval_send(chan, message_interval->message_id, message_interval->interval_us);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MESSAGE_INTERVAL, (const char *)message_interval, MACE_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_CRC);
#endif
}

#if MACE_MSG_ID_MESSAGE_INTERVAL_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_message_interval_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint16_t message_id, int32_t interval_us)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_int32_t(buf, 0, interval_us);
    _mace_put_uint16_t(buf, 4, message_id);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MESSAGE_INTERVAL, buf, MACE_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_CRC);
#else
    mace_message_interval_t *packet = (mace_message_interval_t *)msgbuf;
    packet->interval_us = interval_us;
    packet->message_id = message_id;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MESSAGE_INTERVAL, (const char *)packet, MACE_MSG_ID_MESSAGE_INTERVAL_MIN_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_LEN, MACE_MSG_ID_MESSAGE_INTERVAL_CRC);
#endif
}
#endif

#endif

// MESSAGE MESSAGE_INTERVAL UNPACKING


/**
 * @brief Get field message_id from message_interval message
 *
 * @return The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
 */
static inline uint16_t mace_msg_message_interval_get_message_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field interval_us from message_interval message
 *
 * @return The interval between two messages, in microseconds. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
 */
static inline int32_t mace_msg_message_interval_get_interval_us(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  0);
}

/**
 * @brief Decode a message_interval message into a struct
 *
 * @param msg The message to decode
 * @param message_interval C-struct to decode the message contents into
 */
static inline void mace_msg_message_interval_decode(const mace_message_t* msg, mace_message_interval_t* message_interval)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    message_interval->interval_us = mace_msg_message_interval_get_interval_us(msg);
    message_interval->message_id = mace_msg_message_interval_get_message_id(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_MESSAGE_INTERVAL_LEN? msg->len : MACE_MSG_ID_MESSAGE_INTERVAL_LEN;
        memset(message_interval, 0, MACE_MSG_ID_MESSAGE_INTERVAL_LEN);
    memcpy(message_interval, _MACE_PAYLOAD(msg), len);
#endif
}
