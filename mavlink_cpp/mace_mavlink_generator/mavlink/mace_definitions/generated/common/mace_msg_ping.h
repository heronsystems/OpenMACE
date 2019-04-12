#pragma once
// MESSAGE PING PACKING

#define MACE_MSG_ID_PING 5

MACEPACKED(
typedef struct __mace_ping_t {
 uint64_t time_usec; /*< Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)*/
 uint32_t seq; /*< PING sequence*/
 uint8_t target_system; /*< 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system*/
 uint8_t target_component; /*< 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system*/
}) mace_ping_t;

#define MACE_MSG_ID_PING_LEN 14
#define MACE_MSG_ID_PING_MIN_LEN 14
#define MACE_MSG_ID_5_LEN 14
#define MACE_MSG_ID_5_MIN_LEN 14

#define MACE_MSG_ID_PING_CRC 237
#define MACE_MSG_ID_5_CRC 237



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_PING { \
    5, \
    "PING", \
    4, \
    {  { "time_usec", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_ping_t, time_usec) }, \
         { "seq", NULL, MACE_TYPE_UINT32_T, 0, 8, offsetof(mace_ping_t, seq) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 12, offsetof(mace_ping_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 13, offsetof(mace_ping_t, target_component) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_PING { \
    "PING", \
    4, \
    {  { "time_usec", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_ping_t, time_usec) }, \
         { "seq", NULL, MACE_TYPE_UINT32_T, 0, 8, offsetof(mace_ping_t, seq) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 12, offsetof(mace_ping_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 13, offsetof(mace_ping_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a ping message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_ping_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t time_usec, uint32_t seq, uint8_t target_system, uint8_t target_component)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_PING_LEN];
    _mace_put_uint64_t(buf, 0, time_usec);
    _mace_put_uint32_t(buf, 8, seq);
    _mace_put_uint8_t(buf, 12, target_system);
    _mace_put_uint8_t(buf, 13, target_component);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_PING_LEN);
#else
    mace_ping_t packet;
    packet.time_usec = time_usec;
    packet.seq = seq;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_PING_LEN);
#endif

    msg->msgid = MACE_MSG_ID_PING;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_PING_MIN_LEN, MACE_MSG_ID_PING_LEN, MACE_MSG_ID_PING_CRC);
}

/**
 * @brief Pack a ping message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_ping_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t time_usec,uint32_t seq,uint8_t target_system,uint8_t target_component)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_PING_LEN];
    _mace_put_uint64_t(buf, 0, time_usec);
    _mace_put_uint32_t(buf, 8, seq);
    _mace_put_uint8_t(buf, 12, target_system);
    _mace_put_uint8_t(buf, 13, target_component);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_PING_LEN);
#else
    mace_ping_t packet;
    packet.time_usec = time_usec;
    packet.seq = seq;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_PING_LEN);
#endif

    msg->msgid = MACE_MSG_ID_PING;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_PING_MIN_LEN, MACE_MSG_ID_PING_LEN, MACE_MSG_ID_PING_CRC);
}

/**
 * @brief Encode a ping struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ping C-struct to read the message contents from
 */
static inline uint16_t mace_msg_ping_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_ping_t* ping)
{
    return mace_msg_ping_pack(system_id, component_id, msg, ping->time_usec, ping->seq, ping->target_system, ping->target_component);
}

/**
 * @brief Encode a ping struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ping C-struct to read the message contents from
 */
static inline uint16_t mace_msg_ping_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_ping_t* ping)
{
    return mace_msg_ping_pack_chan(system_id, component_id, chan, msg, ping->time_usec, ping->seq, ping->target_system, ping->target_component);
}

/**
 * @brief Send a ping message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_ping_send(mace_channel_t chan, uint64_t time_usec, uint32_t seq, uint8_t target_system, uint8_t target_component)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_PING_LEN];
    _mace_put_uint64_t(buf, 0, time_usec);
    _mace_put_uint32_t(buf, 8, seq);
    _mace_put_uint8_t(buf, 12, target_system);
    _mace_put_uint8_t(buf, 13, target_component);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PING, buf, MACE_MSG_ID_PING_MIN_LEN, MACE_MSG_ID_PING_LEN, MACE_MSG_ID_PING_CRC);
#else
    mace_ping_t packet;
    packet.time_usec = time_usec;
    packet.seq = seq;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PING, (const char *)&packet, MACE_MSG_ID_PING_MIN_LEN, MACE_MSG_ID_PING_LEN, MACE_MSG_ID_PING_CRC);
#endif
}

/**
 * @brief Send a ping message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_ping_send_struct(mace_channel_t chan, const mace_ping_t* ping)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_ping_send(chan, ping->time_usec, ping->seq, ping->target_system, ping->target_component);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PING, (const char *)ping, MACE_MSG_ID_PING_MIN_LEN, MACE_MSG_ID_PING_LEN, MACE_MSG_ID_PING_CRC);
#endif
}

#if MACE_MSG_ID_PING_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_ping_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t time_usec, uint32_t seq, uint8_t target_system, uint8_t target_component)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, time_usec);
    _mace_put_uint32_t(buf, 8, seq);
    _mace_put_uint8_t(buf, 12, target_system);
    _mace_put_uint8_t(buf, 13, target_component);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PING, buf, MACE_MSG_ID_PING_MIN_LEN, MACE_MSG_ID_PING_LEN, MACE_MSG_ID_PING_CRC);
#else
    mace_ping_t *packet = (mace_ping_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->seq = seq;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PING, (const char *)packet, MACE_MSG_ID_PING_MIN_LEN, MACE_MSG_ID_PING_LEN, MACE_MSG_ID_PING_CRC);
#endif
}
#endif

#endif

// MESSAGE PING UNPACKING


/**
 * @brief Get field time_usec from ping message
 *
 * @return Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
 */
static inline uint64_t mace_msg_ping_get_time_usec(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field seq from ping message
 *
 * @return PING sequence
 */
static inline uint32_t mace_msg_ping_get_seq(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field target_system from ping message
 *
 * @return 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 */
static inline uint8_t mace_msg_ping_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field target_component from ping message
 *
 * @return 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 */
static inline uint8_t mace_msg_ping_get_target_component(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a ping message into a struct
 *
 * @param msg The message to decode
 * @param ping C-struct to decode the message contents into
 */
static inline void mace_msg_ping_decode(const mace_message_t* msg, mace_ping_t* ping)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    ping->time_usec = mace_msg_ping_get_time_usec(msg);
    ping->seq = mace_msg_ping_get_seq(msg);
    ping->target_system = mace_msg_ping_get_target_system(msg);
    ping->target_component = mace_msg_ping_get_target_component(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_PING_LEN? msg->len : MACE_MSG_ID_PING_LEN;
        memset(ping, 0, MACE_MSG_ID_PING_LEN);
    memcpy(ping, _MACE_PAYLOAD(msg), len);
#endif
}
