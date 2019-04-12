#pragma once
// MESSAGE HOME_POSITION_ACK PACKING

#define MACE_MSG_ID_HOME_POSITION_ACK 119

MACEPACKED(
typedef struct __mace_home_position_ack_t {
 uint8_t target_system; /*< System ID*/
 uint8_t ack; /*< Acknowledgement of the home position set request.*/
}) mace_home_position_ack_t;

#define MACE_MSG_ID_HOME_POSITION_ACK_LEN 2
#define MACE_MSG_ID_HOME_POSITION_ACK_MIN_LEN 2
#define MACE_MSG_ID_119_LEN 2
#define MACE_MSG_ID_119_MIN_LEN 2

#define MACE_MSG_ID_HOME_POSITION_ACK_CRC 21
#define MACE_MSG_ID_119_CRC 21



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_HOME_POSITION_ACK { \
    119, \
    "HOME_POSITION_ACK", \
    2, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_home_position_ack_t, target_system) }, \
         { "ack", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_home_position_ack_t, ack) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_HOME_POSITION_ACK { \
    "HOME_POSITION_ACK", \
    2, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_home_position_ack_t, target_system) }, \
         { "ack", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_home_position_ack_t, ack) }, \
         } \
}
#endif

/**
 * @brief Pack a home_position_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param ack Acknowledgement of the home position set request.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_home_position_ack_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_HOME_POSITION_ACK_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, ack);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_HOME_POSITION_ACK_LEN);
#else
    mace_home_position_ack_t packet;
    packet.target_system = target_system;
    packet.ack = ack;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_HOME_POSITION_ACK_LEN);
#endif

    msg->msgid = MACE_MSG_ID_HOME_POSITION_ACK;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_HOME_POSITION_ACK_MIN_LEN, MACE_MSG_ID_HOME_POSITION_ACK_LEN, MACE_MSG_ID_HOME_POSITION_ACK_CRC);
}

/**
 * @brief Pack a home_position_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param ack Acknowledgement of the home position set request.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_home_position_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_HOME_POSITION_ACK_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, ack);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_HOME_POSITION_ACK_LEN);
#else
    mace_home_position_ack_t packet;
    packet.target_system = target_system;
    packet.ack = ack;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_HOME_POSITION_ACK_LEN);
#endif

    msg->msgid = MACE_MSG_ID_HOME_POSITION_ACK;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_HOME_POSITION_ACK_MIN_LEN, MACE_MSG_ID_HOME_POSITION_ACK_LEN, MACE_MSG_ID_HOME_POSITION_ACK_CRC);
}

/**
 * @brief Encode a home_position_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param home_position_ack C-struct to read the message contents from
 */
static inline uint16_t mace_msg_home_position_ack_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_home_position_ack_t* home_position_ack)
{
    return mace_msg_home_position_ack_pack(system_id, component_id, msg, home_position_ack->target_system, home_position_ack->ack);
}

/**
 * @brief Encode a home_position_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param home_position_ack C-struct to read the message contents from
 */
static inline uint16_t mace_msg_home_position_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_home_position_ack_t* home_position_ack)
{
    return mace_msg_home_position_ack_pack_chan(system_id, component_id, chan, msg, home_position_ack->target_system, home_position_ack->ack);
}

/**
 * @brief Send a home_position_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param ack Acknowledgement of the home position set request.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_home_position_ack_send(mace_channel_t chan, uint8_t target_system, uint8_t ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_HOME_POSITION_ACK_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, ack);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_HOME_POSITION_ACK, buf, MACE_MSG_ID_HOME_POSITION_ACK_MIN_LEN, MACE_MSG_ID_HOME_POSITION_ACK_LEN, MACE_MSG_ID_HOME_POSITION_ACK_CRC);
#else
    mace_home_position_ack_t packet;
    packet.target_system = target_system;
    packet.ack = ack;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_HOME_POSITION_ACK, (const char *)&packet, MACE_MSG_ID_HOME_POSITION_ACK_MIN_LEN, MACE_MSG_ID_HOME_POSITION_ACK_LEN, MACE_MSG_ID_HOME_POSITION_ACK_CRC);
#endif
}

/**
 * @brief Send a home_position_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_home_position_ack_send_struct(mace_channel_t chan, const mace_home_position_ack_t* home_position_ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_home_position_ack_send(chan, home_position_ack->target_system, home_position_ack->ack);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_HOME_POSITION_ACK, (const char *)home_position_ack, MACE_MSG_ID_HOME_POSITION_ACK_MIN_LEN, MACE_MSG_ID_HOME_POSITION_ACK_LEN, MACE_MSG_ID_HOME_POSITION_ACK_CRC);
#endif
}

#if MACE_MSG_ID_HOME_POSITION_ACK_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_home_position_ack_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, ack);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_HOME_POSITION_ACK, buf, MACE_MSG_ID_HOME_POSITION_ACK_MIN_LEN, MACE_MSG_ID_HOME_POSITION_ACK_LEN, MACE_MSG_ID_HOME_POSITION_ACK_CRC);
#else
    mace_home_position_ack_t *packet = (mace_home_position_ack_t *)msgbuf;
    packet->target_system = target_system;
    packet->ack = ack;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_HOME_POSITION_ACK, (const char *)packet, MACE_MSG_ID_HOME_POSITION_ACK_MIN_LEN, MACE_MSG_ID_HOME_POSITION_ACK_LEN, MACE_MSG_ID_HOME_POSITION_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE HOME_POSITION_ACK UNPACKING


/**
 * @brief Get field target_system from home_position_ack message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_home_position_ack_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack from home_position_ack message
 *
 * @return Acknowledgement of the home position set request.
 */
static inline uint8_t mace_msg_home_position_ack_get_ack(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a home_position_ack message into a struct
 *
 * @param msg The message to decode
 * @param home_position_ack C-struct to decode the message contents into
 */
static inline void mace_msg_home_position_ack_decode(const mace_message_t* msg, mace_home_position_ack_t* home_position_ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    home_position_ack->target_system = mace_msg_home_position_ack_get_target_system(msg);
    home_position_ack->ack = mace_msg_home_position_ack_get_ack(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_HOME_POSITION_ACK_LEN? msg->len : MACE_MSG_ID_HOME_POSITION_ACK_LEN;
        memset(home_position_ack, 0, MACE_MSG_ID_HOME_POSITION_ACK_LEN);
    memcpy(home_position_ack, _MACE_PAYLOAD(msg), len);
#endif
}
