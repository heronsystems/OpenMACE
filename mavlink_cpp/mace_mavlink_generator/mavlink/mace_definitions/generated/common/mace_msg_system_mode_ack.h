#pragma once
// MESSAGE SYSTEM_MODE_ACK PACKING

#define MACE_MSG_ID_SYSTEM_MODE_ACK 34

MACEPACKED(
typedef struct __mace_system_mode_ack_t {
 uint8_t result; /*< See UXV_RESULT enum*/
}) mace_system_mode_ack_t;

#define MACE_MSG_ID_SYSTEM_MODE_ACK_LEN 1
#define MACE_MSG_ID_SYSTEM_MODE_ACK_MIN_LEN 1
#define MACE_MSG_ID_34_LEN 1
#define MACE_MSG_ID_34_MIN_LEN 1

#define MACE_MSG_ID_SYSTEM_MODE_ACK_CRC 1
#define MACE_MSG_ID_34_CRC 1



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_SYSTEM_MODE_ACK { \
    34, \
    "SYSTEM_MODE_ACK", \
    1, \
    {  { "result", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_system_mode_ack_t, result) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_SYSTEM_MODE_ACK { \
    "SYSTEM_MODE_ACK", \
    1, \
    {  { "result", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_system_mode_ack_t, result) }, \
         } \
}
#endif

/**
 * @brief Pack a system_mode_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param result See UXV_RESULT enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_system_mode_ack_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t result)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_SYSTEM_MODE_ACK_LEN];
    _mace_put_uint8_t(buf, 0, result);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_SYSTEM_MODE_ACK_LEN);
#else
    mace_system_mode_ack_t packet;
    packet.result = result;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_SYSTEM_MODE_ACK_LEN);
#endif

    msg->msgid = MACE_MSG_ID_SYSTEM_MODE_ACK;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_SYSTEM_MODE_ACK_MIN_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_CRC);
}

/**
 * @brief Pack a system_mode_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param result See UXV_RESULT enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_system_mode_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t result)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_SYSTEM_MODE_ACK_LEN];
    _mace_put_uint8_t(buf, 0, result);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_SYSTEM_MODE_ACK_LEN);
#else
    mace_system_mode_ack_t packet;
    packet.result = result;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_SYSTEM_MODE_ACK_LEN);
#endif

    msg->msgid = MACE_MSG_ID_SYSTEM_MODE_ACK;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_SYSTEM_MODE_ACK_MIN_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_CRC);
}

/**
 * @brief Encode a system_mode_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param system_mode_ack C-struct to read the message contents from
 */
static inline uint16_t mace_msg_system_mode_ack_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_system_mode_ack_t* system_mode_ack)
{
    return mace_msg_system_mode_ack_pack(system_id, component_id, msg, system_mode_ack->result);
}

/**
 * @brief Encode a system_mode_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param system_mode_ack C-struct to read the message contents from
 */
static inline uint16_t mace_msg_system_mode_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_system_mode_ack_t* system_mode_ack)
{
    return mace_msg_system_mode_ack_pack_chan(system_id, component_id, chan, msg, system_mode_ack->result);
}

/**
 * @brief Send a system_mode_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param result See UXV_RESULT enum
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_system_mode_ack_send(mace_channel_t chan, uint8_t result)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_SYSTEM_MODE_ACK_LEN];
    _mace_put_uint8_t(buf, 0, result);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SYSTEM_MODE_ACK, buf, MACE_MSG_ID_SYSTEM_MODE_ACK_MIN_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_CRC);
#else
    mace_system_mode_ack_t packet;
    packet.result = result;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SYSTEM_MODE_ACK, (const char *)&packet, MACE_MSG_ID_SYSTEM_MODE_ACK_MIN_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_CRC);
#endif
}

/**
 * @brief Send a system_mode_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_system_mode_ack_send_struct(mace_channel_t chan, const mace_system_mode_ack_t* system_mode_ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_system_mode_ack_send(chan, system_mode_ack->result);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SYSTEM_MODE_ACK, (const char *)system_mode_ack, MACE_MSG_ID_SYSTEM_MODE_ACK_MIN_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_CRC);
#endif
}

#if MACE_MSG_ID_SYSTEM_MODE_ACK_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_system_mode_ack_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t result)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, result);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SYSTEM_MODE_ACK, buf, MACE_MSG_ID_SYSTEM_MODE_ACK_MIN_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_CRC);
#else
    mace_system_mode_ack_t *packet = (mace_system_mode_ack_t *)msgbuf;
    packet->result = result;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SYSTEM_MODE_ACK, (const char *)packet, MACE_MSG_ID_SYSTEM_MODE_ACK_MIN_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_LEN, MACE_MSG_ID_SYSTEM_MODE_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE SYSTEM_MODE_ACK UNPACKING


/**
 * @brief Get field result from system_mode_ack message
 *
 * @return See UXV_RESULT enum
 */
static inline uint8_t mace_msg_system_mode_ack_get_result(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a system_mode_ack message into a struct
 *
 * @param msg The message to decode
 * @param system_mode_ack C-struct to decode the message contents into
 */
static inline void mace_msg_system_mode_ack_decode(const mace_message_t* msg, mace_system_mode_ack_t* system_mode_ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    system_mode_ack->result = mace_msg_system_mode_ack_get_result(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_SYSTEM_MODE_ACK_LEN? msg->len : MACE_MSG_ID_SYSTEM_MODE_ACK_LEN;
        memset(system_mode_ack, 0, MACE_MSG_ID_SYSTEM_MODE_ACK_LEN);
    memcpy(system_mode_ack, _MACE_PAYLOAD(msg), len);
#endif
}
