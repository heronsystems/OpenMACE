#pragma once
// MESSAGE AUTH_KEY PACKING

#define MACE_MSG_ID_AUTH_KEY 8

MACEPACKED(
typedef struct __mace_auth_key_t {
 char key[32]; /*< key*/
}) mace_auth_key_t;

#define MACE_MSG_ID_AUTH_KEY_LEN 32
#define MACE_MSG_ID_AUTH_KEY_MIN_LEN 32
#define MACE_MSG_ID_8_LEN 32
#define MACE_MSG_ID_8_MIN_LEN 32

#define MACE_MSG_ID_AUTH_KEY_CRC 119
#define MACE_MSG_ID_8_CRC 119

#define MACE_MSG_AUTH_KEY_FIELD_KEY_LEN 32

#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUTH_KEY { \
    8, \
    "AUTH_KEY", \
    1, \
    {  { "key", NULL, MACE_TYPE_CHAR, 32, 0, offsetof(mace_auth_key_t, key) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUTH_KEY { \
    "AUTH_KEY", \
    1, \
    {  { "key", NULL, MACE_TYPE_CHAR, 32, 0, offsetof(mace_auth_key_t, key) }, \
         } \
}
#endif

/**
 * @brief Pack a auth_key message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param key key
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auth_key_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               const char *key)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUTH_KEY_LEN];

    _mace_put_char_array(buf, 0, key, 32);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUTH_KEY_LEN);
#else
    mace_auth_key_t packet;

    mace_array_memcpy(packet.key, key, sizeof(char)*32);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUTH_KEY_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUTH_KEY;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUTH_KEY_MIN_LEN, MACE_MSG_ID_AUTH_KEY_LEN, MACE_MSG_ID_AUTH_KEY_CRC);
}

/**
 * @brief Pack a auth_key message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param key key
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auth_key_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   const char *key)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUTH_KEY_LEN];

    _mace_put_char_array(buf, 0, key, 32);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUTH_KEY_LEN);
#else
    mace_auth_key_t packet;

    mace_array_memcpy(packet.key, key, sizeof(char)*32);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUTH_KEY_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUTH_KEY;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUTH_KEY_MIN_LEN, MACE_MSG_ID_AUTH_KEY_LEN, MACE_MSG_ID_AUTH_KEY_CRC);
}

/**
 * @brief Encode a auth_key struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auth_key C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auth_key_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auth_key_t* auth_key)
{
    return mace_msg_auth_key_pack(system_id, component_id, msg, auth_key->key);
}

/**
 * @brief Encode a auth_key struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auth_key C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auth_key_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auth_key_t* auth_key)
{
    return mace_msg_auth_key_pack_chan(system_id, component_id, chan, msg, auth_key->key);
}

/**
 * @brief Send a auth_key message
 * @param chan MAVLink channel to send the message
 *
 * @param key key
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auth_key_send(mace_channel_t chan, const char *key)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUTH_KEY_LEN];

    _mace_put_char_array(buf, 0, key, 32);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUTH_KEY, buf, MACE_MSG_ID_AUTH_KEY_MIN_LEN, MACE_MSG_ID_AUTH_KEY_LEN, MACE_MSG_ID_AUTH_KEY_CRC);
#else
    mace_auth_key_t packet;

    mace_array_memcpy(packet.key, key, sizeof(char)*32);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUTH_KEY, (const char *)&packet, MACE_MSG_ID_AUTH_KEY_MIN_LEN, MACE_MSG_ID_AUTH_KEY_LEN, MACE_MSG_ID_AUTH_KEY_CRC);
#endif
}

/**
 * @brief Send a auth_key message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auth_key_send_struct(mace_channel_t chan, const mace_auth_key_t* auth_key)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auth_key_send(chan, auth_key->key);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUTH_KEY, (const char *)auth_key, MACE_MSG_ID_AUTH_KEY_MIN_LEN, MACE_MSG_ID_AUTH_KEY_LEN, MACE_MSG_ID_AUTH_KEY_CRC);
#endif
}

#if MACE_MSG_ID_AUTH_KEY_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auth_key_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  const char *key)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mace_put_char_array(buf, 0, key, 32);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUTH_KEY, buf, MACE_MSG_ID_AUTH_KEY_MIN_LEN, MACE_MSG_ID_AUTH_KEY_LEN, MACE_MSG_ID_AUTH_KEY_CRC);
#else
    mace_auth_key_t *packet = (mace_auth_key_t *)msgbuf;

    mace_array_memcpy(packet->key, key, sizeof(char)*32);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUTH_KEY, (const char *)packet, MACE_MSG_ID_AUTH_KEY_MIN_LEN, MACE_MSG_ID_AUTH_KEY_LEN, MACE_MSG_ID_AUTH_KEY_CRC);
#endif
}
#endif

#endif

// MESSAGE AUTH_KEY UNPACKING


/**
 * @brief Get field key from auth_key message
 *
 * @return key
 */
static inline uint16_t mace_msg_auth_key_get_key(const mace_message_t* msg, char *key)
{
    return _MACE_RETURN_char_array(msg, key, 32,  0);
}

/**
 * @brief Decode a auth_key message into a struct
 *
 * @param msg The message to decode
 * @param auth_key C-struct to decode the message contents into
 */
static inline void mace_msg_auth_key_decode(const mace_message_t* msg, mace_auth_key_t* auth_key)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auth_key_get_key(msg, auth_key->key);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUTH_KEY_LEN? msg->len : MACE_MSG_ID_AUTH_KEY_LEN;
        memset(auth_key, 0, MACE_MSG_ID_AUTH_KEY_LEN);
    memcpy(auth_key, _MACE_PAYLOAD(msg), len);
#endif
}
