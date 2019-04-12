#pragma once
// MESSAGE CHANGE_OPERATOR_CONTROL PACKING

#define MACE_MSG_ID_CHANGE_OPERATOR_CONTROL 6

MACEPACKED(
typedef struct __mace_change_operator_control_t {
 uint8_t target_system; /*< System the GCS requests control for*/
 uint8_t control_request; /*< 0: request control of this MAV, 1: Release control of this MAV*/
 uint8_t version; /*< 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.*/
 char passkey[25]; /*< Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"*/
}) mace_change_operator_control_t;

#define MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN 28
#define MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN 28
#define MACE_MSG_ID_6_LEN 28
#define MACE_MSG_ID_6_MIN_LEN 28

#define MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC 217
#define MACE_MSG_ID_6_CRC 217

#define MACE_MSG_CHANGE_OPERATOR_CONTROL_FIELD_PASSKEY_LEN 25

#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL { \
    6, \
    "CHANGE_OPERATOR_CONTROL", \
    4, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_change_operator_control_t, target_system) }, \
         { "control_request", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_change_operator_control_t, control_request) }, \
         { "version", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_change_operator_control_t, version) }, \
         { "passkey", NULL, MACE_TYPE_CHAR, 25, 3, offsetof(mace_change_operator_control_t, passkey) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL { \
    "CHANGE_OPERATOR_CONTROL", \
    4, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_change_operator_control_t, target_system) }, \
         { "control_request", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_change_operator_control_t, control_request) }, \
         { "version", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_change_operator_control_t, version) }, \
         { "passkey", NULL, MACE_TYPE_CHAR, 25, 3, offsetof(mace_change_operator_control_t, passkey) }, \
         } \
}
#endif

/**
 * @brief Pack a change_operator_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System the GCS requests control for
 * @param control_request 0: request control of this MAV, 1: Release control of this MAV
 * @param version 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
 * @param passkey Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_change_operator_control_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t control_request, uint8_t version, const char *passkey)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, control_request);
    _mace_put_uint8_t(buf, 2, version);
    _mace_put_char_array(buf, 3, passkey, 25);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#else
    mace_change_operator_control_t packet;
    packet.target_system = target_system;
    packet.control_request = control_request;
    packet.version = version;
    mace_array_memcpy(packet.passkey, passkey, sizeof(char)*25);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#endif

    msg->msgid = MACE_MSG_ID_CHANGE_OPERATOR_CONTROL;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
}

/**
 * @brief Pack a change_operator_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System the GCS requests control for
 * @param control_request 0: request control of this MAV, 1: Release control of this MAV
 * @param version 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
 * @param passkey Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_change_operator_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t control_request,uint8_t version,const char *passkey)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, control_request);
    _mace_put_uint8_t(buf, 2, version);
    _mace_put_char_array(buf, 3, passkey, 25);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#else
    mace_change_operator_control_t packet;
    packet.target_system = target_system;
    packet.control_request = control_request;
    packet.version = version;
    mace_array_memcpy(packet.passkey, passkey, sizeof(char)*25);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
#endif

    msg->msgid = MACE_MSG_ID_CHANGE_OPERATOR_CONTROL;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
}

/**
 * @brief Encode a change_operator_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param change_operator_control C-struct to read the message contents from
 */
static inline uint16_t mace_msg_change_operator_control_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_change_operator_control_t* change_operator_control)
{
    return mace_msg_change_operator_control_pack(system_id, component_id, msg, change_operator_control->target_system, change_operator_control->control_request, change_operator_control->version, change_operator_control->passkey);
}

/**
 * @brief Encode a change_operator_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param change_operator_control C-struct to read the message contents from
 */
static inline uint16_t mace_msg_change_operator_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_change_operator_control_t* change_operator_control)
{
    return mace_msg_change_operator_control_pack_chan(system_id, component_id, chan, msg, change_operator_control->target_system, change_operator_control->control_request, change_operator_control->version, change_operator_control->passkey);
}

/**
 * @brief Send a change_operator_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System the GCS requests control for
 * @param control_request 0: request control of this MAV, 1: Release control of this MAV
 * @param version 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
 * @param passkey Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_change_operator_control_send(mace_channel_t chan, uint8_t target_system, uint8_t control_request, uint8_t version, const char *passkey)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, control_request);
    _mace_put_uint8_t(buf, 2, version);
    _mace_put_char_array(buf, 3, passkey, 25);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL, buf, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
#else
    mace_change_operator_control_t packet;
    packet.target_system = target_system;
    packet.control_request = control_request;
    packet.version = version;
    mace_array_memcpy(packet.passkey, passkey, sizeof(char)*25);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL, (const char *)&packet, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
#endif
}

/**
 * @brief Send a change_operator_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_change_operator_control_send_struct(mace_channel_t chan, const mace_change_operator_control_t* change_operator_control)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_change_operator_control_send(chan, change_operator_control->target_system, change_operator_control->control_request, change_operator_control->version, change_operator_control->passkey);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL, (const char *)change_operator_control, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
#endif
}

#if MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_change_operator_control_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t control_request, uint8_t version, const char *passkey)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, control_request);
    _mace_put_uint8_t(buf, 2, version);
    _mace_put_char_array(buf, 3, passkey, 25);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL, buf, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
#else
    mace_change_operator_control_t *packet = (mace_change_operator_control_t *)msgbuf;
    packet->target_system = target_system;
    packet->control_request = control_request;
    packet->version = version;
    mace_array_memcpy(packet->passkey, passkey, sizeof(char)*25);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL, (const char *)packet, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE CHANGE_OPERATOR_CONTROL UNPACKING


/**
 * @brief Get field target_system from change_operator_control message
 *
 * @return System the GCS requests control for
 */
static inline uint8_t mace_msg_change_operator_control_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field control_request from change_operator_control message
 *
 * @return 0: request control of this MAV, 1: Release control of this MAV
 */
static inline uint8_t mace_msg_change_operator_control_get_control_request(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field version from change_operator_control message
 *
 * @return 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
 */
static inline uint8_t mace_msg_change_operator_control_get_version(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field passkey from change_operator_control message
 *
 * @return Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
 */
static inline uint16_t mace_msg_change_operator_control_get_passkey(const mace_message_t* msg, char *passkey)
{
    return _MACE_RETURN_char_array(msg, passkey, 25,  3);
}

/**
 * @brief Decode a change_operator_control message into a struct
 *
 * @param msg The message to decode
 * @param change_operator_control C-struct to decode the message contents into
 */
static inline void mace_msg_change_operator_control_decode(const mace_message_t* msg, mace_change_operator_control_t* change_operator_control)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    change_operator_control->target_system = mace_msg_change_operator_control_get_target_system(msg);
    change_operator_control->control_request = mace_msg_change_operator_control_get_control_request(msg);
    change_operator_control->version = mace_msg_change_operator_control_get_version(msg);
    mace_msg_change_operator_control_get_passkey(msg, change_operator_control->passkey);
#else
        uint8_t len = msg->len < MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN? msg->len : MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN;
        memset(change_operator_control, 0, MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_LEN);
    memcpy(change_operator_control, _MACE_PAYLOAD(msg), len);
#endif
}
