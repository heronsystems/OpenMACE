#pragma once
// MESSAGE COMMAND_SYSTEM_MODE PACKING

#define MACE_MSG_ID_COMMAND_SYSTEM_MODE 33

MACEPACKED(
typedef struct __mace_command_system_mode_t {
 uint8_t target_system; /*< System which should execute the command*/
 char mode[20]; /*< Char string of the desired flight mode. 20 char limit.*/
}) mace_command_system_mode_t;

#define MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN 21
#define MACE_MSG_ID_COMMAND_SYSTEM_MODE_MIN_LEN 21
#define MACE_MSG_ID_33_LEN 21
#define MACE_MSG_ID_33_MIN_LEN 21

#define MACE_MSG_ID_COMMAND_SYSTEM_MODE_CRC 129
#define MACE_MSG_ID_33_CRC 129

#define MACE_MSG_COMMAND_SYSTEM_MODE_FIELD_MODE_LEN 20

#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_COMMAND_SYSTEM_MODE { \
    33, \
    "COMMAND_SYSTEM_MODE", \
    2, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_command_system_mode_t, target_system) }, \
         { "mode", NULL, MACE_TYPE_CHAR, 20, 1, offsetof(mace_command_system_mode_t, mode) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_COMMAND_SYSTEM_MODE { \
    "COMMAND_SYSTEM_MODE", \
    2, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_command_system_mode_t, target_system) }, \
         { "mode", NULL, MACE_TYPE_CHAR, 20, 1, offsetof(mace_command_system_mode_t, mode) }, \
         } \
}
#endif

/**
 * @brief Pack a command_system_mode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System which should execute the command
 * @param mode Char string of the desired flight mode. 20 char limit.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_command_system_mode_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, const char *mode)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_char_array(buf, 1, mode, 20);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN);
#else
    mace_command_system_mode_t packet;
    packet.target_system = target_system;
    mace_array_memcpy(packet.mode, mode, sizeof(char)*20);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_COMMAND_SYSTEM_MODE;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_COMMAND_SYSTEM_MODE_MIN_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_CRC);
}

/**
 * @brief Pack a command_system_mode message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System which should execute the command
 * @param mode Char string of the desired flight mode. 20 char limit.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_command_system_mode_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,const char *mode)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_char_array(buf, 1, mode, 20);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN);
#else
    mace_command_system_mode_t packet;
    packet.target_system = target_system;
    mace_array_memcpy(packet.mode, mode, sizeof(char)*20);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_COMMAND_SYSTEM_MODE;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_COMMAND_SYSTEM_MODE_MIN_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_CRC);
}

/**
 * @brief Encode a command_system_mode struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_system_mode C-struct to read the message contents from
 */
static inline uint16_t mace_msg_command_system_mode_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_command_system_mode_t* command_system_mode)
{
    return mace_msg_command_system_mode_pack(system_id, component_id, msg, command_system_mode->target_system, command_system_mode->mode);
}

/**
 * @brief Encode a command_system_mode struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_system_mode C-struct to read the message contents from
 */
static inline uint16_t mace_msg_command_system_mode_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_command_system_mode_t* command_system_mode)
{
    return mace_msg_command_system_mode_pack_chan(system_id, component_id, chan, msg, command_system_mode->target_system, command_system_mode->mode);
}

/**
 * @brief Send a command_system_mode message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System which should execute the command
 * @param mode Char string of the desired flight mode. 20 char limit.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_command_system_mode_send(mace_channel_t chan, uint8_t target_system, const char *mode)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_char_array(buf, 1, mode, 20);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_SYSTEM_MODE, buf, MACE_MSG_ID_COMMAND_SYSTEM_MODE_MIN_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_CRC);
#else
    mace_command_system_mode_t packet;
    packet.target_system = target_system;
    mace_array_memcpy(packet.mode, mode, sizeof(char)*20);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_SYSTEM_MODE, (const char *)&packet, MACE_MSG_ID_COMMAND_SYSTEM_MODE_MIN_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_CRC);
#endif
}

/**
 * @brief Send a command_system_mode message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_command_system_mode_send_struct(mace_channel_t chan, const mace_command_system_mode_t* command_system_mode)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_command_system_mode_send(chan, command_system_mode->target_system, command_system_mode->mode);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_SYSTEM_MODE, (const char *)command_system_mode, MACE_MSG_ID_COMMAND_SYSTEM_MODE_MIN_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_CRC);
#endif
}

#if MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_command_system_mode_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, const char *mode)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_char_array(buf, 1, mode, 20);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_SYSTEM_MODE, buf, MACE_MSG_ID_COMMAND_SYSTEM_MODE_MIN_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_CRC);
#else
    mace_command_system_mode_t *packet = (mace_command_system_mode_t *)msgbuf;
    packet->target_system = target_system;
    mace_array_memcpy(packet->mode, mode, sizeof(char)*20);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_SYSTEM_MODE, (const char *)packet, MACE_MSG_ID_COMMAND_SYSTEM_MODE_MIN_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN, MACE_MSG_ID_COMMAND_SYSTEM_MODE_CRC);
#endif
}
#endif

#endif

// MESSAGE COMMAND_SYSTEM_MODE UNPACKING


/**
 * @brief Get field target_system from command_system_mode message
 *
 * @return System which should execute the command
 */
static inline uint8_t mace_msg_command_system_mode_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mode from command_system_mode message
 *
 * @return Char string of the desired flight mode. 20 char limit.
 */
static inline uint16_t mace_msg_command_system_mode_get_mode(const mace_message_t* msg, char *mode)
{
    return _MACE_RETURN_char_array(msg, mode, 20,  1);
}

/**
 * @brief Decode a command_system_mode message into a struct
 *
 * @param msg The message to decode
 * @param command_system_mode C-struct to decode the message contents into
 */
static inline void mace_msg_command_system_mode_decode(const mace_message_t* msg, mace_command_system_mode_t* command_system_mode)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    command_system_mode->target_system = mace_msg_command_system_mode_get_target_system(msg);
    mace_msg_command_system_mode_get_mode(msg, command_system_mode->mode);
#else
        uint8_t len = msg->len < MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN? msg->len : MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN;
        memset(command_system_mode, 0, MACE_MSG_ID_COMMAND_SYSTEM_MODE_LEN);
    memcpy(command_system_mode, _MACE_PAYLOAD(msg), len);
#endif
}
