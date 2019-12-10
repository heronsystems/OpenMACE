#pragma once
// MESSAGE COMMAND_SHORT PACKING

#define MACE_MSG_ID_COMMAND_SHORT 31

MACEPACKED(
typedef struct __mace_command_short_t {
 float param; /*< Parameter as defined by UXV_CMD enum.*/
 uint16_t command; /*< Command ID, as defined by UXV_CMD enum.is was established to reduce the bandwidth required of messages not requiring as much parameterized data.*/
 uint8_t target_system; /*< System which should execute the command*/
 uint8_t target_component; /*< Component which should execute the command, 0 for all components*/
 uint8_t confirmation; /*< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)*/
}) mace_command_short_t;

#define MACE_MSG_ID_COMMAND_SHORT_LEN 9
#define MACE_MSG_ID_COMMAND_SHORT_MIN_LEN 9
#define MACE_MSG_ID_31_LEN 9
#define MACE_MSG_ID_31_MIN_LEN 9

#define MACE_MSG_ID_COMMAND_SHORT_CRC 177
#define MACE_MSG_ID_31_CRC 177



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_COMMAND_SHORT { \
    31, \
    "COMMAND_SHORT", \
    5, \
    {  { "param", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_command_short_t, param) }, \
         { "command", NULL, MACE_TYPE_UINT16_T, 0, 4, offsetof(mace_command_short_t, command) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_command_short_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 7, offsetof(mace_command_short_t, target_component) }, \
         { "confirmation", NULL, MACE_TYPE_UINT8_T, 0, 8, offsetof(mace_command_short_t, confirmation) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_COMMAND_SHORT { \
    "COMMAND_SHORT", \
    5, \
    {  { "param", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_command_short_t, param) }, \
         { "command", NULL, MACE_TYPE_UINT16_T, 0, 4, offsetof(mace_command_short_t, command) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_command_short_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 7, offsetof(mace_command_short_t, target_component) }, \
         { "confirmation", NULL, MACE_TYPE_UINT8_T, 0, 8, offsetof(mace_command_short_t, confirmation) }, \
         } \
}
#endif

/**
 * @brief Pack a command_short message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command Command ID, as defined by UXV_CMD enum.is was established to reduce the bandwidth required of messages not requiring as much parameterized data.
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param Parameter as defined by UXV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_command_short_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint16_t command, uint8_t target_system, uint8_t target_component, uint8_t confirmation, float param)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_SHORT_LEN];
    _mace_put_float(buf, 0, param);
    _mace_put_uint16_t(buf, 4, command);
    _mace_put_uint8_t(buf, 6, target_system);
    _mace_put_uint8_t(buf, 7, target_component);
    _mace_put_uint8_t(buf, 8, confirmation);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_COMMAND_SHORT_LEN);
#else
    mace_command_short_t packet;
    packet.param = param;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.confirmation = confirmation;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_COMMAND_SHORT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_COMMAND_SHORT;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_COMMAND_SHORT_MIN_LEN, MACE_MSG_ID_COMMAND_SHORT_LEN, MACE_MSG_ID_COMMAND_SHORT_CRC);
}

/**
 * @brief Pack a command_short message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command Command ID, as defined by UXV_CMD enum.is was established to reduce the bandwidth required of messages not requiring as much parameterized data.
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param Parameter as defined by UXV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_command_short_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint16_t command,uint8_t target_system,uint8_t target_component,uint8_t confirmation,float param)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_SHORT_LEN];
    _mace_put_float(buf, 0, param);
    _mace_put_uint16_t(buf, 4, command);
    _mace_put_uint8_t(buf, 6, target_system);
    _mace_put_uint8_t(buf, 7, target_component);
    _mace_put_uint8_t(buf, 8, confirmation);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_COMMAND_SHORT_LEN);
#else
    mace_command_short_t packet;
    packet.param = param;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.confirmation = confirmation;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_COMMAND_SHORT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_COMMAND_SHORT;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_COMMAND_SHORT_MIN_LEN, MACE_MSG_ID_COMMAND_SHORT_LEN, MACE_MSG_ID_COMMAND_SHORT_CRC);
}

/**
 * @brief Encode a command_short struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_short C-struct to read the message contents from
 */
static inline uint16_t mace_msg_command_short_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_command_short_t* command_short)
{
    return mace_msg_command_short_pack(system_id, component_id, msg, command_short->command, command_short->target_system, command_short->target_component, command_short->confirmation, command_short->param);
}

/**
 * @brief Encode a command_short struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_short C-struct to read the message contents from
 */
static inline uint16_t mace_msg_command_short_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_command_short_t* command_short)
{
    return mace_msg_command_short_pack_chan(system_id, component_id, chan, msg, command_short->command, command_short->target_system, command_short->target_component, command_short->confirmation, command_short->param);
}

/**
 * @brief Send a command_short message
 * @param chan MAVLink channel to send the message
 *
 * @param command Command ID, as defined by UXV_CMD enum.is was established to reduce the bandwidth required of messages not requiring as much parameterized data.
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param Parameter as defined by UXV_CMD enum.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_command_short_send(mace_channel_t chan, uint16_t command, uint8_t target_system, uint8_t target_component, uint8_t confirmation, float param)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_SHORT_LEN];
    _mace_put_float(buf, 0, param);
    _mace_put_uint16_t(buf, 4, command);
    _mace_put_uint8_t(buf, 6, target_system);
    _mace_put_uint8_t(buf, 7, target_component);
    _mace_put_uint8_t(buf, 8, confirmation);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_SHORT, buf, MACE_MSG_ID_COMMAND_SHORT_MIN_LEN, MACE_MSG_ID_COMMAND_SHORT_LEN, MACE_MSG_ID_COMMAND_SHORT_CRC);
#else
    mace_command_short_t packet;
    packet.param = param;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.confirmation = confirmation;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_SHORT, (const char *)&packet, MACE_MSG_ID_COMMAND_SHORT_MIN_LEN, MACE_MSG_ID_COMMAND_SHORT_LEN, MACE_MSG_ID_COMMAND_SHORT_CRC);
#endif
}

/**
 * @brief Send a command_short message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_command_short_send_struct(mace_channel_t chan, const mace_command_short_t* command_short)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_command_short_send(chan, command_short->command, command_short->target_system, command_short->target_component, command_short->confirmation, command_short->param);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_SHORT, (const char *)command_short, MACE_MSG_ID_COMMAND_SHORT_MIN_LEN, MACE_MSG_ID_COMMAND_SHORT_LEN, MACE_MSG_ID_COMMAND_SHORT_CRC);
#endif
}

#if MACE_MSG_ID_COMMAND_SHORT_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_command_short_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint16_t command, uint8_t target_system, uint8_t target_component, uint8_t confirmation, float param)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_float(buf, 0, param);
    _mace_put_uint16_t(buf, 4, command);
    _mace_put_uint8_t(buf, 6, target_system);
    _mace_put_uint8_t(buf, 7, target_component);
    _mace_put_uint8_t(buf, 8, confirmation);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_SHORT, buf, MACE_MSG_ID_COMMAND_SHORT_MIN_LEN, MACE_MSG_ID_COMMAND_SHORT_LEN, MACE_MSG_ID_COMMAND_SHORT_CRC);
#else
    mace_command_short_t *packet = (mace_command_short_t *)msgbuf;
    packet->param = param;
    packet->command = command;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->confirmation = confirmation;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_SHORT, (const char *)packet, MACE_MSG_ID_COMMAND_SHORT_MIN_LEN, MACE_MSG_ID_COMMAND_SHORT_LEN, MACE_MSG_ID_COMMAND_SHORT_CRC);
#endif
}
#endif

#endif

// MESSAGE COMMAND_SHORT UNPACKING


/**
 * @brief Get field command from command_short message
 *
 * @return Command ID, as defined by UXV_CMD enum.is was established to reduce the bandwidth required of messages not requiring as much parameterized data.
 */
static inline uint16_t mace_msg_command_short_get_command(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field target_system from command_short message
 *
 * @return System which should execute the command
 */
static inline uint8_t mace_msg_command_short_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field target_component from command_short message
 *
 * @return Component which should execute the command, 0 for all components
 */
static inline uint8_t mace_msg_command_short_get_target_component(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field confirmation from command_short message
 *
 * @return 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 */
static inline uint8_t mace_msg_command_short_get_confirmation(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field param from command_short message
 *
 * @return Parameter as defined by UXV_CMD enum.
 */
static inline float mace_msg_command_short_get_param(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  0);
}

/**
 * @brief Decode a command_short message into a struct
 *
 * @param msg The message to decode
 * @param command_short C-struct to decode the message contents into
 */
static inline void mace_msg_command_short_decode(const mace_message_t* msg, mace_command_short_t* command_short)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    command_short->param = mace_msg_command_short_get_param(msg);
    command_short->command = mace_msg_command_short_get_command(msg);
    command_short->target_system = mace_msg_command_short_get_target_system(msg);
    command_short->target_component = mace_msg_command_short_get_target_component(msg);
    command_short->confirmation = mace_msg_command_short_get_confirmation(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_COMMAND_SHORT_LEN? msg->len : MACE_MSG_ID_COMMAND_SHORT_LEN;
        memset(command_short, 0, MACE_MSG_ID_COMMAND_SHORT_LEN);
    memcpy(command_short, _MACE_PAYLOAD(msg), len);
#endif
}
