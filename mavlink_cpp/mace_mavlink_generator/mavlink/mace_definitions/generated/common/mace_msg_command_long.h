#pragma once
// MESSAGE COMMAND_LONG PACKING

#define MACE_MSG_ID_COMMAND_LONG 30

MACEPACKED(
typedef struct __mace_command_long_t {
 float param1; /*< Parameter 1, as defined by MAV_CMD enum.*/
 float param2; /*< Parameter 2, as defined by MAV_CMD enum.*/
 float param3; /*< Parameter 3, as defined by MAV_CMD enum.*/
 float param4; /*< Parameter 4, as defined by MAV_CMD enum.*/
 float param5; /*< Parameter 5, as defined by MAV_CMD enum.*/
 float param6; /*< Parameter 6, as defined by MAV_CMD enum.*/
 float param7; /*< Parameter 7, as defined by MAV_CMD enum.*/
 uint16_t command; /*< Command ID, as defined by MAV_CMD enum.*/
 uint8_t target_system; /*< System which should execute the command*/
 uint8_t target_component; /*< Component which should execute the command, 0 for all components*/
 uint8_t confirmation; /*< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)*/
}) mace_command_long_t;

#define MACE_MSG_ID_COMMAND_LONG_LEN 33
#define MACE_MSG_ID_COMMAND_LONG_MIN_LEN 33
#define MACE_MSG_ID_30_LEN 33
#define MACE_MSG_ID_30_MIN_LEN 33

#define MACE_MSG_ID_COMMAND_LONG_CRC 152
#define MACE_MSG_ID_30_CRC 152



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_COMMAND_LONG { \
    30, \
    "COMMAND_LONG", \
    11, \
    {  { "param1", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_command_long_t, param1) }, \
         { "param2", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_command_long_t, param2) }, \
         { "param3", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_command_long_t, param3) }, \
         { "param4", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_command_long_t, param4) }, \
         { "param5", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_command_long_t, param5) }, \
         { "param6", NULL, MACE_TYPE_FLOAT, 0, 20, offsetof(mace_command_long_t, param6) }, \
         { "param7", NULL, MACE_TYPE_FLOAT, 0, 24, offsetof(mace_command_long_t, param7) }, \
         { "command", NULL, MACE_TYPE_UINT16_T, 0, 28, offsetof(mace_command_long_t, command) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 30, offsetof(mace_command_long_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 31, offsetof(mace_command_long_t, target_component) }, \
         { "confirmation", NULL, MACE_TYPE_UINT8_T, 0, 32, offsetof(mace_command_long_t, confirmation) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_COMMAND_LONG { \
    "COMMAND_LONG", \
    11, \
    {  { "param1", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_command_long_t, param1) }, \
         { "param2", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_command_long_t, param2) }, \
         { "param3", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_command_long_t, param3) }, \
         { "param4", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_command_long_t, param4) }, \
         { "param5", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_command_long_t, param5) }, \
         { "param6", NULL, MACE_TYPE_FLOAT, 0, 20, offsetof(mace_command_long_t, param6) }, \
         { "param7", NULL, MACE_TYPE_FLOAT, 0, 24, offsetof(mace_command_long_t, param7) }, \
         { "command", NULL, MACE_TYPE_UINT16_T, 0, 28, offsetof(mace_command_long_t, command) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 30, offsetof(mace_command_long_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 31, offsetof(mace_command_long_t, target_component) }, \
         { "confirmation", NULL, MACE_TYPE_UINT8_T, 0, 32, offsetof(mace_command_long_t, confirmation) }, \
         } \
}
#endif

/**
 * @brief Pack a command_long message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_LONG_LEN];
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_float(buf, 16, param5);
    _mace_put_float(buf, 20, param6);
    _mace_put_float(buf, 24, param7);
    _mace_put_uint16_t(buf, 28, command);
    _mace_put_uint8_t(buf, 30, target_system);
    _mace_put_uint8_t(buf, 31, target_component);
    _mace_put_uint8_t(buf, 32, confirmation);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_COMMAND_LONG_LEN);
#else
    mace_command_long_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.confirmation = confirmation;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_COMMAND_LONG_LEN);
#endif

    msg->msgid = MACE_MSG_ID_COMMAND_LONG;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_COMMAND_LONG_MIN_LEN, MACE_MSG_ID_COMMAND_LONG_LEN, MACE_MSG_ID_COMMAND_LONG_CRC);
}

/**
 * @brief Pack a command_long message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_command_long_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t command,uint8_t confirmation,float param1,float param2,float param3,float param4,float param5,float param6,float param7)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_LONG_LEN];
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_float(buf, 16, param5);
    _mace_put_float(buf, 20, param6);
    _mace_put_float(buf, 24, param7);
    _mace_put_uint16_t(buf, 28, command);
    _mace_put_uint8_t(buf, 30, target_system);
    _mace_put_uint8_t(buf, 31, target_component);
    _mace_put_uint8_t(buf, 32, confirmation);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_COMMAND_LONG_LEN);
#else
    mace_command_long_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.confirmation = confirmation;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_COMMAND_LONG_LEN);
#endif

    msg->msgid = MACE_MSG_ID_COMMAND_LONG;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_COMMAND_LONG_MIN_LEN, MACE_MSG_ID_COMMAND_LONG_LEN, MACE_MSG_ID_COMMAND_LONG_CRC);
}

/**
 * @brief Encode a command_long struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_long C-struct to read the message contents from
 */
static inline uint16_t mace_msg_command_long_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_command_long_t* command_long)
{
    return mace_msg_command_long_pack(system_id, component_id, msg, command_long->target_system, command_long->target_component, command_long->command, command_long->confirmation, command_long->param1, command_long->param2, command_long->param3, command_long->param4, command_long->param5, command_long->param6, command_long->param7);
}

/**
 * @brief Encode a command_long struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_long C-struct to read the message contents from
 */
static inline uint16_t mace_msg_command_long_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_command_long_t* command_long)
{
    return mace_msg_command_long_pack_chan(system_id, component_id, chan, msg, command_long->target_system, command_long->target_component, command_long->command, command_long->confirmation, command_long->param1, command_long->param2, command_long->param3, command_long->param4, command_long->param5, command_long->param6, command_long->param7);
}

/**
 * @brief Send a command_long message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_command_long_send(mace_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_LONG_LEN];
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_float(buf, 16, param5);
    _mace_put_float(buf, 20, param6);
    _mace_put_float(buf, 24, param7);
    _mace_put_uint16_t(buf, 28, command);
    _mace_put_uint8_t(buf, 30, target_system);
    _mace_put_uint8_t(buf, 31, target_component);
    _mace_put_uint8_t(buf, 32, confirmation);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_LONG, buf, MACE_MSG_ID_COMMAND_LONG_MIN_LEN, MACE_MSG_ID_COMMAND_LONG_LEN, MACE_MSG_ID_COMMAND_LONG_CRC);
#else
    mace_command_long_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.confirmation = confirmation;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_LONG, (const char *)&packet, MACE_MSG_ID_COMMAND_LONG_MIN_LEN, MACE_MSG_ID_COMMAND_LONG_LEN, MACE_MSG_ID_COMMAND_LONG_CRC);
#endif
}

/**
 * @brief Send a command_long message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_command_long_send_struct(mace_channel_t chan, const mace_command_long_t* command_long)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_command_long_send(chan, command_long->target_system, command_long->target_component, command_long->command, command_long->confirmation, command_long->param1, command_long->param2, command_long->param3, command_long->param4, command_long->param5, command_long->param6, command_long->param7);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_LONG, (const char *)command_long, MACE_MSG_ID_COMMAND_LONG_MIN_LEN, MACE_MSG_ID_COMMAND_LONG_LEN, MACE_MSG_ID_COMMAND_LONG_CRC);
#endif
}

#if MACE_MSG_ID_COMMAND_LONG_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_command_long_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_float(buf, 16, param5);
    _mace_put_float(buf, 20, param6);
    _mace_put_float(buf, 24, param7);
    _mace_put_uint16_t(buf, 28, command);
    _mace_put_uint8_t(buf, 30, target_system);
    _mace_put_uint8_t(buf, 31, target_component);
    _mace_put_uint8_t(buf, 32, confirmation);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_LONG, buf, MACE_MSG_ID_COMMAND_LONG_MIN_LEN, MACE_MSG_ID_COMMAND_LONG_LEN, MACE_MSG_ID_COMMAND_LONG_CRC);
#else
    mace_command_long_t *packet = (mace_command_long_t *)msgbuf;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->param5 = param5;
    packet->param6 = param6;
    packet->param7 = param7;
    packet->command = command;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->confirmation = confirmation;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_LONG, (const char *)packet, MACE_MSG_ID_COMMAND_LONG_MIN_LEN, MACE_MSG_ID_COMMAND_LONG_LEN, MACE_MSG_ID_COMMAND_LONG_CRC);
#endif
}
#endif

#endif

// MESSAGE COMMAND_LONG UNPACKING


/**
 * @brief Get field target_system from command_long message
 *
 * @return System which should execute the command
 */
static inline uint8_t mace_msg_command_long_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field target_component from command_long message
 *
 * @return Component which should execute the command, 0 for all components
 */
static inline uint8_t mace_msg_command_long_get_target_component(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field command from command_long message
 *
 * @return Command ID, as defined by MAV_CMD enum.
 */
static inline uint16_t mace_msg_command_long_get_command(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field confirmation from command_long message
 *
 * @return 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 */
static inline uint8_t mace_msg_command_long_get_confirmation(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field param1 from command_long message
 *
 * @return Parameter 1, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_long_get_param1(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  0);
}

/**
 * @brief Get field param2 from command_long message
 *
 * @return Parameter 2, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_long_get_param2(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field param3 from command_long message
 *
 * @return Parameter 3, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_long_get_param3(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field param4 from command_long message
 *
 * @return Parameter 4, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_long_get_param4(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Get field param5 from command_long message
 *
 * @return Parameter 5, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_long_get_param5(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  16);
}

/**
 * @brief Get field param6 from command_long message
 *
 * @return Parameter 6, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_long_get_param6(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  20);
}

/**
 * @brief Get field param7 from command_long message
 *
 * @return Parameter 7, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_long_get_param7(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  24);
}

/**
 * @brief Decode a command_long message into a struct
 *
 * @param msg The message to decode
 * @param command_long C-struct to decode the message contents into
 */
static inline void mace_msg_command_long_decode(const mace_message_t* msg, mace_command_long_t* command_long)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    command_long->param1 = mace_msg_command_long_get_param1(msg);
    command_long->param2 = mace_msg_command_long_get_param2(msg);
    command_long->param3 = mace_msg_command_long_get_param3(msg);
    command_long->param4 = mace_msg_command_long_get_param4(msg);
    command_long->param5 = mace_msg_command_long_get_param5(msg);
    command_long->param6 = mace_msg_command_long_get_param6(msg);
    command_long->param7 = mace_msg_command_long_get_param7(msg);
    command_long->command = mace_msg_command_long_get_command(msg);
    command_long->target_system = mace_msg_command_long_get_target_system(msg);
    command_long->target_component = mace_msg_command_long_get_target_component(msg);
    command_long->confirmation = mace_msg_command_long_get_confirmation(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_COMMAND_LONG_LEN? msg->len : MACE_MSG_ID_COMMAND_LONG_LEN;
        memset(command_long, 0, MACE_MSG_ID_COMMAND_LONG_LEN);
    memcpy(command_long, _MACE_PAYLOAD(msg), len);
#endif
}
