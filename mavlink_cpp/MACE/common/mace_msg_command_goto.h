#pragma once
// MESSAGE COMMAND_GOTO PACKING

#define MACE_MSG_ID_COMMAND_GOTO 35

MACEPACKED(
typedef struct __mace_command_goto_t {
 float param1; /*< Parameter 1, as defined by MAV_CMD enum.*/
 float param2; /*< Parameter 2, as defined by MAV_CMD enum.*/
 float param3; /*< Parameter 3, as defined by MAV_CMD enum.*/
 float param4; /*< Parameter 4, as defined by MAV_CMD enum.*/
 float param5; /*< Parameter 5, as defined by MAV_CMD enum.*/
 float param6; /*< Parameter 6, as defined by MAV_CMD enum.*/
 float param7; /*< Parameter 7, as defined by MAV_CMD enum.*/
 uint16_t action; /*< Command ID, as defined by MAV_CMD enum.*/
 uint8_t target_system; /*< System which should execute the command*/
 uint8_t target_component; /*< Component which should execute the command, 0 for all components*/
 uint8_t frame; /*< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h*/
}) mace_command_goto_t;

#define MACE_MSG_ID_COMMAND_GOTO_LEN 33
#define MACE_MSG_ID_COMMAND_GOTO_MIN_LEN 33
#define MACE_MSG_ID_35_LEN 33
#define MACE_MSG_ID_35_MIN_LEN 33

#define MACE_MSG_ID_COMMAND_GOTO_CRC 64
#define MACE_MSG_ID_35_CRC 64



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_COMMAND_GOTO { \
    35, \
    "COMMAND_GOTO", \
    11, \
    {  { "param1", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_command_goto_t, param1) }, \
         { "param2", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_command_goto_t, param2) }, \
         { "param3", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_command_goto_t, param3) }, \
         { "param4", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_command_goto_t, param4) }, \
         { "param5", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_command_goto_t, param5) }, \
         { "param6", NULL, MACE_TYPE_FLOAT, 0, 20, offsetof(mace_command_goto_t, param6) }, \
         { "param7", NULL, MACE_TYPE_FLOAT, 0, 24, offsetof(mace_command_goto_t, param7) }, \
         { "action", NULL, MACE_TYPE_UINT16_T, 0, 28, offsetof(mace_command_goto_t, action) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 30, offsetof(mace_command_goto_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 31, offsetof(mace_command_goto_t, target_component) }, \
         { "frame", NULL, MACE_TYPE_UINT8_T, 0, 32, offsetof(mace_command_goto_t, frame) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_COMMAND_GOTO { \
    "COMMAND_GOTO", \
    11, \
    {  { "param1", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_command_goto_t, param1) }, \
         { "param2", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_command_goto_t, param2) }, \
         { "param3", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_command_goto_t, param3) }, \
         { "param4", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_command_goto_t, param4) }, \
         { "param5", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_command_goto_t, param5) }, \
         { "param6", NULL, MACE_TYPE_FLOAT, 0, 20, offsetof(mace_command_goto_t, param6) }, \
         { "param7", NULL, MACE_TYPE_FLOAT, 0, 24, offsetof(mace_command_goto_t, param7) }, \
         { "action", NULL, MACE_TYPE_UINT16_T, 0, 28, offsetof(mace_command_goto_t, action) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 30, offsetof(mace_command_goto_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 31, offsetof(mace_command_goto_t, target_component) }, \
         { "frame", NULL, MACE_TYPE_UINT8_T, 0, 32, offsetof(mace_command_goto_t, frame) }, \
         } \
}
#endif

/**
 * @brief Pack a command_goto message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param action Command ID, as defined by MAV_CMD enum.
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_command_goto_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t action, uint8_t frame, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_GOTO_LEN];
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_float(buf, 16, param5);
    _mace_put_float(buf, 20, param6);
    _mace_put_float(buf, 24, param7);
    _mace_put_uint16_t(buf, 28, action);
    _mace_put_uint8_t(buf, 30, target_system);
    _mace_put_uint8_t(buf, 31, target_component);
    _mace_put_uint8_t(buf, 32, frame);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_COMMAND_GOTO_LEN);
#else
    mace_command_goto_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.action = action;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_COMMAND_GOTO_LEN);
#endif

    msg->msgid = MACE_MSG_ID_COMMAND_GOTO;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_COMMAND_GOTO_MIN_LEN, MACE_MSG_ID_COMMAND_GOTO_LEN, MACE_MSG_ID_COMMAND_GOTO_CRC);
}

/**
 * @brief Pack a command_goto message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param action Command ID, as defined by MAV_CMD enum.
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_command_goto_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t action,uint8_t frame,float param1,float param2,float param3,float param4,float param5,float param6,float param7)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_GOTO_LEN];
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_float(buf, 16, param5);
    _mace_put_float(buf, 20, param6);
    _mace_put_float(buf, 24, param7);
    _mace_put_uint16_t(buf, 28, action);
    _mace_put_uint8_t(buf, 30, target_system);
    _mace_put_uint8_t(buf, 31, target_component);
    _mace_put_uint8_t(buf, 32, frame);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_COMMAND_GOTO_LEN);
#else
    mace_command_goto_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.action = action;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_COMMAND_GOTO_LEN);
#endif

    msg->msgid = MACE_MSG_ID_COMMAND_GOTO;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_COMMAND_GOTO_MIN_LEN, MACE_MSG_ID_COMMAND_GOTO_LEN, MACE_MSG_ID_COMMAND_GOTO_CRC);
}

/**
 * @brief Encode a command_goto struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_goto C-struct to read the message contents from
 */
static inline uint16_t mace_msg_command_goto_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_command_goto_t* command_goto)
{
    return mace_msg_command_goto_pack(system_id, component_id, msg, command_goto->target_system, command_goto->target_component, command_goto->action, command_goto->frame, command_goto->param1, command_goto->param2, command_goto->param3, command_goto->param4, command_goto->param5, command_goto->param6, command_goto->param7);
}

/**
 * @brief Encode a command_goto struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_goto C-struct to read the message contents from
 */
static inline uint16_t mace_msg_command_goto_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_command_goto_t* command_goto)
{
    return mace_msg_command_goto_pack_chan(system_id, component_id, chan, msg, command_goto->target_system, command_goto->target_component, command_goto->action, command_goto->frame, command_goto->param1, command_goto->param2, command_goto->param3, command_goto->param4, command_goto->param5, command_goto->param6, command_goto->param7);
}

/**
 * @brief Send a command_goto message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param action Command ID, as defined by MAV_CMD enum.
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_command_goto_send(mace_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t action, uint8_t frame, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_GOTO_LEN];
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_float(buf, 16, param5);
    _mace_put_float(buf, 20, param6);
    _mace_put_float(buf, 24, param7);
    _mace_put_uint16_t(buf, 28, action);
    _mace_put_uint8_t(buf, 30, target_system);
    _mace_put_uint8_t(buf, 31, target_component);
    _mace_put_uint8_t(buf, 32, frame);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_GOTO, buf, MACE_MSG_ID_COMMAND_GOTO_MIN_LEN, MACE_MSG_ID_COMMAND_GOTO_LEN, MACE_MSG_ID_COMMAND_GOTO_CRC);
#else
    mace_command_goto_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.action = action;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_GOTO, (const char *)&packet, MACE_MSG_ID_COMMAND_GOTO_MIN_LEN, MACE_MSG_ID_COMMAND_GOTO_LEN, MACE_MSG_ID_COMMAND_GOTO_CRC);
#endif
}

/**
 * @brief Send a command_goto message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_command_goto_send_struct(mace_channel_t chan, const mace_command_goto_t* command_goto)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_command_goto_send(chan, command_goto->target_system, command_goto->target_component, command_goto->action, command_goto->frame, command_goto->param1, command_goto->param2, command_goto->param3, command_goto->param4, command_goto->param5, command_goto->param6, command_goto->param7);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_GOTO, (const char *)command_goto, MACE_MSG_ID_COMMAND_GOTO_MIN_LEN, MACE_MSG_ID_COMMAND_GOTO_LEN, MACE_MSG_ID_COMMAND_GOTO_CRC);
#endif
}

#if MACE_MSG_ID_COMMAND_GOTO_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_command_goto_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t action, uint8_t frame, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
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
    _mace_put_uint16_t(buf, 28, action);
    _mace_put_uint8_t(buf, 30, target_system);
    _mace_put_uint8_t(buf, 31, target_component);
    _mace_put_uint8_t(buf, 32, frame);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_GOTO, buf, MACE_MSG_ID_COMMAND_GOTO_MIN_LEN, MACE_MSG_ID_COMMAND_GOTO_LEN, MACE_MSG_ID_COMMAND_GOTO_CRC);
#else
    mace_command_goto_t *packet = (mace_command_goto_t *)msgbuf;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->param5 = param5;
    packet->param6 = param6;
    packet->param7 = param7;
    packet->action = action;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->frame = frame;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_GOTO, (const char *)packet, MACE_MSG_ID_COMMAND_GOTO_MIN_LEN, MACE_MSG_ID_COMMAND_GOTO_LEN, MACE_MSG_ID_COMMAND_GOTO_CRC);
#endif
}
#endif

#endif

// MESSAGE COMMAND_GOTO UNPACKING


/**
 * @brief Get field target_system from command_goto message
 *
 * @return System which should execute the command
 */
static inline uint8_t mace_msg_command_goto_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field target_component from command_goto message
 *
 * @return Component which should execute the command, 0 for all components
 */
static inline uint8_t mace_msg_command_goto_get_target_component(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field action from command_goto message
 *
 * @return Command ID, as defined by MAV_CMD enum.
 */
static inline uint16_t mace_msg_command_goto_get_action(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field frame from command_goto message
 *
 * @return The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 */
static inline uint8_t mace_msg_command_goto_get_frame(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field param1 from command_goto message
 *
 * @return Parameter 1, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_goto_get_param1(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  0);
}

/**
 * @brief Get field param2 from command_goto message
 *
 * @return Parameter 2, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_goto_get_param2(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field param3 from command_goto message
 *
 * @return Parameter 3, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_goto_get_param3(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field param4 from command_goto message
 *
 * @return Parameter 4, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_goto_get_param4(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Get field param5 from command_goto message
 *
 * @return Parameter 5, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_goto_get_param5(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  16);
}

/**
 * @brief Get field param6 from command_goto message
 *
 * @return Parameter 6, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_goto_get_param6(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  20);
}

/**
 * @brief Get field param7 from command_goto message
 *
 * @return Parameter 7, as defined by MAV_CMD enum.
 */
static inline float mace_msg_command_goto_get_param7(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  24);
}

/**
 * @brief Decode a command_goto message into a struct
 *
 * @param msg The message to decode
 * @param command_goto C-struct to decode the message contents into
 */
static inline void mace_msg_command_goto_decode(const mace_message_t* msg, mace_command_goto_t* command_goto)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    command_goto->param1 = mace_msg_command_goto_get_param1(msg);
    command_goto->param2 = mace_msg_command_goto_get_param2(msg);
    command_goto->param3 = mace_msg_command_goto_get_param3(msg);
    command_goto->param4 = mace_msg_command_goto_get_param4(msg);
    command_goto->param5 = mace_msg_command_goto_get_param5(msg);
    command_goto->param6 = mace_msg_command_goto_get_param6(msg);
    command_goto->param7 = mace_msg_command_goto_get_param7(msg);
    command_goto->action = mace_msg_command_goto_get_action(msg);
    command_goto->target_system = mace_msg_command_goto_get_target_system(msg);
    command_goto->target_component = mace_msg_command_goto_get_target_component(msg);
    command_goto->frame = mace_msg_command_goto_get_frame(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_COMMAND_GOTO_LEN? msg->len : MACE_MSG_ID_COMMAND_GOTO_LEN;
        memset(command_goto, 0, MACE_MSG_ID_COMMAND_GOTO_LEN);
    memcpy(command_goto, _MACE_PAYLOAD(msg), len);
#endif
}
