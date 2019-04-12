#pragma once
// MESSAGE COMMAND_INT PACKING

#define MACE_MSG_ID_COMMAND_INT 29

MACEPACKED(
typedef struct __mace_command_int_t {
 float param1; /*< PARAM1, see MAV_CMD enum*/
 float param2; /*< PARAM2, see MAV_CMD enum*/
 float param3; /*< PARAM3, see MAV_CMD enum*/
 float param4; /*< PARAM4, see MAV_CMD enum*/
 int32_t x; /*< PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7*/
 int32_t y; /*< PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7*/
 float z; /*< PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.*/
 uint16_t command; /*< The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t frame; /*< The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h*/
 uint8_t current; /*< false:0, true:1*/
 uint8_t autocontinue; /*< autocontinue to next wp*/
}) mace_command_int_t;

#define MACE_MSG_ID_COMMAND_INT_LEN 35
#define MACE_MSG_ID_COMMAND_INT_MIN_LEN 35
#define MACE_MSG_ID_29_LEN 35
#define MACE_MSG_ID_29_MIN_LEN 35

#define MACE_MSG_ID_COMMAND_INT_CRC 158
#define MACE_MSG_ID_29_CRC 158



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_COMMAND_INT { \
    29, \
    "COMMAND_INT", \
    13, \
    {  { "param1", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_command_int_t, param1) }, \
         { "param2", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_command_int_t, param2) }, \
         { "param3", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_command_int_t, param3) }, \
         { "param4", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_command_int_t, param4) }, \
         { "x", NULL, MACE_TYPE_INT32_T, 0, 16, offsetof(mace_command_int_t, x) }, \
         { "y", NULL, MACE_TYPE_INT32_T, 0, 20, offsetof(mace_command_int_t, y) }, \
         { "z", NULL, MACE_TYPE_FLOAT, 0, 24, offsetof(mace_command_int_t, z) }, \
         { "command", NULL, MACE_TYPE_UINT16_T, 0, 28, offsetof(mace_command_int_t, command) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 30, offsetof(mace_command_int_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 31, offsetof(mace_command_int_t, target_component) }, \
         { "frame", NULL, MACE_TYPE_UINT8_T, 0, 32, offsetof(mace_command_int_t, frame) }, \
         { "current", NULL, MACE_TYPE_UINT8_T, 0, 33, offsetof(mace_command_int_t, current) }, \
         { "autocontinue", NULL, MACE_TYPE_UINT8_T, 0, 34, offsetof(mace_command_int_t, autocontinue) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_COMMAND_INT { \
    "COMMAND_INT", \
    13, \
    {  { "param1", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_command_int_t, param1) }, \
         { "param2", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_command_int_t, param2) }, \
         { "param3", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_command_int_t, param3) }, \
         { "param4", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_command_int_t, param4) }, \
         { "x", NULL, MACE_TYPE_INT32_T, 0, 16, offsetof(mace_command_int_t, x) }, \
         { "y", NULL, MACE_TYPE_INT32_T, 0, 20, offsetof(mace_command_int_t, y) }, \
         { "z", NULL, MACE_TYPE_FLOAT, 0, 24, offsetof(mace_command_int_t, z) }, \
         { "command", NULL, MACE_TYPE_UINT16_T, 0, 28, offsetof(mace_command_int_t, command) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 30, offsetof(mace_command_int_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 31, offsetof(mace_command_int_t, target_component) }, \
         { "frame", NULL, MACE_TYPE_UINT8_T, 0, 32, offsetof(mace_command_int_t, frame) }, \
         { "current", NULL, MACE_TYPE_UINT8_T, 0, 33, offsetof(mace_command_int_t, current) }, \
         { "autocontinue", NULL, MACE_TYPE_UINT8_T, 0, 34, offsetof(mace_command_int_t, autocontinue) }, \
         } \
}
#endif

/**
 * @brief Pack a command_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param frame The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1, see MAV_CMD enum
 * @param param2 PARAM2, see MAV_CMD enum
 * @param param3 PARAM3, see MAV_CMD enum
 * @param param4 PARAM4, see MAV_CMD enum
 * @param x PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
 * @param y PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
 * @param z PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_command_int_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_INT_LEN];
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_int32_t(buf, 16, x);
    _mace_put_int32_t(buf, 20, y);
    _mace_put_float(buf, 24, z);
    _mace_put_uint16_t(buf, 28, command);
    _mace_put_uint8_t(buf, 30, target_system);
    _mace_put_uint8_t(buf, 31, target_component);
    _mace_put_uint8_t(buf, 32, frame);
    _mace_put_uint8_t(buf, 33, current);
    _mace_put_uint8_t(buf, 34, autocontinue);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_COMMAND_INT_LEN);
#else
    mace_command_int_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;
    packet.current = current;
    packet.autocontinue = autocontinue;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_COMMAND_INT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_COMMAND_INT;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_COMMAND_INT_MIN_LEN, MACE_MSG_ID_COMMAND_INT_LEN, MACE_MSG_ID_COMMAND_INT_CRC);
}

/**
 * @brief Pack a command_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param frame The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1, see MAV_CMD enum
 * @param param2 PARAM2, see MAV_CMD enum
 * @param param3 PARAM3, see MAV_CMD enum
 * @param param4 PARAM4, see MAV_CMD enum
 * @param x PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
 * @param y PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
 * @param z PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_command_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t frame,uint16_t command,uint8_t current,uint8_t autocontinue,float param1,float param2,float param3,float param4,int32_t x,int32_t y,float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_INT_LEN];
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_int32_t(buf, 16, x);
    _mace_put_int32_t(buf, 20, y);
    _mace_put_float(buf, 24, z);
    _mace_put_uint16_t(buf, 28, command);
    _mace_put_uint8_t(buf, 30, target_system);
    _mace_put_uint8_t(buf, 31, target_component);
    _mace_put_uint8_t(buf, 32, frame);
    _mace_put_uint8_t(buf, 33, current);
    _mace_put_uint8_t(buf, 34, autocontinue);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_COMMAND_INT_LEN);
#else
    mace_command_int_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;
    packet.current = current;
    packet.autocontinue = autocontinue;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_COMMAND_INT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_COMMAND_INT;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_COMMAND_INT_MIN_LEN, MACE_MSG_ID_COMMAND_INT_LEN, MACE_MSG_ID_COMMAND_INT_CRC);
}

/**
 * @brief Encode a command_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_int C-struct to read the message contents from
 */
static inline uint16_t mace_msg_command_int_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_command_int_t* command_int)
{
    return mace_msg_command_int_pack(system_id, component_id, msg, command_int->target_system, command_int->target_component, command_int->frame, command_int->command, command_int->current, command_int->autocontinue, command_int->param1, command_int->param2, command_int->param3, command_int->param4, command_int->x, command_int->y, command_int->z);
}

/**
 * @brief Encode a command_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_int C-struct to read the message contents from
 */
static inline uint16_t mace_msg_command_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_command_int_t* command_int)
{
    return mace_msg_command_int_pack_chan(system_id, component_id, chan, msg, command_int->target_system, command_int->target_component, command_int->frame, command_int->command, command_int->current, command_int->autocontinue, command_int->param1, command_int->param2, command_int->param3, command_int->param4, command_int->x, command_int->y, command_int->z);
}

/**
 * @brief Send a command_int message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param frame The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1, see MAV_CMD enum
 * @param param2 PARAM2, see MAV_CMD enum
 * @param param3 PARAM3, see MAV_CMD enum
 * @param param4 PARAM4, see MAV_CMD enum
 * @param x PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
 * @param y PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
 * @param z PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_command_int_send(mace_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COMMAND_INT_LEN];
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_int32_t(buf, 16, x);
    _mace_put_int32_t(buf, 20, y);
    _mace_put_float(buf, 24, z);
    _mace_put_uint16_t(buf, 28, command);
    _mace_put_uint8_t(buf, 30, target_system);
    _mace_put_uint8_t(buf, 31, target_component);
    _mace_put_uint8_t(buf, 32, frame);
    _mace_put_uint8_t(buf, 33, current);
    _mace_put_uint8_t(buf, 34, autocontinue);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_INT, buf, MACE_MSG_ID_COMMAND_INT_MIN_LEN, MACE_MSG_ID_COMMAND_INT_LEN, MACE_MSG_ID_COMMAND_INT_CRC);
#else
    mace_command_int_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;
    packet.current = current;
    packet.autocontinue = autocontinue;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_INT, (const char *)&packet, MACE_MSG_ID_COMMAND_INT_MIN_LEN, MACE_MSG_ID_COMMAND_INT_LEN, MACE_MSG_ID_COMMAND_INT_CRC);
#endif
}

/**
 * @brief Send a command_int message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_command_int_send_struct(mace_channel_t chan, const mace_command_int_t* command_int)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_command_int_send(chan, command_int->target_system, command_int->target_component, command_int->frame, command_int->command, command_int->current, command_int->autocontinue, command_int->param1, command_int->param2, command_int->param3, command_int->param4, command_int->x, command_int->y, command_int->z);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_INT, (const char *)command_int, MACE_MSG_ID_COMMAND_INT_MIN_LEN, MACE_MSG_ID_COMMAND_INT_LEN, MACE_MSG_ID_COMMAND_INT_CRC);
#endif
}

#if MACE_MSG_ID_COMMAND_INT_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_command_int_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_int32_t(buf, 16, x);
    _mace_put_int32_t(buf, 20, y);
    _mace_put_float(buf, 24, z);
    _mace_put_uint16_t(buf, 28, command);
    _mace_put_uint8_t(buf, 30, target_system);
    _mace_put_uint8_t(buf, 31, target_component);
    _mace_put_uint8_t(buf, 32, frame);
    _mace_put_uint8_t(buf, 33, current);
    _mace_put_uint8_t(buf, 34, autocontinue);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_INT, buf, MACE_MSG_ID_COMMAND_INT_MIN_LEN, MACE_MSG_ID_COMMAND_INT_LEN, MACE_MSG_ID_COMMAND_INT_CRC);
#else
    mace_command_int_t *packet = (mace_command_int_t *)msgbuf;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->command = command;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->frame = frame;
    packet->current = current;
    packet->autocontinue = autocontinue;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COMMAND_INT, (const char *)packet, MACE_MSG_ID_COMMAND_INT_MIN_LEN, MACE_MSG_ID_COMMAND_INT_LEN, MACE_MSG_ID_COMMAND_INT_CRC);
#endif
}
#endif

#endif

// MESSAGE COMMAND_INT UNPACKING


/**
 * @brief Get field target_system from command_int message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_command_int_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field target_component from command_int message
 *
 * @return Component ID
 */
static inline uint8_t mace_msg_command_int_get_target_component(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field frame from command_int message
 *
 * @return The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
 */
static inline uint8_t mace_msg_command_int_get_frame(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field command from command_int message
 *
 * @return The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
 */
static inline uint16_t mace_msg_command_int_get_command(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field current from command_int message
 *
 * @return false:0, true:1
 */
static inline uint8_t mace_msg_command_int_get_current(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field autocontinue from command_int message
 *
 * @return autocontinue to next wp
 */
static inline uint8_t mace_msg_command_int_get_autocontinue(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field param1 from command_int message
 *
 * @return PARAM1, see MAV_CMD enum
 */
static inline float mace_msg_command_int_get_param1(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  0);
}

/**
 * @brief Get field param2 from command_int message
 *
 * @return PARAM2, see MAV_CMD enum
 */
static inline float mace_msg_command_int_get_param2(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field param3 from command_int message
 *
 * @return PARAM3, see MAV_CMD enum
 */
static inline float mace_msg_command_int_get_param3(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field param4 from command_int message
 *
 * @return PARAM4, see MAV_CMD enum
 */
static inline float mace_msg_command_int_get_param4(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Get field x from command_int message
 *
 * @return PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
 */
static inline int32_t mace_msg_command_int_get_x(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field y from command_int message
 *
 * @return PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
 */
static inline int32_t mace_msg_command_int_get_y(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field z from command_int message
 *
 * @return PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
 */
static inline float mace_msg_command_int_get_z(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  24);
}

/**
 * @brief Decode a command_int message into a struct
 *
 * @param msg The message to decode
 * @param command_int C-struct to decode the message contents into
 */
static inline void mace_msg_command_int_decode(const mace_message_t* msg, mace_command_int_t* command_int)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    command_int->param1 = mace_msg_command_int_get_param1(msg);
    command_int->param2 = mace_msg_command_int_get_param2(msg);
    command_int->param3 = mace_msg_command_int_get_param3(msg);
    command_int->param4 = mace_msg_command_int_get_param4(msg);
    command_int->x = mace_msg_command_int_get_x(msg);
    command_int->y = mace_msg_command_int_get_y(msg);
    command_int->z = mace_msg_command_int_get_z(msg);
    command_int->command = mace_msg_command_int_get_command(msg);
    command_int->target_system = mace_msg_command_int_get_target_system(msg);
    command_int->target_component = mace_msg_command_int_get_target_component(msg);
    command_int->frame = mace_msg_command_int_get_frame(msg);
    command_int->current = mace_msg_command_int_get_current(msg);
    command_int->autocontinue = mace_msg_command_int_get_autocontinue(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_COMMAND_INT_LEN? msg->len : MACE_MSG_ID_COMMAND_INT_LEN;
        memset(command_int, 0, MACE_MSG_ID_COMMAND_INT_LEN);
    memcpy(command_int, _MACE_PAYLOAD(msg), len);
#endif
}
