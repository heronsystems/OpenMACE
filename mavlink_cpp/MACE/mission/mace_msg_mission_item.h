#pragma once
// MESSAGE MISSION_ITEM PACKING

#define MACE_MSG_ID_MISSION_ITEM 107

MACEPACKED(
typedef struct __mace_mission_item_t {
 float param1; /*< PARAM1, see MAV_CMD enum*/
 float param2; /*< PARAM2, see MAV_CMD enum*/
 float param3; /*< PARAM3, see MAV_CMD enum*/
 float param4; /*< PARAM4, see MAV_CMD enum*/
 float x; /*< PARAM5 / local: x position, global: latitude*/
 float y; /*< PARAM6 / y position: global: longitude*/
 float z; /*< PARAM7 / z position: global: altitude (relative or absolute, depending on frame.*/
 uint16_t seq; /*< Sequence*/
 uint16_t command; /*< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs*/
 uint8_t target_system; /*< Target System ID*/
 uint8_t mission_system; /*< Mission System ID*/
 uint8_t mission_creator; /*< Creator ID*/
 uint8_t mission_id; /*< Mission ID*/
 uint8_t mission_type; /*< Mission type, see MISSION_TYPE*/
 uint8_t mission_state; /*< The mission state, see MISSION_STATE*/
 uint8_t frame; /*< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h*/
 uint8_t current; /*< false:0, true:1*/
 uint8_t autocontinue; /*< autocontinue to next wp*/
}) mace_mission_item_t;

#define MACE_MSG_ID_MISSION_ITEM_LEN 41
#define MACE_MSG_ID_MISSION_ITEM_MIN_LEN 41
#define MACE_MSG_ID_107_LEN 41
#define MACE_MSG_ID_107_MIN_LEN 41

#define MACE_MSG_ID_MISSION_ITEM_CRC 49
#define MACE_MSG_ID_107_CRC 49



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_MISSION_ITEM { \
    107, \
    "MISSION_ITEM", \
    18, \
    {  { "param1", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_mission_item_t, param1) }, \
         { "param2", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_mission_item_t, param2) }, \
         { "param3", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_mission_item_t, param3) }, \
         { "param4", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_mission_item_t, param4) }, \
         { "x", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_mission_item_t, x) }, \
         { "y", NULL, MACE_TYPE_FLOAT, 0, 20, offsetof(mace_mission_item_t, y) }, \
         { "z", NULL, MACE_TYPE_FLOAT, 0, 24, offsetof(mace_mission_item_t, z) }, \
         { "seq", NULL, MACE_TYPE_UINT16_T, 0, 28, offsetof(mace_mission_item_t, seq) }, \
         { "command", NULL, MACE_TYPE_UINT16_T, 0, 30, offsetof(mace_mission_item_t, command) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 32, offsetof(mace_mission_item_t, target_system) }, \
         { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 33, offsetof(mace_mission_item_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 34, offsetof(mace_mission_item_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 35, offsetof(mace_mission_item_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 36, offsetof(mace_mission_item_t, mission_type) }, \
         { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 37, offsetof(mace_mission_item_t, mission_state) }, \
         { "frame", NULL, MACE_TYPE_UINT8_T, 0, 38, offsetof(mace_mission_item_t, frame) }, \
         { "current", NULL, MACE_TYPE_UINT8_T, 0, 39, offsetof(mace_mission_item_t, current) }, \
         { "autocontinue", NULL, MACE_TYPE_UINT8_T, 0, 40, offsetof(mace_mission_item_t, autocontinue) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_MISSION_ITEM { \
    "MISSION_ITEM", \
    18, \
    {  { "param1", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_mission_item_t, param1) }, \
         { "param2", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_mission_item_t, param2) }, \
         { "param3", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_mission_item_t, param3) }, \
         { "param4", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_mission_item_t, param4) }, \
         { "x", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_mission_item_t, x) }, \
         { "y", NULL, MACE_TYPE_FLOAT, 0, 20, offsetof(mace_mission_item_t, y) }, \
         { "z", NULL, MACE_TYPE_FLOAT, 0, 24, offsetof(mace_mission_item_t, z) }, \
         { "seq", NULL, MACE_TYPE_UINT16_T, 0, 28, offsetof(mace_mission_item_t, seq) }, \
         { "command", NULL, MACE_TYPE_UINT16_T, 0, 30, offsetof(mace_mission_item_t, command) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 32, offsetof(mace_mission_item_t, target_system) }, \
         { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 33, offsetof(mace_mission_item_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 34, offsetof(mace_mission_item_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 35, offsetof(mace_mission_item_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 36, offsetof(mace_mission_item_t, mission_type) }, \
         { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 37, offsetof(mace_mission_item_t, mission_state) }, \
         { "frame", NULL, MACE_TYPE_UINT8_T, 0, 38, offsetof(mace_mission_item_t, frame) }, \
         { "current", NULL, MACE_TYPE_UINT8_T, 0, 39, offsetof(mace_mission_item_t, current) }, \
         { "autocontinue", NULL, MACE_TYPE_UINT8_T, 0, 40, offsetof(mace_mission_item_t, autocontinue) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_item message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system Target System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state The mission state, see MISSION_STATE
 * @param seq Sequence
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1, see MAV_CMD enum
 * @param param2 PARAM2, see MAV_CMD enum
 * @param param3 PARAM3, see MAV_CMD enum
 * @param param4 PARAM4, see MAV_CMD enum
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_ITEM_LEN];
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_float(buf, 16, x);
    _mace_put_float(buf, 20, y);
    _mace_put_float(buf, 24, z);
    _mace_put_uint16_t(buf, 28, seq);
    _mace_put_uint16_t(buf, 30, command);
    _mace_put_uint8_t(buf, 32, target_system);
    _mace_put_uint8_t(buf, 33, mission_system);
    _mace_put_uint8_t(buf, 34, mission_creator);
    _mace_put_uint8_t(buf, 35, mission_id);
    _mace_put_uint8_t(buf, 36, mission_type);
    _mace_put_uint8_t(buf, 37, mission_state);
    _mace_put_uint8_t(buf, 38, frame);
    _mace_put_uint8_t(buf, 39, current);
    _mace_put_uint8_t(buf, 40, autocontinue);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_ITEM_LEN);
#else
    mace_mission_item_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.seq = seq;
    packet.command = command;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;
    packet.frame = frame;
    packet.current = current;
    packet.autocontinue = autocontinue;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_ITEM_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_ITEM;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_MISSION_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_ITEM_LEN, MACE_MSG_ID_MISSION_ITEM_CRC);
}

/**
 * @brief Pack a mission_item message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system Target System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state The mission state, see MISSION_STATE
 * @param seq Sequence
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1, see MAV_CMD enum
 * @param param2 PARAM2, see MAV_CMD enum
 * @param param3 PARAM3, see MAV_CMD enum
 * @param param4 PARAM4, see MAV_CMD enum
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_item_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t mission_system,uint8_t mission_creator,uint8_t mission_id,uint8_t mission_type,uint8_t mission_state,uint16_t seq,uint8_t frame,uint16_t command,uint8_t current,uint8_t autocontinue,float param1,float param2,float param3,float param4,float x,float y,float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_ITEM_LEN];
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_float(buf, 16, x);
    _mace_put_float(buf, 20, y);
    _mace_put_float(buf, 24, z);
    _mace_put_uint16_t(buf, 28, seq);
    _mace_put_uint16_t(buf, 30, command);
    _mace_put_uint8_t(buf, 32, target_system);
    _mace_put_uint8_t(buf, 33, mission_system);
    _mace_put_uint8_t(buf, 34, mission_creator);
    _mace_put_uint8_t(buf, 35, mission_id);
    _mace_put_uint8_t(buf, 36, mission_type);
    _mace_put_uint8_t(buf, 37, mission_state);
    _mace_put_uint8_t(buf, 38, frame);
    _mace_put_uint8_t(buf, 39, current);
    _mace_put_uint8_t(buf, 40, autocontinue);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_ITEM_LEN);
#else
    mace_mission_item_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.seq = seq;
    packet.command = command;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;
    packet.frame = frame;
    packet.current = current;
    packet.autocontinue = autocontinue;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_ITEM_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_ITEM;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_MISSION_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_ITEM_LEN, MACE_MSG_ID_MISSION_ITEM_CRC);
}

/**
 * @brief Encode a mission_item struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_item C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_item_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_mission_item_t* mission_item)
{
    return mace_msg_mission_item_pack(system_id, component_id, msg, mission_item->target_system, mission_item->mission_system, mission_item->mission_creator, mission_item->mission_id, mission_item->mission_type, mission_item->mission_state, mission_item->seq, mission_item->frame, mission_item->command, mission_item->current, mission_item->autocontinue, mission_item->param1, mission_item->param2, mission_item->param3, mission_item->param4, mission_item->x, mission_item->y, mission_item->z);
}

/**
 * @brief Encode a mission_item struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_item C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_item_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_mission_item_t* mission_item)
{
    return mace_msg_mission_item_pack_chan(system_id, component_id, chan, msg, mission_item->target_system, mission_item->mission_system, mission_item->mission_creator, mission_item->mission_id, mission_item->mission_type, mission_item->mission_state, mission_item->seq, mission_item->frame, mission_item->command, mission_item->current, mission_item->autocontinue, mission_item->param1, mission_item->param2, mission_item->param3, mission_item->param4, mission_item->x, mission_item->y, mission_item->z);
}

/**
 * @brief Send a mission_item message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system Target System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state The mission state, see MISSION_STATE
 * @param seq Sequence
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1, see MAV_CMD enum
 * @param param2 PARAM2, see MAV_CMD enum
 * @param param3 PARAM3, see MAV_CMD enum
 * @param param4 PARAM4, see MAV_CMD enum
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_mission_item_send(mace_channel_t chan, uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_ITEM_LEN];
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_float(buf, 16, x);
    _mace_put_float(buf, 20, y);
    _mace_put_float(buf, 24, z);
    _mace_put_uint16_t(buf, 28, seq);
    _mace_put_uint16_t(buf, 30, command);
    _mace_put_uint8_t(buf, 32, target_system);
    _mace_put_uint8_t(buf, 33, mission_system);
    _mace_put_uint8_t(buf, 34, mission_creator);
    _mace_put_uint8_t(buf, 35, mission_id);
    _mace_put_uint8_t(buf, 36, mission_type);
    _mace_put_uint8_t(buf, 37, mission_state);
    _mace_put_uint8_t(buf, 38, frame);
    _mace_put_uint8_t(buf, 39, current);
    _mace_put_uint8_t(buf, 40, autocontinue);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_ITEM, buf, MACE_MSG_ID_MISSION_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_ITEM_LEN, MACE_MSG_ID_MISSION_ITEM_CRC);
#else
    mace_mission_item_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.seq = seq;
    packet.command = command;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;
    packet.frame = frame;
    packet.current = current;
    packet.autocontinue = autocontinue;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_ITEM, (const char *)&packet, MACE_MSG_ID_MISSION_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_ITEM_LEN, MACE_MSG_ID_MISSION_ITEM_CRC);
#endif
}

/**
 * @brief Send a mission_item message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_mission_item_send_struct(mace_channel_t chan, const mace_mission_item_t* mission_item)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_mission_item_send(chan, mission_item->target_system, mission_item->mission_system, mission_item->mission_creator, mission_item->mission_id, mission_item->mission_type, mission_item->mission_state, mission_item->seq, mission_item->frame, mission_item->command, mission_item->current, mission_item->autocontinue, mission_item->param1, mission_item->param2, mission_item->param3, mission_item->param4, mission_item->x, mission_item->y, mission_item->z);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_ITEM, (const char *)mission_item, MACE_MSG_ID_MISSION_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_ITEM_LEN, MACE_MSG_ID_MISSION_ITEM_CRC);
#endif
}

#if MACE_MSG_ID_MISSION_ITEM_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_mission_item_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_float(buf, 0, param1);
    _mace_put_float(buf, 4, param2);
    _mace_put_float(buf, 8, param3);
    _mace_put_float(buf, 12, param4);
    _mace_put_float(buf, 16, x);
    _mace_put_float(buf, 20, y);
    _mace_put_float(buf, 24, z);
    _mace_put_uint16_t(buf, 28, seq);
    _mace_put_uint16_t(buf, 30, command);
    _mace_put_uint8_t(buf, 32, target_system);
    _mace_put_uint8_t(buf, 33, mission_system);
    _mace_put_uint8_t(buf, 34, mission_creator);
    _mace_put_uint8_t(buf, 35, mission_id);
    _mace_put_uint8_t(buf, 36, mission_type);
    _mace_put_uint8_t(buf, 37, mission_state);
    _mace_put_uint8_t(buf, 38, frame);
    _mace_put_uint8_t(buf, 39, current);
    _mace_put_uint8_t(buf, 40, autocontinue);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_ITEM, buf, MACE_MSG_ID_MISSION_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_ITEM_LEN, MACE_MSG_ID_MISSION_ITEM_CRC);
#else
    mace_mission_item_t *packet = (mace_mission_item_t *)msgbuf;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->seq = seq;
    packet->command = command;
    packet->target_system = target_system;
    packet->mission_system = mission_system;
    packet->mission_creator = mission_creator;
    packet->mission_id = mission_id;
    packet->mission_type = mission_type;
    packet->mission_state = mission_state;
    packet->frame = frame;
    packet->current = current;
    packet->autocontinue = autocontinue;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_ITEM, (const char *)packet, MACE_MSG_ID_MISSION_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_ITEM_LEN, MACE_MSG_ID_MISSION_ITEM_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_ITEM UNPACKING


/**
 * @brief Get field target_system from mission_item message
 *
 * @return Target System ID
 */
static inline uint8_t mace_msg_mission_item_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field mission_system from mission_item message
 *
 * @return Mission System ID
 */
static inline uint8_t mace_msg_mission_item_get_mission_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field mission_creator from mission_item message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_mission_item_get_mission_creator(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field mission_id from mission_item message
 *
 * @return Mission ID
 */
static inline uint8_t mace_msg_mission_item_get_mission_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field mission_type from mission_item message
 *
 * @return Mission type, see MISSION_TYPE
 */
static inline uint8_t mace_msg_mission_item_get_mission_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field mission_state from mission_item message
 *
 * @return The mission state, see MISSION_STATE
 */
static inline uint8_t mace_msg_mission_item_get_mission_state(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field seq from mission_item message
 *
 * @return Sequence
 */
static inline uint16_t mace_msg_mission_item_get_seq(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field frame from mission_item message
 *
 * @return The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 */
static inline uint8_t mace_msg_mission_item_get_frame(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field command from mission_item message
 *
 * @return The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 */
static inline uint16_t mace_msg_mission_item_get_command(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field current from mission_item message
 *
 * @return false:0, true:1
 */
static inline uint8_t mace_msg_mission_item_get_current(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field autocontinue from mission_item message
 *
 * @return autocontinue to next wp
 */
static inline uint8_t mace_msg_mission_item_get_autocontinue(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field param1 from mission_item message
 *
 * @return PARAM1, see MAV_CMD enum
 */
static inline float mace_msg_mission_item_get_param1(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  0);
}

/**
 * @brief Get field param2 from mission_item message
 *
 * @return PARAM2, see MAV_CMD enum
 */
static inline float mace_msg_mission_item_get_param2(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field param3 from mission_item message
 *
 * @return PARAM3, see MAV_CMD enum
 */
static inline float mace_msg_mission_item_get_param3(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field param4 from mission_item message
 *
 * @return PARAM4, see MAV_CMD enum
 */
static inline float mace_msg_mission_item_get_param4(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Get field x from mission_item message
 *
 * @return PARAM5 / local: x position, global: latitude
 */
static inline float mace_msg_mission_item_get_x(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  16);
}

/**
 * @brief Get field y from mission_item message
 *
 * @return PARAM6 / y position: global: longitude
 */
static inline float mace_msg_mission_item_get_y(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  20);
}

/**
 * @brief Get field z from mission_item message
 *
 * @return PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
 */
static inline float mace_msg_mission_item_get_z(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  24);
}

/**
 * @brief Decode a mission_item message into a struct
 *
 * @param msg The message to decode
 * @param mission_item C-struct to decode the message contents into
 */
static inline void mace_msg_mission_item_decode(const mace_message_t* msg, mace_mission_item_t* mission_item)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mission_item->param1 = mace_msg_mission_item_get_param1(msg);
    mission_item->param2 = mace_msg_mission_item_get_param2(msg);
    mission_item->param3 = mace_msg_mission_item_get_param3(msg);
    mission_item->param4 = mace_msg_mission_item_get_param4(msg);
    mission_item->x = mace_msg_mission_item_get_x(msg);
    mission_item->y = mace_msg_mission_item_get_y(msg);
    mission_item->z = mace_msg_mission_item_get_z(msg);
    mission_item->seq = mace_msg_mission_item_get_seq(msg);
    mission_item->command = mace_msg_mission_item_get_command(msg);
    mission_item->target_system = mace_msg_mission_item_get_target_system(msg);
    mission_item->mission_system = mace_msg_mission_item_get_mission_system(msg);
    mission_item->mission_creator = mace_msg_mission_item_get_mission_creator(msg);
    mission_item->mission_id = mace_msg_mission_item_get_mission_id(msg);
    mission_item->mission_type = mace_msg_mission_item_get_mission_type(msg);
    mission_item->mission_state = mace_msg_mission_item_get_mission_state(msg);
    mission_item->frame = mace_msg_mission_item_get_frame(msg);
    mission_item->current = mace_msg_mission_item_get_current(msg);
    mission_item->autocontinue = mace_msg_mission_item_get_autocontinue(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_MISSION_ITEM_LEN? msg->len : MACE_MSG_ID_MISSION_ITEM_LEN;
        memset(mission_item, 0, MACE_MSG_ID_MISSION_ITEM_LEN);
    memcpy(mission_item, _MACE_PAYLOAD(msg), len);
#endif
}
