#pragma once
// MESSAGE MISSION_REQUEST_PARTIAL_LIST PACKING

#define MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST 108

MACEPACKED(
typedef struct __mace_mission_request_partial_list_t {
 int16_t start_index; /*< Start index, 0 by default*/
 int16_t end_index; /*< End index, -1 by default (-1: send list to end). Else a valid index of the list*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t mission_type; /*< Mission type, see MISSION_TYPE*/
}) mace_mission_request_partial_list_t;

#define MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN 7
#define MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN 7
#define MACE_MSG_ID_108_LEN 7
#define MACE_MSG_ID_108_MIN_LEN 7

#define MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC 4
#define MACE_MSG_ID_108_CRC 4



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_MISSION_REQUEST_PARTIAL_LIST { \
    108, \
    "MISSION_REQUEST_PARTIAL_LIST", \
    5, \
    {  { "start_index", NULL, MACE_TYPE_INT16_T, 0, 0, offsetof(mace_mission_request_partial_list_t, start_index) }, \
         { "end_index", NULL, MACE_TYPE_INT16_T, 0, 2, offsetof(mace_mission_request_partial_list_t, end_index) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_mission_request_partial_list_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_mission_request_partial_list_t, target_component) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_mission_request_partial_list_t, mission_type) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_MISSION_REQUEST_PARTIAL_LIST { \
    "MISSION_REQUEST_PARTIAL_LIST", \
    5, \
    {  { "start_index", NULL, MACE_TYPE_INT16_T, 0, 0, offsetof(mace_mission_request_partial_list_t, start_index) }, \
         { "end_index", NULL, MACE_TYPE_INT16_T, 0, 2, offsetof(mace_mission_request_partial_list_t, end_index) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_mission_request_partial_list_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_mission_request_partial_list_t, target_component) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_mission_request_partial_list_t, mission_type) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_request_partial_list message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param start_index Start index, 0 by default
 * @param end_index End index, -1 by default (-1: send list to end). Else a valid index of the list
 * @param mission_type Mission type, see MISSION_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_request_partial_list_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t target_component, int16_t start_index, int16_t end_index, uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN];
    _mace_put_int16_t(buf, 0, start_index);
    _mace_put_int16_t(buf, 2, end_index);
    _mace_put_uint8_t(buf, 4, target_system);
    _mace_put_uint8_t(buf, 5, target_component);
    _mace_put_uint8_t(buf, 6, mission_type);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#else
    mace_mission_request_partial_list_t packet;
    packet.start_index = start_index;
    packet.end_index = end_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mission_type = mission_type;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
}

/**
 * @brief Pack a mission_request_partial_list message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param start_index Start index, 0 by default
 * @param end_index End index, -1 by default (-1: send list to end). Else a valid index of the list
 * @param mission_type Mission type, see MISSION_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_request_partial_list_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,int16_t start_index,int16_t end_index,uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN];
    _mace_put_int16_t(buf, 0, start_index);
    _mace_put_int16_t(buf, 2, end_index);
    _mace_put_uint8_t(buf, 4, target_system);
    _mace_put_uint8_t(buf, 5, target_component);
    _mace_put_uint8_t(buf, 6, mission_type);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#else
    mace_mission_request_partial_list_t packet;
    packet.start_index = start_index;
    packet.end_index = end_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mission_type = mission_type;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
}

/**
 * @brief Encode a mission_request_partial_list struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_partial_list C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_request_partial_list_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_mission_request_partial_list_t* mission_request_partial_list)
{
    return mace_msg_mission_request_partial_list_pack(system_id, component_id, msg, mission_request_partial_list->target_system, mission_request_partial_list->target_component, mission_request_partial_list->start_index, mission_request_partial_list->end_index, mission_request_partial_list->mission_type);
}

/**
 * @brief Encode a mission_request_partial_list struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_partial_list C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_request_partial_list_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_mission_request_partial_list_t* mission_request_partial_list)
{
    return mace_msg_mission_request_partial_list_pack_chan(system_id, component_id, chan, msg, mission_request_partial_list->target_system, mission_request_partial_list->target_component, mission_request_partial_list->start_index, mission_request_partial_list->end_index, mission_request_partial_list->mission_type);
}

/**
 * @brief Send a mission_request_partial_list message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param start_index Start index, 0 by default
 * @param end_index End index, -1 by default (-1: send list to end). Else a valid index of the list
 * @param mission_type Mission type, see MISSION_TYPE
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_mission_request_partial_list_send(mace_channel_t chan, uint8_t target_system, uint8_t target_component, int16_t start_index, int16_t end_index, uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN];
    _mace_put_int16_t(buf, 0, start_index);
    _mace_put_int16_t(buf, 2, end_index);
    _mace_put_uint8_t(buf, 4, target_system);
    _mace_put_uint8_t(buf, 5, target_component);
    _mace_put_uint8_t(buf, 6, mission_type);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, buf, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
#else
    mace_mission_request_partial_list_t packet;
    packet.start_index = start_index;
    packet.end_index = end_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mission_type = mission_type;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, (const char *)&packet, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
#endif
}

/**
 * @brief Send a mission_request_partial_list message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_mission_request_partial_list_send_struct(mace_channel_t chan, const mace_mission_request_partial_list_t* mission_request_partial_list)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_mission_request_partial_list_send(chan, mission_request_partial_list->target_system, mission_request_partial_list->target_component, mission_request_partial_list->start_index, mission_request_partial_list->end_index, mission_request_partial_list->mission_type);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, (const char *)mission_request_partial_list, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
#endif
}

#if MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_mission_request_partial_list_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t target_component, int16_t start_index, int16_t end_index, uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_int16_t(buf, 0, start_index);
    _mace_put_int16_t(buf, 2, end_index);
    _mace_put_uint8_t(buf, 4, target_system);
    _mace_put_uint8_t(buf, 5, target_component);
    _mace_put_uint8_t(buf, 6, mission_type);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, buf, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
#else
    mace_mission_request_partial_list_t *packet = (mace_mission_request_partial_list_t *)msgbuf;
    packet->start_index = start_index;
    packet->end_index = end_index;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->mission_type = mission_type;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, (const char *)packet, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_REQUEST_PARTIAL_LIST UNPACKING


/**
 * @brief Get field target_system from mission_request_partial_list message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_mission_request_partial_list_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from mission_request_partial_list message
 *
 * @return Component ID
 */
static inline uint8_t mace_msg_mission_request_partial_list_get_target_component(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field start_index from mission_request_partial_list message
 *
 * @return Start index, 0 by default
 */
static inline int16_t mace_msg_mission_request_partial_list_get_start_index(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field end_index from mission_request_partial_list message
 *
 * @return End index, -1 by default (-1: send list to end). Else a valid index of the list
 */
static inline int16_t mace_msg_mission_request_partial_list_get_end_index(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field mission_type from mission_request_partial_list message
 *
 * @return Mission type, see MISSION_TYPE
 */
static inline uint8_t mace_msg_mission_request_partial_list_get_mission_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a mission_request_partial_list message into a struct
 *
 * @param msg The message to decode
 * @param mission_request_partial_list C-struct to decode the message contents into
 */
static inline void mace_msg_mission_request_partial_list_decode(const mace_message_t* msg, mace_mission_request_partial_list_t* mission_request_partial_list)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mission_request_partial_list->start_index = mace_msg_mission_request_partial_list_get_start_index(msg);
    mission_request_partial_list->end_index = mace_msg_mission_request_partial_list_get_end_index(msg);
    mission_request_partial_list->target_system = mace_msg_mission_request_partial_list_get_target_system(msg);
    mission_request_partial_list->target_component = mace_msg_mission_request_partial_list_get_target_component(msg);
    mission_request_partial_list->mission_type = mace_msg_mission_request_partial_list_get_mission_type(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN? msg->len : MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN;
        memset(mission_request_partial_list, 0, MACE_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
    memcpy(mission_request_partial_list, _MACE_PAYLOAD(msg), len);
#endif
}
