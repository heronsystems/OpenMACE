#pragma once
// MESSAGE MISSION_COUNT PACKING

#define MACE_MSG_ID_MISSION_COUNT 105

MACEPACKED(
typedef struct __mace_mission_count_t {
 uint16_t count; /*< Number of mission items in the sequence*/
 uint8_t target_system; /*< System ID*/
 uint8_t mission_system; /*< Mission System ID*/
 uint8_t mission_creator; /*< Creator ID*/
 uint8_t mission_id; /*< Mission ID*/
 uint8_t mission_type; /*< Mission type, see MISSION_TYPE*/
 uint8_t mission_state; /*< The mission state, see MISSION_STATE*/
}) mace_mission_count_t;

#define MACE_MSG_ID_MISSION_COUNT_LEN 8
#define MACE_MSG_ID_MISSION_COUNT_MIN_LEN 8
#define MACE_MSG_ID_105_LEN 8
#define MACE_MSG_ID_105_MIN_LEN 8

#define MACE_MSG_ID_MISSION_COUNT_CRC 165
#define MACE_MSG_ID_105_CRC 165



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_MISSION_COUNT { \
    105, \
    "MISSION_COUNT", \
    7, \
    {  { "count", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_mission_count_t, count) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_mission_count_t, target_system) }, \
         { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_mission_count_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_mission_count_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_mission_count_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_mission_count_t, mission_type) }, \
         { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 7, offsetof(mace_mission_count_t, mission_state) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_MISSION_COUNT { \
    "MISSION_COUNT", \
    7, \
    {  { "count", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_mission_count_t, count) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_mission_count_t, target_system) }, \
         { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_mission_count_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_mission_count_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_mission_count_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_mission_count_t, mission_type) }, \
         { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 7, offsetof(mace_mission_count_t, mission_state) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_count message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state The mission state, see MISSION_STATE
 * @param count Number of mission items in the sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_count_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state, uint16_t count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_COUNT_LEN];
    _mace_put_uint16_t(buf, 0, count);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, mission_system);
    _mace_put_uint8_t(buf, 4, mission_creator);
    _mace_put_uint8_t(buf, 5, mission_id);
    _mace_put_uint8_t(buf, 6, mission_type);
    _mace_put_uint8_t(buf, 7, mission_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_COUNT_LEN);
#else
    mace_mission_count_t packet;
    packet.count = count;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_COUNT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_COUNT;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_MISSION_COUNT_MIN_LEN, MACE_MSG_ID_MISSION_COUNT_LEN, MACE_MSG_ID_MISSION_COUNT_CRC);
}

/**
 * @brief Pack a mission_count message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state The mission state, see MISSION_STATE
 * @param count Number of mission items in the sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_count_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t mission_system,uint8_t mission_creator,uint8_t mission_id,uint8_t mission_type,uint8_t mission_state,uint16_t count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_COUNT_LEN];
    _mace_put_uint16_t(buf, 0, count);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, mission_system);
    _mace_put_uint8_t(buf, 4, mission_creator);
    _mace_put_uint8_t(buf, 5, mission_id);
    _mace_put_uint8_t(buf, 6, mission_type);
    _mace_put_uint8_t(buf, 7, mission_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_COUNT_LEN);
#else
    mace_mission_count_t packet;
    packet.count = count;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_COUNT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_COUNT;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_MISSION_COUNT_MIN_LEN, MACE_MSG_ID_MISSION_COUNT_LEN, MACE_MSG_ID_MISSION_COUNT_CRC);
}

/**
 * @brief Encode a mission_count struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_count C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_count_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_mission_count_t* mission_count)
{
    return mace_msg_mission_count_pack(system_id, component_id, msg, mission_count->target_system, mission_count->mission_system, mission_count->mission_creator, mission_count->mission_id, mission_count->mission_type, mission_count->mission_state, mission_count->count);
}

/**
 * @brief Encode a mission_count struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_count C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_count_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_mission_count_t* mission_count)
{
    return mace_msg_mission_count_pack_chan(system_id, component_id, chan, msg, mission_count->target_system, mission_count->mission_system, mission_count->mission_creator, mission_count->mission_id, mission_count->mission_type, mission_count->mission_state, mission_count->count);
}

/**
 * @brief Send a mission_count message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state The mission state, see MISSION_STATE
 * @param count Number of mission items in the sequence
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_mission_count_send(mace_channel_t chan, uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state, uint16_t count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_COUNT_LEN];
    _mace_put_uint16_t(buf, 0, count);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, mission_system);
    _mace_put_uint8_t(buf, 4, mission_creator);
    _mace_put_uint8_t(buf, 5, mission_id);
    _mace_put_uint8_t(buf, 6, mission_type);
    _mace_put_uint8_t(buf, 7, mission_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_COUNT, buf, MACE_MSG_ID_MISSION_COUNT_MIN_LEN, MACE_MSG_ID_MISSION_COUNT_LEN, MACE_MSG_ID_MISSION_COUNT_CRC);
#else
    mace_mission_count_t packet;
    packet.count = count;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_COUNT, (const char *)&packet, MACE_MSG_ID_MISSION_COUNT_MIN_LEN, MACE_MSG_ID_MISSION_COUNT_LEN, MACE_MSG_ID_MISSION_COUNT_CRC);
#endif
}

/**
 * @brief Send a mission_count message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_mission_count_send_struct(mace_channel_t chan, const mace_mission_count_t* mission_count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_mission_count_send(chan, mission_count->target_system, mission_count->mission_system, mission_count->mission_creator, mission_count->mission_id, mission_count->mission_type, mission_count->mission_state, mission_count->count);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_COUNT, (const char *)mission_count, MACE_MSG_ID_MISSION_COUNT_MIN_LEN, MACE_MSG_ID_MISSION_COUNT_LEN, MACE_MSG_ID_MISSION_COUNT_CRC);
#endif
}

#if MACE_MSG_ID_MISSION_COUNT_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_mission_count_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state, uint16_t count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint16_t(buf, 0, count);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, mission_system);
    _mace_put_uint8_t(buf, 4, mission_creator);
    _mace_put_uint8_t(buf, 5, mission_id);
    _mace_put_uint8_t(buf, 6, mission_type);
    _mace_put_uint8_t(buf, 7, mission_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_COUNT, buf, MACE_MSG_ID_MISSION_COUNT_MIN_LEN, MACE_MSG_ID_MISSION_COUNT_LEN, MACE_MSG_ID_MISSION_COUNT_CRC);
#else
    mace_mission_count_t *packet = (mace_mission_count_t *)msgbuf;
    packet->count = count;
    packet->target_system = target_system;
    packet->mission_system = mission_system;
    packet->mission_creator = mission_creator;
    packet->mission_id = mission_id;
    packet->mission_type = mission_type;
    packet->mission_state = mission_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_COUNT, (const char *)packet, MACE_MSG_ID_MISSION_COUNT_MIN_LEN, MACE_MSG_ID_MISSION_COUNT_LEN, MACE_MSG_ID_MISSION_COUNT_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_COUNT UNPACKING


/**
 * @brief Get field target_system from mission_count message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_mission_count_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_system from mission_count message
 *
 * @return Mission System ID
 */
static inline uint8_t mace_msg_mission_count_get_mission_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field mission_creator from mission_count message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_mission_count_get_mission_creator(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field mission_id from mission_count message
 *
 * @return Mission ID
 */
static inline uint8_t mace_msg_mission_count_get_mission_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field mission_type from mission_count message
 *
 * @return Mission type, see MISSION_TYPE
 */
static inline uint8_t mace_msg_mission_count_get_mission_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field mission_state from mission_count message
 *
 * @return The mission state, see MISSION_STATE
 */
static inline uint8_t mace_msg_mission_count_get_mission_state(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field count from mission_count message
 *
 * @return Number of mission items in the sequence
 */
static inline uint16_t mace_msg_mission_count_get_count(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a mission_count message into a struct
 *
 * @param msg The message to decode
 * @param mission_count C-struct to decode the message contents into
 */
static inline void mace_msg_mission_count_decode(const mace_message_t* msg, mace_mission_count_t* mission_count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mission_count->count = mace_msg_mission_count_get_count(msg);
    mission_count->target_system = mace_msg_mission_count_get_target_system(msg);
    mission_count->mission_system = mace_msg_mission_count_get_mission_system(msg);
    mission_count->mission_creator = mace_msg_mission_count_get_mission_creator(msg);
    mission_count->mission_id = mace_msg_mission_count_get_mission_id(msg);
    mission_count->mission_type = mace_msg_mission_count_get_mission_type(msg);
    mission_count->mission_state = mace_msg_mission_count_get_mission_state(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_MISSION_COUNT_LEN? msg->len : MACE_MSG_ID_MISSION_COUNT_LEN;
        memset(mission_count, 0, MACE_MSG_ID_MISSION_COUNT_LEN);
    memcpy(mission_count, _MACE_PAYLOAD(msg), len);
#endif
}
