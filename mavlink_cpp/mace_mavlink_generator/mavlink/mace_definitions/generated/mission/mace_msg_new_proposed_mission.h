#pragma once
// MESSAGE NEW_PROPOSED_MISSION PACKING

#define MACE_MSG_ID_NEW_PROPOSED_MISSION 101

MACEPACKED(
typedef struct __mace_new_proposed_mission_t {
 uint16_t count; /*< Number of mission items in the sequence*/
 uint8_t target_system; /*< System ID*/
 uint8_t mission_creator; /*< Creator ID*/
 uint8_t mission_id; /*< Mission ID*/
 uint8_t mission_type; /*< Mission type, see MISSION_TYPE*/
 uint8_t mission_state; /*< Mission type, see MISSION_STATE*/
}) mace_new_proposed_mission_t;

#define MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN 7
#define MACE_MSG_ID_NEW_PROPOSED_MISSION_MIN_LEN 7
#define MACE_MSG_ID_101_LEN 7
#define MACE_MSG_ID_101_MIN_LEN 7

#define MACE_MSG_ID_NEW_PROPOSED_MISSION_CRC 18
#define MACE_MSG_ID_101_CRC 18



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_NEW_PROPOSED_MISSION { \
    101, \
    "NEW_PROPOSED_MISSION", \
    6, \
    {  { "count", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_new_proposed_mission_t, count) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_new_proposed_mission_t, target_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_new_proposed_mission_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_new_proposed_mission_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_new_proposed_mission_t, mission_type) }, \
         { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_new_proposed_mission_t, mission_state) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_NEW_PROPOSED_MISSION { \
    "NEW_PROPOSED_MISSION", \
    6, \
    {  { "count", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_new_proposed_mission_t, count) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_new_proposed_mission_t, target_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_new_proposed_mission_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_new_proposed_mission_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_new_proposed_mission_t, mission_type) }, \
         { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_new_proposed_mission_t, mission_state) }, \
         } \
}
#endif

/**
 * @brief Pack a new_proposed_mission message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state Mission type, see MISSION_STATE
 * @param count Number of mission items in the sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_new_proposed_mission_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state, uint16_t count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN];
    _mace_put_uint16_t(buf, 0, count);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, mission_creator);
    _mace_put_uint8_t(buf, 4, mission_id);
    _mace_put_uint8_t(buf, 5, mission_type);
    _mace_put_uint8_t(buf, 6, mission_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN);
#else
    mace_new_proposed_mission_t packet;
    packet.count = count;
    packet.target_system = target_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN);
#endif

    msg->msgid = MACE_MSG_ID_NEW_PROPOSED_MISSION;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_NEW_PROPOSED_MISSION_MIN_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_CRC);
}

/**
 * @brief Pack a new_proposed_mission message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state Mission type, see MISSION_STATE
 * @param count Number of mission items in the sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_new_proposed_mission_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t mission_creator,uint8_t mission_id,uint8_t mission_type,uint8_t mission_state,uint16_t count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN];
    _mace_put_uint16_t(buf, 0, count);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, mission_creator);
    _mace_put_uint8_t(buf, 4, mission_id);
    _mace_put_uint8_t(buf, 5, mission_type);
    _mace_put_uint8_t(buf, 6, mission_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN);
#else
    mace_new_proposed_mission_t packet;
    packet.count = count;
    packet.target_system = target_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN);
#endif

    msg->msgid = MACE_MSG_ID_NEW_PROPOSED_MISSION;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_NEW_PROPOSED_MISSION_MIN_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_CRC);
}

/**
 * @brief Encode a new_proposed_mission struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param new_proposed_mission C-struct to read the message contents from
 */
static inline uint16_t mace_msg_new_proposed_mission_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_new_proposed_mission_t* new_proposed_mission)
{
    return mace_msg_new_proposed_mission_pack(system_id, component_id, msg, new_proposed_mission->target_system, new_proposed_mission->mission_creator, new_proposed_mission->mission_id, new_proposed_mission->mission_type, new_proposed_mission->mission_state, new_proposed_mission->count);
}

/**
 * @brief Encode a new_proposed_mission struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param new_proposed_mission C-struct to read the message contents from
 */
static inline uint16_t mace_msg_new_proposed_mission_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_new_proposed_mission_t* new_proposed_mission)
{
    return mace_msg_new_proposed_mission_pack_chan(system_id, component_id, chan, msg, new_proposed_mission->target_system, new_proposed_mission->mission_creator, new_proposed_mission->mission_id, new_proposed_mission->mission_type, new_proposed_mission->mission_state, new_proposed_mission->count);
}

/**
 * @brief Send a new_proposed_mission message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state Mission type, see MISSION_STATE
 * @param count Number of mission items in the sequence
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_new_proposed_mission_send(mace_channel_t chan, uint8_t target_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state, uint16_t count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN];
    _mace_put_uint16_t(buf, 0, count);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, mission_creator);
    _mace_put_uint8_t(buf, 4, mission_id);
    _mace_put_uint8_t(buf, 5, mission_type);
    _mace_put_uint8_t(buf, 6, mission_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_NEW_PROPOSED_MISSION, buf, MACE_MSG_ID_NEW_PROPOSED_MISSION_MIN_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_CRC);
#else
    mace_new_proposed_mission_t packet;
    packet.count = count;
    packet.target_system = target_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_NEW_PROPOSED_MISSION, (const char *)&packet, MACE_MSG_ID_NEW_PROPOSED_MISSION_MIN_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_CRC);
#endif
}

/**
 * @brief Send a new_proposed_mission message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_new_proposed_mission_send_struct(mace_channel_t chan, const mace_new_proposed_mission_t* new_proposed_mission)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_new_proposed_mission_send(chan, new_proposed_mission->target_system, new_proposed_mission->mission_creator, new_proposed_mission->mission_id, new_proposed_mission->mission_type, new_proposed_mission->mission_state, new_proposed_mission->count);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_NEW_PROPOSED_MISSION, (const char *)new_proposed_mission, MACE_MSG_ID_NEW_PROPOSED_MISSION_MIN_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_CRC);
#endif
}

#if MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_new_proposed_mission_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state, uint16_t count)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint16_t(buf, 0, count);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, mission_creator);
    _mace_put_uint8_t(buf, 4, mission_id);
    _mace_put_uint8_t(buf, 5, mission_type);
    _mace_put_uint8_t(buf, 6, mission_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_NEW_PROPOSED_MISSION, buf, MACE_MSG_ID_NEW_PROPOSED_MISSION_MIN_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_CRC);
#else
    mace_new_proposed_mission_t *packet = (mace_new_proposed_mission_t *)msgbuf;
    packet->count = count;
    packet->target_system = target_system;
    packet->mission_creator = mission_creator;
    packet->mission_id = mission_id;
    packet->mission_type = mission_type;
    packet->mission_state = mission_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_NEW_PROPOSED_MISSION, (const char *)packet, MACE_MSG_ID_NEW_PROPOSED_MISSION_MIN_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN, MACE_MSG_ID_NEW_PROPOSED_MISSION_CRC);
#endif
}
#endif

#endif

// MESSAGE NEW_PROPOSED_MISSION UNPACKING


/**
 * @brief Get field target_system from new_proposed_mission message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_new_proposed_mission_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_creator from new_proposed_mission message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_new_proposed_mission_get_mission_creator(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field mission_id from new_proposed_mission message
 *
 * @return Mission ID
 */
static inline uint8_t mace_msg_new_proposed_mission_get_mission_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field mission_type from new_proposed_mission message
 *
 * @return Mission type, see MISSION_TYPE
 */
static inline uint8_t mace_msg_new_proposed_mission_get_mission_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field mission_state from new_proposed_mission message
 *
 * @return Mission type, see MISSION_STATE
 */
static inline uint8_t mace_msg_new_proposed_mission_get_mission_state(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field count from new_proposed_mission message
 *
 * @return Number of mission items in the sequence
 */
static inline uint16_t mace_msg_new_proposed_mission_get_count(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a new_proposed_mission message into a struct
 *
 * @param msg The message to decode
 * @param new_proposed_mission C-struct to decode the message contents into
 */
static inline void mace_msg_new_proposed_mission_decode(const mace_message_t* msg, mace_new_proposed_mission_t* new_proposed_mission)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    new_proposed_mission->count = mace_msg_new_proposed_mission_get_count(msg);
    new_proposed_mission->target_system = mace_msg_new_proposed_mission_get_target_system(msg);
    new_proposed_mission->mission_creator = mace_msg_new_proposed_mission_get_mission_creator(msg);
    new_proposed_mission->mission_id = mace_msg_new_proposed_mission_get_mission_id(msg);
    new_proposed_mission->mission_type = mace_msg_new_proposed_mission_get_mission_type(msg);
    new_proposed_mission->mission_state = mace_msg_new_proposed_mission_get_mission_state(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN? msg->len : MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN;
        memset(new_proposed_mission, 0, MACE_MSG_ID_NEW_PROPOSED_MISSION_LEN);
    memcpy(new_proposed_mission, _MACE_PAYLOAD(msg), len);
#endif
}
