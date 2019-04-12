#pragma once
// MESSAGE MISSION_CLEAR PACKING

#define MACE_MSG_ID_MISSION_CLEAR 114

MACEPACKED(
typedef struct __mace_mission_clear_t {
 uint8_t target_system; /*< System ID*/
 uint8_t mission_creator; /*< Creator ID*/
 uint8_t mission_id; /*< Mission ID*/
 uint8_t mission_type; /*< Mission type, see MISSION_TYPE*/
}) mace_mission_clear_t;

#define MACE_MSG_ID_MISSION_CLEAR_LEN 4
#define MACE_MSG_ID_MISSION_CLEAR_MIN_LEN 4
#define MACE_MSG_ID_114_LEN 4
#define MACE_MSG_ID_114_MIN_LEN 4

#define MACE_MSG_ID_MISSION_CLEAR_CRC 34
#define MACE_MSG_ID_114_CRC 34



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_MISSION_CLEAR { \
    114, \
    "MISSION_CLEAR", \
    4, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_mission_clear_t, target_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_mission_clear_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_mission_clear_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_mission_clear_t, mission_type) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_MISSION_CLEAR { \
    "MISSION_CLEAR", \
    4, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_mission_clear_t, target_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_mission_clear_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_mission_clear_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_mission_clear_t, mission_type) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_clear message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_clear_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_CLEAR_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_CLEAR_LEN);
#else
    mace_mission_clear_t packet;
    packet.target_system = target_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_CLEAR_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_CLEAR;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_MISSION_CLEAR_MIN_LEN, MACE_MSG_ID_MISSION_CLEAR_LEN, MACE_MSG_ID_MISSION_CLEAR_CRC);
}

/**
 * @brief Pack a mission_clear message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_clear_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t mission_creator,uint8_t mission_id,uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_CLEAR_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_CLEAR_LEN);
#else
    mace_mission_clear_t packet;
    packet.target_system = target_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_CLEAR_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_CLEAR;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_MISSION_CLEAR_MIN_LEN, MACE_MSG_ID_MISSION_CLEAR_LEN, MACE_MSG_ID_MISSION_CLEAR_CRC);
}

/**
 * @brief Encode a mission_clear struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_clear C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_clear_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_mission_clear_t* mission_clear)
{
    return mace_msg_mission_clear_pack(system_id, component_id, msg, mission_clear->target_system, mission_clear->mission_creator, mission_clear->mission_id, mission_clear->mission_type);
}

/**
 * @brief Encode a mission_clear struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_clear C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_clear_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_mission_clear_t* mission_clear)
{
    return mace_msg_mission_clear_pack_chan(system_id, component_id, chan, msg, mission_clear->target_system, mission_clear->mission_creator, mission_clear->mission_id, mission_clear->mission_type);
}

/**
 * @brief Send a mission_clear message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_mission_clear_send(mace_channel_t chan, uint8_t target_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_CLEAR_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_CLEAR, buf, MACE_MSG_ID_MISSION_CLEAR_MIN_LEN, MACE_MSG_ID_MISSION_CLEAR_LEN, MACE_MSG_ID_MISSION_CLEAR_CRC);
#else
    mace_mission_clear_t packet;
    packet.target_system = target_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_CLEAR, (const char *)&packet, MACE_MSG_ID_MISSION_CLEAR_MIN_LEN, MACE_MSG_ID_MISSION_CLEAR_LEN, MACE_MSG_ID_MISSION_CLEAR_CRC);
#endif
}

/**
 * @brief Send a mission_clear message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_mission_clear_send_struct(mace_channel_t chan, const mace_mission_clear_t* mission_clear)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_mission_clear_send(chan, mission_clear->target_system, mission_clear->mission_creator, mission_clear->mission_id, mission_clear->mission_type);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_CLEAR, (const char *)mission_clear, MACE_MSG_ID_MISSION_CLEAR_MIN_LEN, MACE_MSG_ID_MISSION_CLEAR_LEN, MACE_MSG_ID_MISSION_CLEAR_CRC);
#endif
}

#if MACE_MSG_ID_MISSION_CLEAR_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_mission_clear_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_CLEAR, buf, MACE_MSG_ID_MISSION_CLEAR_MIN_LEN, MACE_MSG_ID_MISSION_CLEAR_LEN, MACE_MSG_ID_MISSION_CLEAR_CRC);
#else
    mace_mission_clear_t *packet = (mace_mission_clear_t *)msgbuf;
    packet->target_system = target_system;
    packet->mission_creator = mission_creator;
    packet->mission_id = mission_id;
    packet->mission_type = mission_type;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_CLEAR, (const char *)packet, MACE_MSG_ID_MISSION_CLEAR_MIN_LEN, MACE_MSG_ID_MISSION_CLEAR_LEN, MACE_MSG_ID_MISSION_CLEAR_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_CLEAR UNPACKING


/**
 * @brief Get field target_system from mission_clear message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_mission_clear_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mission_creator from mission_clear message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_mission_clear_get_mission_creator(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field mission_id from mission_clear message
 *
 * @return Mission ID
 */
static inline uint8_t mace_msg_mission_clear_get_mission_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_type from mission_clear message
 *
 * @return Mission type, see MISSION_TYPE
 */
static inline uint8_t mace_msg_mission_clear_get_mission_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a mission_clear message into a struct
 *
 * @param msg The message to decode
 * @param mission_clear C-struct to decode the message contents into
 */
static inline void mace_msg_mission_clear_decode(const mace_message_t* msg, mace_mission_clear_t* mission_clear)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mission_clear->target_system = mace_msg_mission_clear_get_target_system(msg);
    mission_clear->mission_creator = mace_msg_mission_clear_get_mission_creator(msg);
    mission_clear->mission_id = mace_msg_mission_clear_get_mission_id(msg);
    mission_clear->mission_type = mace_msg_mission_clear_get_mission_type(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_MISSION_CLEAR_LEN? msg->len : MACE_MSG_ID_MISSION_CLEAR_LEN;
        memset(mission_clear, 0, MACE_MSG_ID_MISSION_CLEAR_LEN);
    memcpy(mission_clear, _MACE_PAYLOAD(msg), len);
#endif
}
