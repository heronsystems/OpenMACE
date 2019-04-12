#pragma once
// MESSAGE STARTING_CURRENT_MISSION PACKING

#define MACE_MSG_ID_STARTING_CURRENT_MISSION 110

MACEPACKED(
typedef struct __mace_starting_current_mission_t {
 uint8_t mission_system; /*< Mission System ID*/
 uint8_t mission_creator; /*< Creator ID*/
 uint8_t mission_id; /*< Mission ID*/
 uint8_t mission_type; /*< Mission type, see MISSION_TYPE*/
}) mace_starting_current_mission_t;

#define MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN 4
#define MACE_MSG_ID_STARTING_CURRENT_MISSION_MIN_LEN 4
#define MACE_MSG_ID_110_LEN 4
#define MACE_MSG_ID_110_MIN_LEN 4

#define MACE_MSG_ID_STARTING_CURRENT_MISSION_CRC 224
#define MACE_MSG_ID_110_CRC 224



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_STARTING_CURRENT_MISSION { \
    110, \
    "STARTING_CURRENT_MISSION", \
    4, \
    {  { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_starting_current_mission_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_starting_current_mission_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_starting_current_mission_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_starting_current_mission_t, mission_type) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_STARTING_CURRENT_MISSION { \
    "STARTING_CURRENT_MISSION", \
    4, \
    {  { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_starting_current_mission_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_starting_current_mission_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_starting_current_mission_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_starting_current_mission_t, mission_type) }, \
         } \
}
#endif

/**
 * @brief Pack a starting_current_mission message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_starting_current_mission_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN];
    _mace_put_uint8_t(buf, 0, mission_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN);
#else
    mace_starting_current_mission_t packet;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN);
#endif

    msg->msgid = MACE_MSG_ID_STARTING_CURRENT_MISSION;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_STARTING_CURRENT_MISSION_MIN_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_CRC);
}

/**
 * @brief Pack a starting_current_mission message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_starting_current_mission_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t mission_system,uint8_t mission_creator,uint8_t mission_id,uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN];
    _mace_put_uint8_t(buf, 0, mission_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN);
#else
    mace_starting_current_mission_t packet;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN);
#endif

    msg->msgid = MACE_MSG_ID_STARTING_CURRENT_MISSION;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_STARTING_CURRENT_MISSION_MIN_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_CRC);
}

/**
 * @brief Encode a starting_current_mission struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param starting_current_mission C-struct to read the message contents from
 */
static inline uint16_t mace_msg_starting_current_mission_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_starting_current_mission_t* starting_current_mission)
{
    return mace_msg_starting_current_mission_pack(system_id, component_id, msg, starting_current_mission->mission_system, starting_current_mission->mission_creator, starting_current_mission->mission_id, starting_current_mission->mission_type);
}

/**
 * @brief Encode a starting_current_mission struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param starting_current_mission C-struct to read the message contents from
 */
static inline uint16_t mace_msg_starting_current_mission_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_starting_current_mission_t* starting_current_mission)
{
    return mace_msg_starting_current_mission_pack_chan(system_id, component_id, chan, msg, starting_current_mission->mission_system, starting_current_mission->mission_creator, starting_current_mission->mission_id, starting_current_mission->mission_type);
}

/**
 * @brief Send a starting_current_mission message
 * @param chan MAVLink channel to send the message
 *
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_starting_current_mission_send(mace_channel_t chan, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN];
    _mace_put_uint8_t(buf, 0, mission_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_STARTING_CURRENT_MISSION, buf, MACE_MSG_ID_STARTING_CURRENT_MISSION_MIN_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_CRC);
#else
    mace_starting_current_mission_t packet;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_STARTING_CURRENT_MISSION, (const char *)&packet, MACE_MSG_ID_STARTING_CURRENT_MISSION_MIN_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_CRC);
#endif
}

/**
 * @brief Send a starting_current_mission message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_starting_current_mission_send_struct(mace_channel_t chan, const mace_starting_current_mission_t* starting_current_mission)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_starting_current_mission_send(chan, starting_current_mission->mission_system, starting_current_mission->mission_creator, starting_current_mission->mission_id, starting_current_mission->mission_type);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_STARTING_CURRENT_MISSION, (const char *)starting_current_mission, MACE_MSG_ID_STARTING_CURRENT_MISSION_MIN_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_CRC);
#endif
}

#if MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_starting_current_mission_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, mission_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_STARTING_CURRENT_MISSION, buf, MACE_MSG_ID_STARTING_CURRENT_MISSION_MIN_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_CRC);
#else
    mace_starting_current_mission_t *packet = (mace_starting_current_mission_t *)msgbuf;
    packet->mission_system = mission_system;
    packet->mission_creator = mission_creator;
    packet->mission_id = mission_id;
    packet->mission_type = mission_type;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_STARTING_CURRENT_MISSION, (const char *)packet, MACE_MSG_ID_STARTING_CURRENT_MISSION_MIN_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN, MACE_MSG_ID_STARTING_CURRENT_MISSION_CRC);
#endif
}
#endif

#endif

// MESSAGE STARTING_CURRENT_MISSION UNPACKING


/**
 * @brief Get field mission_system from starting_current_mission message
 *
 * @return Mission System ID
 */
static inline uint8_t mace_msg_starting_current_mission_get_mission_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mission_creator from starting_current_mission message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_starting_current_mission_get_mission_creator(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field mission_id from starting_current_mission message
 *
 * @return Mission ID
 */
static inline uint8_t mace_msg_starting_current_mission_get_mission_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_type from starting_current_mission message
 *
 * @return Mission type, see MISSION_TYPE
 */
static inline uint8_t mace_msg_starting_current_mission_get_mission_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a starting_current_mission message into a struct
 *
 * @param msg The message to decode
 * @param starting_current_mission C-struct to decode the message contents into
 */
static inline void mace_msg_starting_current_mission_decode(const mace_message_t* msg, mace_starting_current_mission_t* starting_current_mission)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    starting_current_mission->mission_system = mace_msg_starting_current_mission_get_mission_system(msg);
    starting_current_mission->mission_creator = mace_msg_starting_current_mission_get_mission_creator(msg);
    starting_current_mission->mission_id = mace_msg_starting_current_mission_get_mission_id(msg);
    starting_current_mission->mission_type = mace_msg_starting_current_mission_get_mission_type(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN? msg->len : MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN;
        memset(starting_current_mission, 0, MACE_MSG_ID_STARTING_CURRENT_MISSION_LEN);
    memcpy(starting_current_mission, _MACE_PAYLOAD(msg), len);
#endif
}
