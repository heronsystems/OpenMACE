#pragma once
// MESSAGE MISSION_SET_CURRENT PACKING

#define MACE_MSG_ID_MISSION_SET_CURRENT 111

MACEPACKED(
typedef struct __mace_mission_set_current_t {
 uint16_t seq; /*< Sequence*/
 uint8_t mission_system; /*< Mission System ID*/
 uint8_t mission_creator; /*< Creator ID*/
 uint8_t mission_id; /*< Mission ID*/
}) mace_mission_set_current_t;

#define MACE_MSG_ID_MISSION_SET_CURRENT_LEN 5
#define MACE_MSG_ID_MISSION_SET_CURRENT_MIN_LEN 5
#define MACE_MSG_ID_111_LEN 5
#define MACE_MSG_ID_111_MIN_LEN 5

#define MACE_MSG_ID_MISSION_SET_CURRENT_CRC 156
#define MACE_MSG_ID_111_CRC 156



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_MISSION_SET_CURRENT { \
    111, \
    "MISSION_SET_CURRENT", \
    4, \
    {  { "seq", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_mission_set_current_t, seq) }, \
         { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_mission_set_current_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_mission_set_current_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_mission_set_current_t, mission_id) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_MISSION_SET_CURRENT { \
    "MISSION_SET_CURRENT", \
    4, \
    {  { "seq", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_mission_set_current_t, seq) }, \
         { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_mission_set_current_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_mission_set_current_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_mission_set_current_t, mission_id) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_set_current message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_set_current_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint16_t seq)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_SET_CURRENT_LEN];
    _mace_put_uint16_t(buf, 0, seq);
    _mace_put_uint8_t(buf, 2, mission_system);
    _mace_put_uint8_t(buf, 3, mission_creator);
    _mace_put_uint8_t(buf, 4, mission_id);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_SET_CURRENT_LEN);
#else
    mace_mission_set_current_t packet;
    packet.seq = seq;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_SET_CURRENT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_SET_CURRENT;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_MISSION_SET_CURRENT_MIN_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_CRC);
}

/**
 * @brief Pack a mission_set_current message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_set_current_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t mission_system,uint8_t mission_creator,uint8_t mission_id,uint16_t seq)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_SET_CURRENT_LEN];
    _mace_put_uint16_t(buf, 0, seq);
    _mace_put_uint8_t(buf, 2, mission_system);
    _mace_put_uint8_t(buf, 3, mission_creator);
    _mace_put_uint8_t(buf, 4, mission_id);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_SET_CURRENT_LEN);
#else
    mace_mission_set_current_t packet;
    packet.seq = seq;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_SET_CURRENT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_SET_CURRENT;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_MISSION_SET_CURRENT_MIN_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_CRC);
}

/**
 * @brief Encode a mission_set_current struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_set_current C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_set_current_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_mission_set_current_t* mission_set_current)
{
    return mace_msg_mission_set_current_pack(system_id, component_id, msg, mission_set_current->mission_system, mission_set_current->mission_creator, mission_set_current->mission_id, mission_set_current->seq);
}

/**
 * @brief Encode a mission_set_current struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_set_current C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_set_current_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_mission_set_current_t* mission_set_current)
{
    return mace_msg_mission_set_current_pack_chan(system_id, component_id, chan, msg, mission_set_current->mission_system, mission_set_current->mission_creator, mission_set_current->mission_id, mission_set_current->seq);
}

/**
 * @brief Send a mission_set_current message
 * @param chan MAVLink channel to send the message
 *
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param seq Sequence
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_mission_set_current_send(mace_channel_t chan, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint16_t seq)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_SET_CURRENT_LEN];
    _mace_put_uint16_t(buf, 0, seq);
    _mace_put_uint8_t(buf, 2, mission_system);
    _mace_put_uint8_t(buf, 3, mission_creator);
    _mace_put_uint8_t(buf, 4, mission_id);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_SET_CURRENT, buf, MACE_MSG_ID_MISSION_SET_CURRENT_MIN_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_CRC);
#else
    mace_mission_set_current_t packet;
    packet.seq = seq;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_SET_CURRENT, (const char *)&packet, MACE_MSG_ID_MISSION_SET_CURRENT_MIN_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_CRC);
#endif
}

/**
 * @brief Send a mission_set_current message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_mission_set_current_send_struct(mace_channel_t chan, const mace_mission_set_current_t* mission_set_current)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_mission_set_current_send(chan, mission_set_current->mission_system, mission_set_current->mission_creator, mission_set_current->mission_id, mission_set_current->seq);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_SET_CURRENT, (const char *)mission_set_current, MACE_MSG_ID_MISSION_SET_CURRENT_MIN_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_CRC);
#endif
}

#if MACE_MSG_ID_MISSION_SET_CURRENT_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_mission_set_current_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint16_t seq)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint16_t(buf, 0, seq);
    _mace_put_uint8_t(buf, 2, mission_system);
    _mace_put_uint8_t(buf, 3, mission_creator);
    _mace_put_uint8_t(buf, 4, mission_id);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_SET_CURRENT, buf, MACE_MSG_ID_MISSION_SET_CURRENT_MIN_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_CRC);
#else
    mace_mission_set_current_t *packet = (mace_mission_set_current_t *)msgbuf;
    packet->seq = seq;
    packet->mission_system = mission_system;
    packet->mission_creator = mission_creator;
    packet->mission_id = mission_id;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_SET_CURRENT, (const char *)packet, MACE_MSG_ID_MISSION_SET_CURRENT_MIN_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_LEN, MACE_MSG_ID_MISSION_SET_CURRENT_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_SET_CURRENT UNPACKING


/**
 * @brief Get field mission_system from mission_set_current message
 *
 * @return Mission System ID
 */
static inline uint8_t mace_msg_mission_set_current_get_mission_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_creator from mission_set_current message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_mission_set_current_get_mission_creator(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field mission_id from mission_set_current message
 *
 * @return Mission ID
 */
static inline uint8_t mace_msg_mission_set_current_get_mission_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field seq from mission_set_current message
 *
 * @return Sequence
 */
static inline uint16_t mace_msg_mission_set_current_get_seq(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a mission_set_current message into a struct
 *
 * @param msg The message to decode
 * @param mission_set_current C-struct to decode the message contents into
 */
static inline void mace_msg_mission_set_current_decode(const mace_message_t* msg, mace_mission_set_current_t* mission_set_current)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mission_set_current->seq = mace_msg_mission_set_current_get_seq(msg);
    mission_set_current->mission_system = mace_msg_mission_set_current_get_mission_system(msg);
    mission_set_current->mission_creator = mace_msg_mission_set_current_get_mission_creator(msg);
    mission_set_current->mission_id = mace_msg_mission_set_current_get_mission_id(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_MISSION_SET_CURRENT_LEN? msg->len : MACE_MSG_ID_MISSION_SET_CURRENT_LEN;
        memset(mission_set_current, 0, MACE_MSG_ID_MISSION_SET_CURRENT_LEN);
    memcpy(mission_set_current, _MACE_PAYLOAD(msg), len);
#endif
}
