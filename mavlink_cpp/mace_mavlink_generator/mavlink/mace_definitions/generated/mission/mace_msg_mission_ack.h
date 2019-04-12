#pragma once
// MESSAGE MISSION_ACK PACKING

#define MACE_MSG_ID_MISSION_ACK 102

MACEPACKED(
typedef struct __mace_mission_ack_t {
 uint8_t mission_system; /*< Mission System ID*/
 uint8_t mission_creator; /*< Creator ID*/
 uint8_t mission_id; /*< Mission ID*/
 uint8_t mission_type; /*< Mission type, see MISSION_TYPE*/
 uint8_t prev_mission_state; /*< The previous mission state, allowing us to recognize the original key. See MISSION_STATE*/
 uint8_t mission_result; /*< The acknowledgement result associated, see MAV_MISSION_RESULT*/
 uint8_t cur_mission_state; /*< The potential new mission state, see MISSION_STATE*/
}) mace_mission_ack_t;

#define MACE_MSG_ID_MISSION_ACK_LEN 7
#define MACE_MSG_ID_MISSION_ACK_MIN_LEN 7
#define MACE_MSG_ID_102_LEN 7
#define MACE_MSG_ID_102_MIN_LEN 7

#define MACE_MSG_ID_MISSION_ACK_CRC 87
#define MACE_MSG_ID_102_CRC 87



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_MISSION_ACK { \
    102, \
    "MISSION_ACK", \
    7, \
    {  { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_mission_ack_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_mission_ack_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_mission_ack_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_mission_ack_t, mission_type) }, \
         { "prev_mission_state", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_mission_ack_t, prev_mission_state) }, \
         { "mission_result", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_mission_ack_t, mission_result) }, \
         { "cur_mission_state", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_mission_ack_t, cur_mission_state) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_MISSION_ACK { \
    "MISSION_ACK", \
    7, \
    {  { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_mission_ack_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_mission_ack_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_mission_ack_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_mission_ack_t, mission_type) }, \
         { "prev_mission_state", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_mission_ack_t, prev_mission_state) }, \
         { "mission_result", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_mission_ack_t, mission_result) }, \
         { "cur_mission_state", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_mission_ack_t, cur_mission_state) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param prev_mission_state The previous mission state, allowing us to recognize the original key. See MISSION_STATE
 * @param mission_result The acknowledgement result associated, see MAV_MISSION_RESULT
 * @param cur_mission_state The potential new mission state, see MISSION_STATE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_ack_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t prev_mission_state, uint8_t mission_result, uint8_t cur_mission_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_ACK_LEN];
    _mace_put_uint8_t(buf, 0, mission_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);
    _mace_put_uint8_t(buf, 4, prev_mission_state);
    _mace_put_uint8_t(buf, 5, mission_result);
    _mace_put_uint8_t(buf, 6, cur_mission_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_ACK_LEN);
#else
    mace_mission_ack_t packet;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.prev_mission_state = prev_mission_state;
    packet.mission_result = mission_result;
    packet.cur_mission_state = cur_mission_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_ACK_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_ACK;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_MISSION_ACK_MIN_LEN, MACE_MSG_ID_MISSION_ACK_LEN, MACE_MSG_ID_MISSION_ACK_CRC);
}

/**
 * @brief Pack a mission_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param prev_mission_state The previous mission state, allowing us to recognize the original key. See MISSION_STATE
 * @param mission_result The acknowledgement result associated, see MAV_MISSION_RESULT
 * @param cur_mission_state The potential new mission state, see MISSION_STATE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t mission_system,uint8_t mission_creator,uint8_t mission_id,uint8_t mission_type,uint8_t prev_mission_state,uint8_t mission_result,uint8_t cur_mission_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_ACK_LEN];
    _mace_put_uint8_t(buf, 0, mission_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);
    _mace_put_uint8_t(buf, 4, prev_mission_state);
    _mace_put_uint8_t(buf, 5, mission_result);
    _mace_put_uint8_t(buf, 6, cur_mission_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_ACK_LEN);
#else
    mace_mission_ack_t packet;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.prev_mission_state = prev_mission_state;
    packet.mission_result = mission_result;
    packet.cur_mission_state = cur_mission_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_ACK_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_ACK;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_MISSION_ACK_MIN_LEN, MACE_MSG_ID_MISSION_ACK_LEN, MACE_MSG_ID_MISSION_ACK_CRC);
}

/**
 * @brief Encode a mission_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_ack C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_ack_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_mission_ack_t* mission_ack)
{
    return mace_msg_mission_ack_pack(system_id, component_id, msg, mission_ack->mission_system, mission_ack->mission_creator, mission_ack->mission_id, mission_ack->mission_type, mission_ack->prev_mission_state, mission_ack->mission_result, mission_ack->cur_mission_state);
}

/**
 * @brief Encode a mission_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_ack C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_mission_ack_t* mission_ack)
{
    return mace_msg_mission_ack_pack_chan(system_id, component_id, chan, msg, mission_ack->mission_system, mission_ack->mission_creator, mission_ack->mission_id, mission_ack->mission_type, mission_ack->prev_mission_state, mission_ack->mission_result, mission_ack->cur_mission_state);
}

/**
 * @brief Send a mission_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param prev_mission_state The previous mission state, allowing us to recognize the original key. See MISSION_STATE
 * @param mission_result The acknowledgement result associated, see MAV_MISSION_RESULT
 * @param cur_mission_state The potential new mission state, see MISSION_STATE
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_mission_ack_send(mace_channel_t chan, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t prev_mission_state, uint8_t mission_result, uint8_t cur_mission_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_ACK_LEN];
    _mace_put_uint8_t(buf, 0, mission_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);
    _mace_put_uint8_t(buf, 4, prev_mission_state);
    _mace_put_uint8_t(buf, 5, mission_result);
    _mace_put_uint8_t(buf, 6, cur_mission_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_ACK, buf, MACE_MSG_ID_MISSION_ACK_MIN_LEN, MACE_MSG_ID_MISSION_ACK_LEN, MACE_MSG_ID_MISSION_ACK_CRC);
#else
    mace_mission_ack_t packet;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.prev_mission_state = prev_mission_state;
    packet.mission_result = mission_result;
    packet.cur_mission_state = cur_mission_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_ACK, (const char *)&packet, MACE_MSG_ID_MISSION_ACK_MIN_LEN, MACE_MSG_ID_MISSION_ACK_LEN, MACE_MSG_ID_MISSION_ACK_CRC);
#endif
}

/**
 * @brief Send a mission_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_mission_ack_send_struct(mace_channel_t chan, const mace_mission_ack_t* mission_ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_mission_ack_send(chan, mission_ack->mission_system, mission_ack->mission_creator, mission_ack->mission_id, mission_ack->mission_type, mission_ack->prev_mission_state, mission_ack->mission_result, mission_ack->cur_mission_state);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_ACK, (const char *)mission_ack, MACE_MSG_ID_MISSION_ACK_MIN_LEN, MACE_MSG_ID_MISSION_ACK_LEN, MACE_MSG_ID_MISSION_ACK_CRC);
#endif
}

#if MACE_MSG_ID_MISSION_ACK_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_mission_ack_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t prev_mission_state, uint8_t mission_result, uint8_t cur_mission_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, mission_system);
    _mace_put_uint8_t(buf, 1, mission_creator);
    _mace_put_uint8_t(buf, 2, mission_id);
    _mace_put_uint8_t(buf, 3, mission_type);
    _mace_put_uint8_t(buf, 4, prev_mission_state);
    _mace_put_uint8_t(buf, 5, mission_result);
    _mace_put_uint8_t(buf, 6, cur_mission_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_ACK, buf, MACE_MSG_ID_MISSION_ACK_MIN_LEN, MACE_MSG_ID_MISSION_ACK_LEN, MACE_MSG_ID_MISSION_ACK_CRC);
#else
    mace_mission_ack_t *packet = (mace_mission_ack_t *)msgbuf;
    packet->mission_system = mission_system;
    packet->mission_creator = mission_creator;
    packet->mission_id = mission_id;
    packet->mission_type = mission_type;
    packet->prev_mission_state = prev_mission_state;
    packet->mission_result = mission_result;
    packet->cur_mission_state = cur_mission_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_ACK, (const char *)packet, MACE_MSG_ID_MISSION_ACK_MIN_LEN, MACE_MSG_ID_MISSION_ACK_LEN, MACE_MSG_ID_MISSION_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_ACK UNPACKING


/**
 * @brief Get field mission_system from mission_ack message
 *
 * @return Mission System ID
 */
static inline uint8_t mace_msg_mission_ack_get_mission_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mission_creator from mission_ack message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_mission_ack_get_mission_creator(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field mission_id from mission_ack message
 *
 * @return Mission ID
 */
static inline uint8_t mace_msg_mission_ack_get_mission_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_type from mission_ack message
 *
 * @return Mission type, see MISSION_TYPE
 */
static inline uint8_t mace_msg_mission_ack_get_mission_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field prev_mission_state from mission_ack message
 *
 * @return The previous mission state, allowing us to recognize the original key. See MISSION_STATE
 */
static inline uint8_t mace_msg_mission_ack_get_prev_mission_state(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field mission_result from mission_ack message
 *
 * @return The acknowledgement result associated, see MAV_MISSION_RESULT
 */
static inline uint8_t mace_msg_mission_ack_get_mission_result(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field cur_mission_state from mission_ack message
 *
 * @return The potential new mission state, see MISSION_STATE
 */
static inline uint8_t mace_msg_mission_ack_get_cur_mission_state(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a mission_ack message into a struct
 *
 * @param msg The message to decode
 * @param mission_ack C-struct to decode the message contents into
 */
static inline void mace_msg_mission_ack_decode(const mace_message_t* msg, mace_mission_ack_t* mission_ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mission_ack->mission_system = mace_msg_mission_ack_get_mission_system(msg);
    mission_ack->mission_creator = mace_msg_mission_ack_get_mission_creator(msg);
    mission_ack->mission_id = mace_msg_mission_ack_get_mission_id(msg);
    mission_ack->mission_type = mace_msg_mission_ack_get_mission_type(msg);
    mission_ack->prev_mission_state = mace_msg_mission_ack_get_prev_mission_state(msg);
    mission_ack->mission_result = mace_msg_mission_ack_get_mission_result(msg);
    mission_ack->cur_mission_state = mace_msg_mission_ack_get_cur_mission_state(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_MISSION_ACK_LEN? msg->len : MACE_MSG_ID_MISSION_ACK_LEN;
        memset(mission_ack, 0, MACE_MSG_ID_MISSION_ACK_LEN);
    memcpy(mission_ack, _MACE_PAYLOAD(msg), len);
#endif
}
