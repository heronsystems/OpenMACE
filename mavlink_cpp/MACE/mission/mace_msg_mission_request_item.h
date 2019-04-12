#pragma once
// MESSAGE MISSION_REQUEST_ITEM PACKING

#define MACE_MSG_ID_MISSION_REQUEST_ITEM 106

MACEPACKED(
typedef struct __mace_mission_request_item_t {
 uint16_t seq; /*< Sequence*/
 uint8_t target_system; /*< System ID*/
 uint8_t mission_system; /*< Mission System ID*/
 uint8_t mission_creator; /*< Creator ID*/
 uint8_t mission_id; /*< Mission ID*/
 uint8_t mission_type; /*< Mission type, see MISSION_TYPE*/
 uint8_t mission_state; /*< The mission state, see MISSION_STATE*/
}) mace_mission_request_item_t;

#define MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN 8
#define MACE_MSG_ID_MISSION_REQUEST_ITEM_MIN_LEN 8
#define MACE_MSG_ID_106_LEN 8
#define MACE_MSG_ID_106_MIN_LEN 8

#define MACE_MSG_ID_MISSION_REQUEST_ITEM_CRC 3
#define MACE_MSG_ID_106_CRC 3



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_MISSION_REQUEST_ITEM { \
    106, \
    "MISSION_REQUEST_ITEM", \
    7, \
    {  { "seq", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_mission_request_item_t, seq) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_mission_request_item_t, target_system) }, \
         { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_mission_request_item_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_mission_request_item_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_mission_request_item_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_mission_request_item_t, mission_type) }, \
         { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 7, offsetof(mace_mission_request_item_t, mission_state) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_MISSION_REQUEST_ITEM { \
    "MISSION_REQUEST_ITEM", \
    7, \
    {  { "seq", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_mission_request_item_t, seq) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_mission_request_item_t, target_system) }, \
         { "mission_system", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_mission_request_item_t, mission_system) }, \
         { "mission_creator", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_mission_request_item_t, mission_creator) }, \
         { "mission_id", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_mission_request_item_t, mission_id) }, \
         { "mission_type", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_mission_request_item_t, mission_type) }, \
         { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 7, offsetof(mace_mission_request_item_t, mission_state) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_request_item message
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
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_request_item_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state, uint16_t seq)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN];
    _mace_put_uint16_t(buf, 0, seq);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, mission_system);
    _mace_put_uint8_t(buf, 4, mission_creator);
    _mace_put_uint8_t(buf, 5, mission_id);
    _mace_put_uint8_t(buf, 6, mission_type);
    _mace_put_uint8_t(buf, 7, mission_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN);
#else
    mace_mission_request_item_t packet;
    packet.seq = seq;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_REQUEST_ITEM;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_MISSION_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_CRC);
}

/**
 * @brief Pack a mission_request_item message on a channel
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
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_request_item_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t mission_system,uint8_t mission_creator,uint8_t mission_id,uint8_t mission_type,uint8_t mission_state,uint16_t seq)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN];
    _mace_put_uint16_t(buf, 0, seq);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, mission_system);
    _mace_put_uint8_t(buf, 4, mission_creator);
    _mace_put_uint8_t(buf, 5, mission_id);
    _mace_put_uint8_t(buf, 6, mission_type);
    _mace_put_uint8_t(buf, 7, mission_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN);
#else
    mace_mission_request_item_t packet;
    packet.seq = seq;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_REQUEST_ITEM;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_MISSION_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_CRC);
}

/**
 * @brief Encode a mission_request_item struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_item C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_request_item_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_mission_request_item_t* mission_request_item)
{
    return mace_msg_mission_request_item_pack(system_id, component_id, msg, mission_request_item->target_system, mission_request_item->mission_system, mission_request_item->mission_creator, mission_request_item->mission_id, mission_request_item->mission_type, mission_request_item->mission_state, mission_request_item->seq);
}

/**
 * @brief Encode a mission_request_item struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_item C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_request_item_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_mission_request_item_t* mission_request_item)
{
    return mace_msg_mission_request_item_pack_chan(system_id, component_id, chan, msg, mission_request_item->target_system, mission_request_item->mission_system, mission_request_item->mission_creator, mission_request_item->mission_id, mission_request_item->mission_type, mission_request_item->mission_state, mission_request_item->seq);
}

/**
 * @brief Send a mission_request_item message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param mission_system Mission System ID
 * @param mission_creator Creator ID
 * @param mission_id Mission ID
 * @param mission_type Mission type, see MISSION_TYPE
 * @param mission_state The mission state, see MISSION_STATE
 * @param seq Sequence
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_mission_request_item_send(mace_channel_t chan, uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state, uint16_t seq)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN];
    _mace_put_uint16_t(buf, 0, seq);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, mission_system);
    _mace_put_uint8_t(buf, 4, mission_creator);
    _mace_put_uint8_t(buf, 5, mission_id);
    _mace_put_uint8_t(buf, 6, mission_type);
    _mace_put_uint8_t(buf, 7, mission_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_ITEM, buf, MACE_MSG_ID_MISSION_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_CRC);
#else
    mace_mission_request_item_t packet;
    packet.seq = seq;
    packet.target_system = target_system;
    packet.mission_system = mission_system;
    packet.mission_creator = mission_creator;
    packet.mission_id = mission_id;
    packet.mission_type = mission_type;
    packet.mission_state = mission_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_ITEM, (const char *)&packet, MACE_MSG_ID_MISSION_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_CRC);
#endif
}

/**
 * @brief Send a mission_request_item message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_mission_request_item_send_struct(mace_channel_t chan, const mace_mission_request_item_t* mission_request_item)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_mission_request_item_send(chan, mission_request_item->target_system, mission_request_item->mission_system, mission_request_item->mission_creator, mission_request_item->mission_id, mission_request_item->mission_type, mission_request_item->mission_state, mission_request_item->seq);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_ITEM, (const char *)mission_request_item, MACE_MSG_ID_MISSION_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_CRC);
#endif
}

#if MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_mission_request_item_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t mission_system, uint8_t mission_creator, uint8_t mission_id, uint8_t mission_type, uint8_t mission_state, uint16_t seq)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint16_t(buf, 0, seq);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, mission_system);
    _mace_put_uint8_t(buf, 4, mission_creator);
    _mace_put_uint8_t(buf, 5, mission_id);
    _mace_put_uint8_t(buf, 6, mission_type);
    _mace_put_uint8_t(buf, 7, mission_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_ITEM, buf, MACE_MSG_ID_MISSION_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_CRC);
#else
    mace_mission_request_item_t *packet = (mace_mission_request_item_t *)msgbuf;
    packet->seq = seq;
    packet->target_system = target_system;
    packet->mission_system = mission_system;
    packet->mission_creator = mission_creator;
    packet->mission_id = mission_id;
    packet->mission_type = mission_type;
    packet->mission_state = mission_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_ITEM, (const char *)packet, MACE_MSG_ID_MISSION_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN, MACE_MSG_ID_MISSION_REQUEST_ITEM_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_REQUEST_ITEM UNPACKING


/**
 * @brief Get field target_system from mission_request_item message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_mission_request_item_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_system from mission_request_item message
 *
 * @return Mission System ID
 */
static inline uint8_t mace_msg_mission_request_item_get_mission_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field mission_creator from mission_request_item message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_mission_request_item_get_mission_creator(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field mission_id from mission_request_item message
 *
 * @return Mission ID
 */
static inline uint8_t mace_msg_mission_request_item_get_mission_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field mission_type from mission_request_item message
 *
 * @return Mission type, see MISSION_TYPE
 */
static inline uint8_t mace_msg_mission_request_item_get_mission_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field mission_state from mission_request_item message
 *
 * @return The mission state, see MISSION_STATE
 */
static inline uint8_t mace_msg_mission_request_item_get_mission_state(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field seq from mission_request_item message
 *
 * @return Sequence
 */
static inline uint16_t mace_msg_mission_request_item_get_seq(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a mission_request_item message into a struct
 *
 * @param msg The message to decode
 * @param mission_request_item C-struct to decode the message contents into
 */
static inline void mace_msg_mission_request_item_decode(const mace_message_t* msg, mace_mission_request_item_t* mission_request_item)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mission_request_item->seq = mace_msg_mission_request_item_get_seq(msg);
    mission_request_item->target_system = mace_msg_mission_request_item_get_target_system(msg);
    mission_request_item->mission_system = mace_msg_mission_request_item_get_mission_system(msg);
    mission_request_item->mission_creator = mace_msg_mission_request_item_get_mission_creator(msg);
    mission_request_item->mission_id = mace_msg_mission_request_item_get_mission_id(msg);
    mission_request_item->mission_type = mace_msg_mission_request_item_get_mission_type(msg);
    mission_request_item->mission_state = mace_msg_mission_request_item_get_mission_state(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN? msg->len : MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN;
        memset(mission_request_item, 0, MACE_MSG_ID_MISSION_REQUEST_ITEM_LEN);
    memcpy(mission_request_item, _MACE_PAYLOAD(msg), len);
#endif
}
