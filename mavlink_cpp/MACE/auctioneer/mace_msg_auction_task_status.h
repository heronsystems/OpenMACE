#pragma once
// MESSAGE AUCTION_TASK_STATUS PACKING

#define MACE_MSG_ID_AUCTION_TASK_STATUS 10009

MACEPACKED(
typedef struct __mace_auction_task_status_t {
 uint64_t creatorID; /*< Task creator ID*/
 uint64_t agentID; /*< Agent which is setting the status*/
 uint8_t taskID; /*< Creator local task ID*/
 uint8_t type; /*< Status update type*/
 uint8_t data[64]; /*< Optional data dependent on type, up to 64 bytes. Set unused bytes to 0, to allow MAVLink v2 to truncate the message. Must be manually set to network byte order*/
}) mace_auction_task_status_t;

#define MACE_MSG_ID_AUCTION_TASK_STATUS_LEN 82
#define MACE_MSG_ID_AUCTION_TASK_STATUS_MIN_LEN 82
#define MACE_MSG_ID_10009_LEN 82
#define MACE_MSG_ID_10009_MIN_LEN 82

#define MACE_MSG_ID_AUCTION_TASK_STATUS_CRC 91
#define MACE_MSG_ID_10009_CRC 91

#define MACE_MSG_AUCTION_TASK_STATUS_FIELD_DATA_LEN 64

#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_TASK_STATUS { \
    10009, \
    "AUCTION_TASK_STATUS", \
    5, \
    {  { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_task_status_t, creatorID) }, \
         { "agentID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_task_status_t, agentID) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 16, offsetof(mace_auction_task_status_t, taskID) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 17, offsetof(mace_auction_task_status_t, type) }, \
         { "data", NULL, MACE_TYPE_UINT8_T, 64, 18, offsetof(mace_auction_task_status_t, data) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_TASK_STATUS { \
    "AUCTION_TASK_STATUS", \
    5, \
    {  { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_task_status_t, creatorID) }, \
         { "agentID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_task_status_t, agentID) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 16, offsetof(mace_auction_task_status_t, taskID) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 17, offsetof(mace_auction_task_status_t, type) }, \
         { "data", NULL, MACE_TYPE_UINT8_T, 64, 18, offsetof(mace_auction_task_status_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_task_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param type Status update type
 * @param agentID Agent which is setting the status
 * @param data Optional data dependent on type, up to 64 bytes. Set unused bytes to 0, to allow MAVLink v2 to truncate the message. Must be manually set to network byte order
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_task_status_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t creatorID, uint8_t taskID, uint8_t type, uint64_t agentID, const uint8_t *data)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_STATUS_LEN];
    _mace_put_uint64_t(buf, 0, creatorID);
    _mace_put_uint64_t(buf, 8, agentID);
    _mace_put_uint8_t(buf, 16, taskID);
    _mace_put_uint8_t(buf, 17, type);
    _mace_put_uint8_t_array(buf, 18, data, 64);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_TASK_STATUS_LEN);
#else
    mace_auction_task_status_t packet;
    packet.creatorID = creatorID;
    packet.agentID = agentID;
    packet.taskID = taskID;
    packet.type = type;
    mace_array_memcpy(packet.data, data, sizeof(uint8_t)*64);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_TASK_STATUS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_TASK_STATUS;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_TASK_STATUS_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_CRC);
}

/**
 * @brief Pack a auction_task_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param type Status update type
 * @param agentID Agent which is setting the status
 * @param data Optional data dependent on type, up to 64 bytes. Set unused bytes to 0, to allow MAVLink v2 to truncate the message. Must be manually set to network byte order
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_task_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t creatorID,uint8_t taskID,uint8_t type,uint64_t agentID,const uint8_t *data)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_STATUS_LEN];
    _mace_put_uint64_t(buf, 0, creatorID);
    _mace_put_uint64_t(buf, 8, agentID);
    _mace_put_uint8_t(buf, 16, taskID);
    _mace_put_uint8_t(buf, 17, type);
    _mace_put_uint8_t_array(buf, 18, data, 64);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_TASK_STATUS_LEN);
#else
    mace_auction_task_status_t packet;
    packet.creatorID = creatorID;
    packet.agentID = agentID;
    packet.taskID = taskID;
    packet.type = type;
    mace_array_memcpy(packet.data, data, sizeof(uint8_t)*64);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_TASK_STATUS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_TASK_STATUS;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_TASK_STATUS_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_CRC);
}

/**
 * @brief Encode a auction_task_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_task_status C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_task_status_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_task_status_t* auction_task_status)
{
    return mace_msg_auction_task_status_pack(system_id, component_id, msg, auction_task_status->creatorID, auction_task_status->taskID, auction_task_status->type, auction_task_status->agentID, auction_task_status->data);
}

/**
 * @brief Encode a auction_task_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_task_status C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_task_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_task_status_t* auction_task_status)
{
    return mace_msg_auction_task_status_pack_chan(system_id, component_id, chan, msg, auction_task_status->creatorID, auction_task_status->taskID, auction_task_status->type, auction_task_status->agentID, auction_task_status->data);
}

/**
 * @brief Send a auction_task_status message
 * @param chan MAVLink channel to send the message
 *
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param type Status update type
 * @param agentID Agent which is setting the status
 * @param data Optional data dependent on type, up to 64 bytes. Set unused bytes to 0, to allow MAVLink v2 to truncate the message. Must be manually set to network byte order
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_task_status_send(mace_channel_t chan, uint64_t creatorID, uint8_t taskID, uint8_t type, uint64_t agentID, const uint8_t *data)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_STATUS_LEN];
    _mace_put_uint64_t(buf, 0, creatorID);
    _mace_put_uint64_t(buf, 8, agentID);
    _mace_put_uint8_t(buf, 16, taskID);
    _mace_put_uint8_t(buf, 17, type);
    _mace_put_uint8_t_array(buf, 18, data, 64);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_STATUS, buf, MACE_MSG_ID_AUCTION_TASK_STATUS_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_CRC);
#else
    mace_auction_task_status_t packet;
    packet.creatorID = creatorID;
    packet.agentID = agentID;
    packet.taskID = taskID;
    packet.type = type;
    mace_array_memcpy(packet.data, data, sizeof(uint8_t)*64);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_STATUS, (const char *)&packet, MACE_MSG_ID_AUCTION_TASK_STATUS_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_CRC);
#endif
}

/**
 * @brief Send a auction_task_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_task_status_send_struct(mace_channel_t chan, const mace_auction_task_status_t* auction_task_status)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_task_status_send(chan, auction_task_status->creatorID, auction_task_status->taskID, auction_task_status->type, auction_task_status->agentID, auction_task_status->data);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_STATUS, (const char *)auction_task_status, MACE_MSG_ID_AUCTION_TASK_STATUS_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_TASK_STATUS_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_task_status_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t creatorID, uint8_t taskID, uint8_t type, uint64_t agentID, const uint8_t *data)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, creatorID);
    _mace_put_uint64_t(buf, 8, agentID);
    _mace_put_uint8_t(buf, 16, taskID);
    _mace_put_uint8_t(buf, 17, type);
    _mace_put_uint8_t_array(buf, 18, data, 64);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_STATUS, buf, MACE_MSG_ID_AUCTION_TASK_STATUS_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_CRC);
#else
    mace_auction_task_status_t *packet = (mace_auction_task_status_t *)msgbuf;
    packet->creatorID = creatorID;
    packet->agentID = agentID;
    packet->taskID = taskID;
    packet->type = type;
    mace_array_memcpy(packet->data, data, sizeof(uint8_t)*64);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_STATUS, (const char *)packet, MACE_MSG_ID_AUCTION_TASK_STATUS_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_LEN, MACE_MSG_ID_AUCTION_TASK_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_TASK_STATUS UNPACKING


/**
 * @brief Get field creatorID from auction_task_status message
 *
 * @return Task creator ID
 */
static inline uint64_t mace_msg_auction_task_status_get_creatorID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field taskID from auction_task_status message
 *
 * @return Creator local task ID
 */
static inline uint8_t mace_msg_auction_task_status_get_taskID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field type from auction_task_status message
 *
 * @return Status update type
 */
static inline uint8_t mace_msg_auction_task_status_get_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field agentID from auction_task_status message
 *
 * @return Agent which is setting the status
 */
static inline uint64_t mace_msg_auction_task_status_get_agentID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field data from auction_task_status message
 *
 * @return Optional data dependent on type, up to 64 bytes. Set unused bytes to 0, to allow MAVLink v2 to truncate the message. Must be manually set to network byte order
 */
static inline uint16_t mace_msg_auction_task_status_get_data(const mace_message_t* msg, uint8_t *data)
{
    return _MACE_RETURN_uint8_t_array(msg, data, 64,  18);
}

/**
 * @brief Decode a auction_task_status message into a struct
 *
 * @param msg The message to decode
 * @param auction_task_status C-struct to decode the message contents into
 */
static inline void mace_msg_auction_task_status_decode(const mace_message_t* msg, mace_auction_task_status_t* auction_task_status)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_task_status->creatorID = mace_msg_auction_task_status_get_creatorID(msg);
    auction_task_status->agentID = mace_msg_auction_task_status_get_agentID(msg);
    auction_task_status->taskID = mace_msg_auction_task_status_get_taskID(msg);
    auction_task_status->type = mace_msg_auction_task_status_get_type(msg);
    mace_msg_auction_task_status_get_data(msg, auction_task_status->data);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_TASK_STATUS_LEN? msg->len : MACE_MSG_ID_AUCTION_TASK_STATUS_LEN;
        memset(auction_task_status, 0, MACE_MSG_ID_AUCTION_TASK_STATUS_LEN);
    memcpy(auction_task_status, _MACE_PAYLOAD(msg), len);
#endif
}
