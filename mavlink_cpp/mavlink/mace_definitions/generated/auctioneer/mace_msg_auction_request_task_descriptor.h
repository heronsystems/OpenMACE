#pragma once
// MESSAGE AUCTION_REQUEST_TASK_DESCRIPTOR PACKING

#define MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR 10007

MACEPACKED(
typedef struct __mace_auction_request_task_descriptor_t {
 uint64_t requestFrom; /*< ID of agent the task is being requested from*/
 uint64_t creatorID; /*< Task creator ID*/
 uint32_t taskID; /*< Creator local task ID*/
}) mace_auction_request_task_descriptor_t;

#define MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN 20
#define MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_MIN_LEN 20
#define MACE_MSG_ID_10007_LEN 20
#define MACE_MSG_ID_10007_MIN_LEN 20

#define MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_CRC 74
#define MACE_MSG_ID_10007_CRC 74



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_REQUEST_TASK_DESCRIPTOR { \
    10007, \
    "AUCTION_REQUEST_TASK_DESCRIPTOR", \
    3, \
    {  { "requestFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_request_task_descriptor_t, requestFrom) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_request_task_descriptor_t, creatorID) }, \
         { "taskID", NULL, MACE_TYPE_UINT32_T, 0, 16, offsetof(mace_auction_request_task_descriptor_t, taskID) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_REQUEST_TASK_DESCRIPTOR { \
    "AUCTION_REQUEST_TASK_DESCRIPTOR", \
    3, \
    {  { "requestFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_request_task_descriptor_t, requestFrom) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_request_task_descriptor_t, creatorID) }, \
         { "taskID", NULL, MACE_TYPE_UINT32_T, 0, 16, offsetof(mace_auction_request_task_descriptor_t, taskID) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_request_task_descriptor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param requestFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_request_task_descriptor_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t requestFrom, uint64_t creatorID, uint32_t taskID)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN];
    _mace_put_uint64_t(buf, 0, requestFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_uint32_t(buf, 16, taskID);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN);
#else
    mace_auction_request_task_descriptor_t packet;
    packet.requestFrom = requestFrom;
    packet.creatorID = creatorID;
    packet.taskID = taskID;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_CRC);
}

/**
 * @brief Pack a auction_request_task_descriptor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param requestFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_request_task_descriptor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t requestFrom,uint64_t creatorID,uint32_t taskID)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN];
    _mace_put_uint64_t(buf, 0, requestFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_uint32_t(buf, 16, taskID);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN);
#else
    mace_auction_request_task_descriptor_t packet;
    packet.requestFrom = requestFrom;
    packet.creatorID = creatorID;
    packet.taskID = taskID;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_CRC);
}

/**
 * @brief Encode a auction_request_task_descriptor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_request_task_descriptor C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_request_task_descriptor_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_request_task_descriptor_t* auction_request_task_descriptor)
{
    return mace_msg_auction_request_task_descriptor_pack(system_id, component_id, msg, auction_request_task_descriptor->requestFrom, auction_request_task_descriptor->creatorID, auction_request_task_descriptor->taskID);
}

/**
 * @brief Encode a auction_request_task_descriptor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_request_task_descriptor C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_request_task_descriptor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_request_task_descriptor_t* auction_request_task_descriptor)
{
    return mace_msg_auction_request_task_descriptor_pack_chan(system_id, component_id, chan, msg, auction_request_task_descriptor->requestFrom, auction_request_task_descriptor->creatorID, auction_request_task_descriptor->taskID);
}

/**
 * @brief Send a auction_request_task_descriptor message
 * @param chan MAVLink channel to send the message
 *
 * @param requestFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_request_task_descriptor_send(mace_channel_t chan, uint64_t requestFrom, uint64_t creatorID, uint32_t taskID)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN];
    _mace_put_uint64_t(buf, 0, requestFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_uint32_t(buf, 16, taskID);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR, buf, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_CRC);
#else
    mace_auction_request_task_descriptor_t packet;
    packet.requestFrom = requestFrom;
    packet.creatorID = creatorID;
    packet.taskID = taskID;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR, (const char *)&packet, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_CRC);
#endif
}

/**
 * @brief Send a auction_request_task_descriptor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_request_task_descriptor_send_struct(mace_channel_t chan, const mace_auction_request_task_descriptor_t* auction_request_task_descriptor)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_request_task_descriptor_send(chan, auction_request_task_descriptor->requestFrom, auction_request_task_descriptor->creatorID, auction_request_task_descriptor->taskID);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR, (const char *)auction_request_task_descriptor, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_request_task_descriptor_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t requestFrom, uint64_t creatorID, uint32_t taskID)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, requestFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_uint32_t(buf, 16, taskID);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR, buf, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_CRC);
#else
    mace_auction_request_task_descriptor_t *packet = (mace_auction_request_task_descriptor_t *)msgbuf;
    packet->requestFrom = requestFrom;
    packet->creatorID = creatorID;
    packet->taskID = taskID;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR, (const char *)packet, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_REQUEST_TASK_DESCRIPTOR UNPACKING


/**
 * @brief Get field requestFrom from auction_request_task_descriptor message
 *
 * @return ID of agent the task is being requested from
 */
static inline uint64_t mace_msg_auction_request_task_descriptor_get_requestFrom(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field creatorID from auction_request_task_descriptor message
 *
 * @return Task creator ID
 */
static inline uint64_t mace_msg_auction_request_task_descriptor_get_creatorID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field taskID from auction_request_task_descriptor message
 *
 * @return Creator local task ID
 */
static inline uint32_t mace_msg_auction_request_task_descriptor_get_taskID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Decode a auction_request_task_descriptor message into a struct
 *
 * @param msg The message to decode
 * @param auction_request_task_descriptor C-struct to decode the message contents into
 */
static inline void mace_msg_auction_request_task_descriptor_decode(const mace_message_t* msg, mace_auction_request_task_descriptor_t* auction_request_task_descriptor)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_request_task_descriptor->requestFrom = mace_msg_auction_request_task_descriptor_get_requestFrom(msg);
    auction_request_task_descriptor->creatorID = mace_msg_auction_request_task_descriptor_get_creatorID(msg);
    auction_request_task_descriptor->taskID = mace_msg_auction_request_task_descriptor_get_taskID(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN? msg->len : MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN;
        memset(auction_request_task_descriptor, 0, MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_LEN);
    memcpy(auction_request_task_descriptor, _MACE_PAYLOAD(msg), len);
#endif
}
