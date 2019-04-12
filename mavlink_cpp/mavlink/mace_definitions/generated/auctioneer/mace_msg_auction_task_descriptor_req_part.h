#pragma once
// MESSAGE AUCTION_TASK_DESCRIPTOR_REQ_PART PACKING

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART 10012

MACEPACKED(
typedef struct __mace_auction_task_descriptor_req_part_t {
 uint64_t sentFrom; /*< ID of agent the task is being requested from*/
 uint64_t creatorID; /*< Task creator ID*/
 uint8_t taskID; /*< Creator local task ID*/
 int8_t seqNum; /*< Sequence number ACKed*/
}) mace_auction_task_descriptor_req_part_t;

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN 18
#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_MIN_LEN 18
#define MACE_MSG_ID_10012_LEN 18
#define MACE_MSG_ID_10012_MIN_LEN 18

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_CRC 180
#define MACE_MSG_ID_10012_CRC 180



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_REQ_PART { \
    10012, \
    "AUCTION_TASK_DESCRIPTOR_REQ_PART", \
    4, \
    {  { "sentFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_task_descriptor_req_part_t, sentFrom) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_task_descriptor_req_part_t, creatorID) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 16, offsetof(mace_auction_task_descriptor_req_part_t, taskID) }, \
         { "seqNum", NULL, MACE_TYPE_INT8_T, 0, 17, offsetof(mace_auction_task_descriptor_req_part_t, seqNum) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_REQ_PART { \
    "AUCTION_TASK_DESCRIPTOR_REQ_PART", \
    4, \
    {  { "sentFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_task_descriptor_req_part_t, sentFrom) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_task_descriptor_req_part_t, creatorID) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 16, offsetof(mace_auction_task_descriptor_req_part_t, taskID) }, \
         { "seqNum", NULL, MACE_TYPE_INT8_T, 0, 17, offsetof(mace_auction_task_descriptor_req_part_t, seqNum) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_task_descriptor_req_part message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sentFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param seqNum Sequence number ACKed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_task_descriptor_req_part_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, int8_t seqNum)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_uint8_t(buf, 16, taskID);
    _mace_put_int8_t(buf, 17, seqNum);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN);
#else
    mace_auction_task_descriptor_req_part_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.taskID = taskID;
    packet.seqNum = seqNum;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_CRC);
}

/**
 * @brief Pack a auction_task_descriptor_req_part message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sentFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param seqNum Sequence number ACKed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_task_descriptor_req_part_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t sentFrom,uint64_t creatorID,uint8_t taskID,int8_t seqNum)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_uint8_t(buf, 16, taskID);
    _mace_put_int8_t(buf, 17, seqNum);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN);
#else
    mace_auction_task_descriptor_req_part_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.taskID = taskID;
    packet.seqNum = seqNum;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_CRC);
}

/**
 * @brief Encode a auction_task_descriptor_req_part struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_task_descriptor_req_part C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_task_descriptor_req_part_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_task_descriptor_req_part_t* auction_task_descriptor_req_part)
{
    return mace_msg_auction_task_descriptor_req_part_pack(system_id, component_id, msg, auction_task_descriptor_req_part->sentFrom, auction_task_descriptor_req_part->creatorID, auction_task_descriptor_req_part->taskID, auction_task_descriptor_req_part->seqNum);
}

/**
 * @brief Encode a auction_task_descriptor_req_part struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_task_descriptor_req_part C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_task_descriptor_req_part_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_task_descriptor_req_part_t* auction_task_descriptor_req_part)
{
    return mace_msg_auction_task_descriptor_req_part_pack_chan(system_id, component_id, chan, msg, auction_task_descriptor_req_part->sentFrom, auction_task_descriptor_req_part->creatorID, auction_task_descriptor_req_part->taskID, auction_task_descriptor_req_part->seqNum);
}

/**
 * @brief Send a auction_task_descriptor_req_part message
 * @param chan MAVLink channel to send the message
 *
 * @param sentFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param seqNum Sequence number ACKed
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_task_descriptor_req_part_send(mace_channel_t chan, uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, int8_t seqNum)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_uint8_t(buf, 16, taskID);
    _mace_put_int8_t(buf, 17, seqNum);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART, buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_CRC);
#else
    mace_auction_task_descriptor_req_part_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.taskID = taskID;
    packet.seqNum = seqNum;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART, (const char *)&packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_CRC);
#endif
}

/**
 * @brief Send a auction_task_descriptor_req_part message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_task_descriptor_req_part_send_struct(mace_channel_t chan, const mace_auction_task_descriptor_req_part_t* auction_task_descriptor_req_part)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_task_descriptor_req_part_send(chan, auction_task_descriptor_req_part->sentFrom, auction_task_descriptor_req_part->creatorID, auction_task_descriptor_req_part->taskID, auction_task_descriptor_req_part->seqNum);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART, (const char *)auction_task_descriptor_req_part, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_task_descriptor_req_part_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, int8_t seqNum)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_uint8_t(buf, 16, taskID);
    _mace_put_int8_t(buf, 17, seqNum);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART, buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_CRC);
#else
    mace_auction_task_descriptor_req_part_t *packet = (mace_auction_task_descriptor_req_part_t *)msgbuf;
    packet->sentFrom = sentFrom;
    packet->creatorID = creatorID;
    packet->taskID = taskID;
    packet->seqNum = seqNum;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART, (const char *)packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_TASK_DESCRIPTOR_REQ_PART UNPACKING


/**
 * @brief Get field sentFrom from auction_task_descriptor_req_part message
 *
 * @return ID of agent the task is being requested from
 */
static inline uint64_t mace_msg_auction_task_descriptor_req_part_get_sentFrom(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field creatorID from auction_task_descriptor_req_part message
 *
 * @return Task creator ID
 */
static inline uint64_t mace_msg_auction_task_descriptor_req_part_get_creatorID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field taskID from auction_task_descriptor_req_part message
 *
 * @return Creator local task ID
 */
static inline uint8_t mace_msg_auction_task_descriptor_req_part_get_taskID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field seqNum from auction_task_descriptor_req_part message
 *
 * @return Sequence number ACKed
 */
static inline int8_t mace_msg_auction_task_descriptor_req_part_get_seqNum(const mace_message_t* msg)
{
    return _MACE_RETURN_int8_t(msg,  17);
}

/**
 * @brief Decode a auction_task_descriptor_req_part message into a struct
 *
 * @param msg The message to decode
 * @param auction_task_descriptor_req_part C-struct to decode the message contents into
 */
static inline void mace_msg_auction_task_descriptor_req_part_decode(const mace_message_t* msg, mace_auction_task_descriptor_req_part_t* auction_task_descriptor_req_part)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_task_descriptor_req_part->sentFrom = mace_msg_auction_task_descriptor_req_part_get_sentFrom(msg);
    auction_task_descriptor_req_part->creatorID = mace_msg_auction_task_descriptor_req_part_get_creatorID(msg);
    auction_task_descriptor_req_part->taskID = mace_msg_auction_task_descriptor_req_part_get_taskID(msg);
    auction_task_descriptor_req_part->seqNum = mace_msg_auction_task_descriptor_req_part_get_seqNum(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN? msg->len : MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN;
        memset(auction_task_descriptor_req_part, 0, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_LEN);
    memcpy(auction_task_descriptor_req_part, _MACE_PAYLOAD(msg), len);
#endif
}
