#pragma once
// MESSAGE AUCTION_TASK_DESCRIPTOR PACKING

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR 10009

MACEPACKED(
typedef struct __mace_auction_task_descriptor_t {
 uint64_t sentFrom; /*< ID of agent the descriptor is being requested from*/
 uint64_t creatorID; /*< Task creator ID*/
 double taskGenTime; /*< Task generation time*/
 double reqStart; /*< Required Start*/
 double reqEnd; /*< Required End*/
 uint32_t penalty; /*< Time penalty*/
 uint8_t taskID; /*< Creator local task ID*/
 uint8_t type; /*< Task type*/
 uint8_t numParts; /*< Number of messages to be sent to receive the rest of the descriptor*/
}) mace_auction_task_descriptor_t;

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN 47
#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_MIN_LEN 47
#define MACE_MSG_ID_10009_LEN 47
#define MACE_MSG_ID_10009_MIN_LEN 47

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_CRC 128
#define MACE_MSG_ID_10009_CRC 128



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR { \
    10009, \
    "AUCTION_TASK_DESCRIPTOR", \
    9, \
    {  { "sentFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_task_descriptor_t, sentFrom) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_task_descriptor_t, creatorID) }, \
         { "taskGenTime", NULL, MACE_TYPE_DOUBLE, 0, 16, offsetof(mace_auction_task_descriptor_t, taskGenTime) }, \
         { "reqStart", NULL, MACE_TYPE_DOUBLE, 0, 24, offsetof(mace_auction_task_descriptor_t, reqStart) }, \
         { "reqEnd", NULL, MACE_TYPE_DOUBLE, 0, 32, offsetof(mace_auction_task_descriptor_t, reqEnd) }, \
         { "penalty", NULL, MACE_TYPE_UINT32_T, 0, 40, offsetof(mace_auction_task_descriptor_t, penalty) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 44, offsetof(mace_auction_task_descriptor_t, taskID) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 45, offsetof(mace_auction_task_descriptor_t, type) }, \
         { "numParts", NULL, MACE_TYPE_UINT8_T, 0, 46, offsetof(mace_auction_task_descriptor_t, numParts) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR { \
    "AUCTION_TASK_DESCRIPTOR", \
    9, \
    {  { "sentFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_task_descriptor_t, sentFrom) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_task_descriptor_t, creatorID) }, \
         { "taskGenTime", NULL, MACE_TYPE_DOUBLE, 0, 16, offsetof(mace_auction_task_descriptor_t, taskGenTime) }, \
         { "reqStart", NULL, MACE_TYPE_DOUBLE, 0, 24, offsetof(mace_auction_task_descriptor_t, reqStart) }, \
         { "reqEnd", NULL, MACE_TYPE_DOUBLE, 0, 32, offsetof(mace_auction_task_descriptor_t, reqEnd) }, \
         { "penalty", NULL, MACE_TYPE_UINT32_T, 0, 40, offsetof(mace_auction_task_descriptor_t, penalty) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 44, offsetof(mace_auction_task_descriptor_t, taskID) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 45, offsetof(mace_auction_task_descriptor_t, type) }, \
         { "numParts", NULL, MACE_TYPE_UINT8_T, 0, 46, offsetof(mace_auction_task_descriptor_t, numParts) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_task_descriptor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sentFrom ID of agent the descriptor is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param taskGenTime Task generation time
 * @param type Task type
 * @param penalty Time penalty
 * @param reqStart Required Start
 * @param reqEnd Required End
 * @param numParts Number of messages to be sent to receive the rest of the descriptor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_task_descriptor_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, double taskGenTime, uint8_t type, uint32_t penalty, double reqStart, double reqEnd, uint8_t numParts)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, taskGenTime);
    _mace_put_double(buf, 24, reqStart);
    _mace_put_double(buf, 32, reqEnd);
    _mace_put_uint32_t(buf, 40, penalty);
    _mace_put_uint8_t(buf, 44, taskID);
    _mace_put_uint8_t(buf, 45, type);
    _mace_put_uint8_t(buf, 46, numParts);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN);
#else
    mace_auction_task_descriptor_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.taskGenTime = taskGenTime;
    packet.reqStart = reqStart;
    packet.reqEnd = reqEnd;
    packet.penalty = penalty;
    packet.taskID = taskID;
    packet.type = type;
    packet.numParts = numParts;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_CRC);
}

/**
 * @brief Pack a auction_task_descriptor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sentFrom ID of agent the descriptor is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param taskGenTime Task generation time
 * @param type Task type
 * @param penalty Time penalty
 * @param reqStart Required Start
 * @param reqEnd Required End
 * @param numParts Number of messages to be sent to receive the rest of the descriptor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_task_descriptor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t sentFrom,uint64_t creatorID,uint8_t taskID,double taskGenTime,uint8_t type,uint32_t penalty,double reqStart,double reqEnd,uint8_t numParts)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, taskGenTime);
    _mace_put_double(buf, 24, reqStart);
    _mace_put_double(buf, 32, reqEnd);
    _mace_put_uint32_t(buf, 40, penalty);
    _mace_put_uint8_t(buf, 44, taskID);
    _mace_put_uint8_t(buf, 45, type);
    _mace_put_uint8_t(buf, 46, numParts);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN);
#else
    mace_auction_task_descriptor_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.taskGenTime = taskGenTime;
    packet.reqStart = reqStart;
    packet.reqEnd = reqEnd;
    packet.penalty = penalty;
    packet.taskID = taskID;
    packet.type = type;
    packet.numParts = numParts;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_CRC);
}

/**
 * @brief Encode a auction_task_descriptor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_task_descriptor C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_task_descriptor_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_task_descriptor_t* auction_task_descriptor)
{
    return mace_msg_auction_task_descriptor_pack(system_id, component_id, msg, auction_task_descriptor->sentFrom, auction_task_descriptor->creatorID, auction_task_descriptor->taskID, auction_task_descriptor->taskGenTime, auction_task_descriptor->type, auction_task_descriptor->penalty, auction_task_descriptor->reqStart, auction_task_descriptor->reqEnd, auction_task_descriptor->numParts);
}

/**
 * @brief Encode a auction_task_descriptor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_task_descriptor C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_task_descriptor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_task_descriptor_t* auction_task_descriptor)
{
    return mace_msg_auction_task_descriptor_pack_chan(system_id, component_id, chan, msg, auction_task_descriptor->sentFrom, auction_task_descriptor->creatorID, auction_task_descriptor->taskID, auction_task_descriptor->taskGenTime, auction_task_descriptor->type, auction_task_descriptor->penalty, auction_task_descriptor->reqStart, auction_task_descriptor->reqEnd, auction_task_descriptor->numParts);
}

/**
 * @brief Send a auction_task_descriptor message
 * @param chan MAVLink channel to send the message
 *
 * @param sentFrom ID of agent the descriptor is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param taskGenTime Task generation time
 * @param type Task type
 * @param penalty Time penalty
 * @param reqStart Required Start
 * @param reqEnd Required End
 * @param numParts Number of messages to be sent to receive the rest of the descriptor
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_task_descriptor_send(mace_channel_t chan, uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, double taskGenTime, uint8_t type, uint32_t penalty, double reqStart, double reqEnd, uint8_t numParts)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, taskGenTime);
    _mace_put_double(buf, 24, reqStart);
    _mace_put_double(buf, 32, reqEnd);
    _mace_put_uint32_t(buf, 40, penalty);
    _mace_put_uint8_t(buf, 44, taskID);
    _mace_put_uint8_t(buf, 45, type);
    _mace_put_uint8_t(buf, 46, numParts);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR, buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_CRC);
#else
    mace_auction_task_descriptor_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.taskGenTime = taskGenTime;
    packet.reqStart = reqStart;
    packet.reqEnd = reqEnd;
    packet.penalty = penalty;
    packet.taskID = taskID;
    packet.type = type;
    packet.numParts = numParts;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR, (const char *)&packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_CRC);
#endif
}

/**
 * @brief Send a auction_task_descriptor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_task_descriptor_send_struct(mace_channel_t chan, const mace_auction_task_descriptor_t* auction_task_descriptor)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_task_descriptor_send(chan, auction_task_descriptor->sentFrom, auction_task_descriptor->creatorID, auction_task_descriptor->taskID, auction_task_descriptor->taskGenTime, auction_task_descriptor->type, auction_task_descriptor->penalty, auction_task_descriptor->reqStart, auction_task_descriptor->reqEnd, auction_task_descriptor->numParts);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR, (const char *)auction_task_descriptor, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_task_descriptor_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, double taskGenTime, uint8_t type, uint32_t penalty, double reqStart, double reqEnd, uint8_t numParts)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, taskGenTime);
    _mace_put_double(buf, 24, reqStart);
    _mace_put_double(buf, 32, reqEnd);
    _mace_put_uint32_t(buf, 40, penalty);
    _mace_put_uint8_t(buf, 44, taskID);
    _mace_put_uint8_t(buf, 45, type);
    _mace_put_uint8_t(buf, 46, numParts);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR, buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_CRC);
#else
    mace_auction_task_descriptor_t *packet = (mace_auction_task_descriptor_t *)msgbuf;
    packet->sentFrom = sentFrom;
    packet->creatorID = creatorID;
    packet->taskGenTime = taskGenTime;
    packet->reqStart = reqStart;
    packet->reqEnd = reqEnd;
    packet->penalty = penalty;
    packet->taskID = taskID;
    packet->type = type;
    packet->numParts = numParts;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR, (const char *)packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_TASK_DESCRIPTOR UNPACKING


/**
 * @brief Get field sentFrom from auction_task_descriptor message
 *
 * @return ID of agent the descriptor is being requested from
 */
static inline uint64_t mace_msg_auction_task_descriptor_get_sentFrom(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field creatorID from auction_task_descriptor message
 *
 * @return Task creator ID
 */
static inline uint64_t mace_msg_auction_task_descriptor_get_creatorID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field taskID from auction_task_descriptor message
 *
 * @return Creator local task ID
 */
static inline uint8_t mace_msg_auction_task_descriptor_get_taskID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field taskGenTime from auction_task_descriptor message
 *
 * @return Task generation time
 */
static inline double mace_msg_auction_task_descriptor_get_taskGenTime(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  16);
}

/**
 * @brief Get field type from auction_task_descriptor message
 *
 * @return Task type
 */
static inline uint8_t mace_msg_auction_task_descriptor_get_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field penalty from auction_task_descriptor message
 *
 * @return Time penalty
 */
static inline uint32_t mace_msg_auction_task_descriptor_get_penalty(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  40);
}

/**
 * @brief Get field reqStart from auction_task_descriptor message
 *
 * @return Required Start
 */
static inline double mace_msg_auction_task_descriptor_get_reqStart(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  24);
}

/**
 * @brief Get field reqEnd from auction_task_descriptor message
 *
 * @return Required End
 */
static inline double mace_msg_auction_task_descriptor_get_reqEnd(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  32);
}

/**
 * @brief Get field numParts from auction_task_descriptor message
 *
 * @return Number of messages to be sent to receive the rest of the descriptor
 */
static inline uint8_t mace_msg_auction_task_descriptor_get_numParts(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Decode a auction_task_descriptor message into a struct
 *
 * @param msg The message to decode
 * @param auction_task_descriptor C-struct to decode the message contents into
 */
static inline void mace_msg_auction_task_descriptor_decode(const mace_message_t* msg, mace_auction_task_descriptor_t* auction_task_descriptor)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_task_descriptor->sentFrom = mace_msg_auction_task_descriptor_get_sentFrom(msg);
    auction_task_descriptor->creatorID = mace_msg_auction_task_descriptor_get_creatorID(msg);
    auction_task_descriptor->taskGenTime = mace_msg_auction_task_descriptor_get_taskGenTime(msg);
    auction_task_descriptor->reqStart = mace_msg_auction_task_descriptor_get_reqStart(msg);
    auction_task_descriptor->reqEnd = mace_msg_auction_task_descriptor_get_reqEnd(msg);
    auction_task_descriptor->penalty = mace_msg_auction_task_descriptor_get_penalty(msg);
    auction_task_descriptor->taskID = mace_msg_auction_task_descriptor_get_taskID(msg);
    auction_task_descriptor->type = mace_msg_auction_task_descriptor_get_type(msg);
    auction_task_descriptor->numParts = mace_msg_auction_task_descriptor_get_numParts(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN? msg->len : MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN;
        memset(auction_task_descriptor, 0, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LEN);
    memcpy(auction_task_descriptor, _MACE_PAYLOAD(msg), len);
#endif
}
