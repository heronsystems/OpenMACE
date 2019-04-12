#pragma once
// MESSAGE AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER PACKING

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER 10013

MACEPACKED(
typedef struct __mace_auction_task_descriptor_send_next_data_part_loiter_t {
 uint64_t sentFrom; /*< ID of agent the task is being requested from*/
 uint64_t creatorID; /*< Task creator ID*/
 double xPos; /*< x position*/
 double yPos; /*< y position*/
 double zPos; /*< z position*/
 uint32_t duration; /*< duration*/
 uint8_t taskID; /*< Creator local task ID*/
 int8_t seqNum; /*< Sequence number*/
 uint8_t coordinateType; /*< Coordinate type. See TaskDescriptor::TaskType*/
 uint8_t coordinateFrame; /*< Coordinate frame. See mace::pose::CoordinateFrame*/
}) mace_auction_task_descriptor_send_next_data_part_loiter_t;

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN 48
#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_MIN_LEN 48
#define MACE_MSG_ID_10013_LEN 48
#define MACE_MSG_ID_10013_MIN_LEN 48

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_CRC 19
#define MACE_MSG_ID_10013_CRC 19



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER { \
    10013, \
    "AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER", \
    10, \
    {  { "sentFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, sentFrom) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, creatorID) }, \
         { "xPos", NULL, MACE_TYPE_DOUBLE, 0, 16, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, xPos) }, \
         { "yPos", NULL, MACE_TYPE_DOUBLE, 0, 24, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, yPos) }, \
         { "zPos", NULL, MACE_TYPE_DOUBLE, 0, 32, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, zPos) }, \
         { "duration", NULL, MACE_TYPE_UINT32_T, 0, 40, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, duration) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 44, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, taskID) }, \
         { "seqNum", NULL, MACE_TYPE_INT8_T, 0, 45, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, seqNum) }, \
         { "coordinateType", NULL, MACE_TYPE_UINT8_T, 0, 46, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, coordinateType) }, \
         { "coordinateFrame", NULL, MACE_TYPE_UINT8_T, 0, 47, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, coordinateFrame) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER { \
    "AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER", \
    10, \
    {  { "sentFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, sentFrom) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, creatorID) }, \
         { "xPos", NULL, MACE_TYPE_DOUBLE, 0, 16, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, xPos) }, \
         { "yPos", NULL, MACE_TYPE_DOUBLE, 0, 24, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, yPos) }, \
         { "zPos", NULL, MACE_TYPE_DOUBLE, 0, 32, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, zPos) }, \
         { "duration", NULL, MACE_TYPE_UINT32_T, 0, 40, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, duration) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 44, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, taskID) }, \
         { "seqNum", NULL, MACE_TYPE_INT8_T, 0, 45, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, seqNum) }, \
         { "coordinateType", NULL, MACE_TYPE_UINT8_T, 0, 46, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, coordinateType) }, \
         { "coordinateFrame", NULL, MACE_TYPE_UINT8_T, 0, 47, offsetof(mace_auction_task_descriptor_send_next_data_part_loiter_t, coordinateFrame) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_task_descriptor_send_next_data_part_loiter message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sentFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param seqNum Sequence number
 * @param xPos x position
 * @param yPos y position
 * @param zPos z position
 * @param duration duration
 * @param coordinateType Coordinate type. See TaskDescriptor::TaskType
 * @param coordinateFrame Coordinate frame. See mace::pose::CoordinateFrame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_task_descriptor_send_next_data_part_loiter_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, int8_t seqNum, double xPos, double yPos, double zPos, uint32_t duration, uint8_t coordinateType, uint8_t coordinateFrame)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, xPos);
    _mace_put_double(buf, 24, yPos);
    _mace_put_double(buf, 32, zPos);
    _mace_put_uint32_t(buf, 40, duration);
    _mace_put_uint8_t(buf, 44, taskID);
    _mace_put_int8_t(buf, 45, seqNum);
    _mace_put_uint8_t(buf, 46, coordinateType);
    _mace_put_uint8_t(buf, 47, coordinateFrame);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN);
#else
    mace_auction_task_descriptor_send_next_data_part_loiter_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.xPos = xPos;
    packet.yPos = yPos;
    packet.zPos = zPos;
    packet.duration = duration;
    packet.taskID = taskID;
    packet.seqNum = seqNum;
    packet.coordinateType = coordinateType;
    packet.coordinateFrame = coordinateFrame;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_CRC);
}

/**
 * @brief Pack a auction_task_descriptor_send_next_data_part_loiter message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sentFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param seqNum Sequence number
 * @param xPos x position
 * @param yPos y position
 * @param zPos z position
 * @param duration duration
 * @param coordinateType Coordinate type. See TaskDescriptor::TaskType
 * @param coordinateFrame Coordinate frame. See mace::pose::CoordinateFrame
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_task_descriptor_send_next_data_part_loiter_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t sentFrom,uint64_t creatorID,uint8_t taskID,int8_t seqNum,double xPos,double yPos,double zPos,uint32_t duration,uint8_t coordinateType,uint8_t coordinateFrame)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, xPos);
    _mace_put_double(buf, 24, yPos);
    _mace_put_double(buf, 32, zPos);
    _mace_put_uint32_t(buf, 40, duration);
    _mace_put_uint8_t(buf, 44, taskID);
    _mace_put_int8_t(buf, 45, seqNum);
    _mace_put_uint8_t(buf, 46, coordinateType);
    _mace_put_uint8_t(buf, 47, coordinateFrame);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN);
#else
    mace_auction_task_descriptor_send_next_data_part_loiter_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.xPos = xPos;
    packet.yPos = yPos;
    packet.zPos = zPos;
    packet.duration = duration;
    packet.taskID = taskID;
    packet.seqNum = seqNum;
    packet.coordinateType = coordinateType;
    packet.coordinateFrame = coordinateFrame;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_CRC);
}

/**
 * @brief Encode a auction_task_descriptor_send_next_data_part_loiter struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_task_descriptor_send_next_data_part_loiter C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_task_descriptor_send_next_data_part_loiter_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_task_descriptor_send_next_data_part_loiter_t* auction_task_descriptor_send_next_data_part_loiter)
{
    return mace_msg_auction_task_descriptor_send_next_data_part_loiter_pack(system_id, component_id, msg, auction_task_descriptor_send_next_data_part_loiter->sentFrom, auction_task_descriptor_send_next_data_part_loiter->creatorID, auction_task_descriptor_send_next_data_part_loiter->taskID, auction_task_descriptor_send_next_data_part_loiter->seqNum, auction_task_descriptor_send_next_data_part_loiter->xPos, auction_task_descriptor_send_next_data_part_loiter->yPos, auction_task_descriptor_send_next_data_part_loiter->zPos, auction_task_descriptor_send_next_data_part_loiter->duration, auction_task_descriptor_send_next_data_part_loiter->coordinateType, auction_task_descriptor_send_next_data_part_loiter->coordinateFrame);
}

/**
 * @brief Encode a auction_task_descriptor_send_next_data_part_loiter struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_task_descriptor_send_next_data_part_loiter C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_task_descriptor_send_next_data_part_loiter_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_task_descriptor_send_next_data_part_loiter_t* auction_task_descriptor_send_next_data_part_loiter)
{
    return mace_msg_auction_task_descriptor_send_next_data_part_loiter_pack_chan(system_id, component_id, chan, msg, auction_task_descriptor_send_next_data_part_loiter->sentFrom, auction_task_descriptor_send_next_data_part_loiter->creatorID, auction_task_descriptor_send_next_data_part_loiter->taskID, auction_task_descriptor_send_next_data_part_loiter->seqNum, auction_task_descriptor_send_next_data_part_loiter->xPos, auction_task_descriptor_send_next_data_part_loiter->yPos, auction_task_descriptor_send_next_data_part_loiter->zPos, auction_task_descriptor_send_next_data_part_loiter->duration, auction_task_descriptor_send_next_data_part_loiter->coordinateType, auction_task_descriptor_send_next_data_part_loiter->coordinateFrame);
}

/**
 * @brief Send a auction_task_descriptor_send_next_data_part_loiter message
 * @param chan MAVLink channel to send the message
 *
 * @param sentFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param seqNum Sequence number
 * @param xPos x position
 * @param yPos y position
 * @param zPos z position
 * @param duration duration
 * @param coordinateType Coordinate type. See TaskDescriptor::TaskType
 * @param coordinateFrame Coordinate frame. See mace::pose::CoordinateFrame
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_task_descriptor_send_next_data_part_loiter_send(mace_channel_t chan, uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, int8_t seqNum, double xPos, double yPos, double zPos, uint32_t duration, uint8_t coordinateType, uint8_t coordinateFrame)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, xPos);
    _mace_put_double(buf, 24, yPos);
    _mace_put_double(buf, 32, zPos);
    _mace_put_uint32_t(buf, 40, duration);
    _mace_put_uint8_t(buf, 44, taskID);
    _mace_put_int8_t(buf, 45, seqNum);
    _mace_put_uint8_t(buf, 46, coordinateType);
    _mace_put_uint8_t(buf, 47, coordinateFrame);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER, buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_CRC);
#else
    mace_auction_task_descriptor_send_next_data_part_loiter_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.xPos = xPos;
    packet.yPos = yPos;
    packet.zPos = zPos;
    packet.duration = duration;
    packet.taskID = taskID;
    packet.seqNum = seqNum;
    packet.coordinateType = coordinateType;
    packet.coordinateFrame = coordinateFrame;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER, (const char *)&packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_CRC);
#endif
}

/**
 * @brief Send a auction_task_descriptor_send_next_data_part_loiter message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_task_descriptor_send_next_data_part_loiter_send_struct(mace_channel_t chan, const mace_auction_task_descriptor_send_next_data_part_loiter_t* auction_task_descriptor_send_next_data_part_loiter)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_task_descriptor_send_next_data_part_loiter_send(chan, auction_task_descriptor_send_next_data_part_loiter->sentFrom, auction_task_descriptor_send_next_data_part_loiter->creatorID, auction_task_descriptor_send_next_data_part_loiter->taskID, auction_task_descriptor_send_next_data_part_loiter->seqNum, auction_task_descriptor_send_next_data_part_loiter->xPos, auction_task_descriptor_send_next_data_part_loiter->yPos, auction_task_descriptor_send_next_data_part_loiter->zPos, auction_task_descriptor_send_next_data_part_loiter->duration, auction_task_descriptor_send_next_data_part_loiter->coordinateType, auction_task_descriptor_send_next_data_part_loiter->coordinateFrame);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER, (const char *)auction_task_descriptor_send_next_data_part_loiter, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_task_descriptor_send_next_data_part_loiter_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, int8_t seqNum, double xPos, double yPos, double zPos, uint32_t duration, uint8_t coordinateType, uint8_t coordinateFrame)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, xPos);
    _mace_put_double(buf, 24, yPos);
    _mace_put_double(buf, 32, zPos);
    _mace_put_uint32_t(buf, 40, duration);
    _mace_put_uint8_t(buf, 44, taskID);
    _mace_put_int8_t(buf, 45, seqNum);
    _mace_put_uint8_t(buf, 46, coordinateType);
    _mace_put_uint8_t(buf, 47, coordinateFrame);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER, buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_CRC);
#else
    mace_auction_task_descriptor_send_next_data_part_loiter_t *packet = (mace_auction_task_descriptor_send_next_data_part_loiter_t *)msgbuf;
    packet->sentFrom = sentFrom;
    packet->creatorID = creatorID;
    packet->xPos = xPos;
    packet->yPos = yPos;
    packet->zPos = zPos;
    packet->duration = duration;
    packet->taskID = taskID;
    packet->seqNum = seqNum;
    packet->coordinateType = coordinateType;
    packet->coordinateFrame = coordinateFrame;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER, (const char *)packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER UNPACKING


/**
 * @brief Get field sentFrom from auction_task_descriptor_send_next_data_part_loiter message
 *
 * @return ID of agent the task is being requested from
 */
static inline uint64_t mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_sentFrom(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field creatorID from auction_task_descriptor_send_next_data_part_loiter message
 *
 * @return Task creator ID
 */
static inline uint64_t mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_creatorID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field taskID from auction_task_descriptor_send_next_data_part_loiter message
 *
 * @return Creator local task ID
 */
static inline uint8_t mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_taskID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field seqNum from auction_task_descriptor_send_next_data_part_loiter message
 *
 * @return Sequence number
 */
static inline int8_t mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_seqNum(const mace_message_t* msg)
{
    return _MACE_RETURN_int8_t(msg,  45);
}

/**
 * @brief Get field xPos from auction_task_descriptor_send_next_data_part_loiter message
 *
 * @return x position
 */
static inline double mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_xPos(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  16);
}

/**
 * @brief Get field yPos from auction_task_descriptor_send_next_data_part_loiter message
 *
 * @return y position
 */
static inline double mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_yPos(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  24);
}

/**
 * @brief Get field zPos from auction_task_descriptor_send_next_data_part_loiter message
 *
 * @return z position
 */
static inline double mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_zPos(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  32);
}

/**
 * @brief Get field duration from auction_task_descriptor_send_next_data_part_loiter message
 *
 * @return duration
 */
static inline uint32_t mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_duration(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  40);
}

/**
 * @brief Get field coordinateType from auction_task_descriptor_send_next_data_part_loiter message
 *
 * @return Coordinate type. See TaskDescriptor::TaskType
 */
static inline uint8_t mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_coordinateType(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field coordinateFrame from auction_task_descriptor_send_next_data_part_loiter message
 *
 * @return Coordinate frame. See mace::pose::CoordinateFrame
 */
static inline uint8_t mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_coordinateFrame(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  47);
}

/**
 * @brief Decode a auction_task_descriptor_send_next_data_part_loiter message into a struct
 *
 * @param msg The message to decode
 * @param auction_task_descriptor_send_next_data_part_loiter C-struct to decode the message contents into
 */
static inline void mace_msg_auction_task_descriptor_send_next_data_part_loiter_decode(const mace_message_t* msg, mace_auction_task_descriptor_send_next_data_part_loiter_t* auction_task_descriptor_send_next_data_part_loiter)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_task_descriptor_send_next_data_part_loiter->sentFrom = mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_sentFrom(msg);
    auction_task_descriptor_send_next_data_part_loiter->creatorID = mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_creatorID(msg);
    auction_task_descriptor_send_next_data_part_loiter->xPos = mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_xPos(msg);
    auction_task_descriptor_send_next_data_part_loiter->yPos = mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_yPos(msg);
    auction_task_descriptor_send_next_data_part_loiter->zPos = mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_zPos(msg);
    auction_task_descriptor_send_next_data_part_loiter->duration = mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_duration(msg);
    auction_task_descriptor_send_next_data_part_loiter->taskID = mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_taskID(msg);
    auction_task_descriptor_send_next_data_part_loiter->seqNum = mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_seqNum(msg);
    auction_task_descriptor_send_next_data_part_loiter->coordinateType = mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_coordinateType(msg);
    auction_task_descriptor_send_next_data_part_loiter->coordinateFrame = mace_msg_auction_task_descriptor_send_next_data_part_loiter_get_coordinateFrame(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN? msg->len : MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN;
        memset(auction_task_descriptor_send_next_data_part_loiter, 0, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_LOITER_LEN);
    memcpy(auction_task_descriptor_send_next_data_part_loiter, _MACE_PAYLOAD(msg), len);
#endif
}
