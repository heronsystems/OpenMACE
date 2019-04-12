#pragma once
// MESSAGE AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST PACKING

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST 10055

MACEPACKED(
typedef struct __mace_auction_task_descriptor_part_survey_first_t {
 uint64_t sentFrom; /*< ID of agent the task is being requested from*/
 uint64_t creatorID; /*< Task creator ID*/
 double sensorResolution; /*< Sensor resolution*/
 double overlapHorizontal; /*< Horizontal overlap*/
 double overlapVertical; /*< Vertical overlap*/
 uint8_t taskID; /*< Creator local task ID*/
 int8_t seqNum; /*< Sequence number*/
 uint8_t coordinateType; /*< Coordinate type. See TaskDescriptor::TaskType*/
}) mace_auction_task_descriptor_part_survey_first_t;

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN 43
#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_MIN_LEN 43
#define MACE_MSG_ID_10055_LEN 43
#define MACE_MSG_ID_10055_MIN_LEN 43

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_CRC 54
#define MACE_MSG_ID_10055_CRC 54



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST { \
    10055, \
    "AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST", \
    8, \
    {  { "sentFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_task_descriptor_part_survey_first_t, sentFrom) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_task_descriptor_part_survey_first_t, creatorID) }, \
         { "sensorResolution", NULL, MACE_TYPE_DOUBLE, 0, 16, offsetof(mace_auction_task_descriptor_part_survey_first_t, sensorResolution) }, \
         { "overlapHorizontal", NULL, MACE_TYPE_DOUBLE, 0, 24, offsetof(mace_auction_task_descriptor_part_survey_first_t, overlapHorizontal) }, \
         { "overlapVertical", NULL, MACE_TYPE_DOUBLE, 0, 32, offsetof(mace_auction_task_descriptor_part_survey_first_t, overlapVertical) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 40, offsetof(mace_auction_task_descriptor_part_survey_first_t, taskID) }, \
         { "seqNum", NULL, MACE_TYPE_INT8_T, 0, 41, offsetof(mace_auction_task_descriptor_part_survey_first_t, seqNum) }, \
         { "coordinateType", NULL, MACE_TYPE_UINT8_T, 0, 42, offsetof(mace_auction_task_descriptor_part_survey_first_t, coordinateType) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST { \
    "AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST", \
    8, \
    {  { "sentFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_task_descriptor_part_survey_first_t, sentFrom) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_task_descriptor_part_survey_first_t, creatorID) }, \
         { "sensorResolution", NULL, MACE_TYPE_DOUBLE, 0, 16, offsetof(mace_auction_task_descriptor_part_survey_first_t, sensorResolution) }, \
         { "overlapHorizontal", NULL, MACE_TYPE_DOUBLE, 0, 24, offsetof(mace_auction_task_descriptor_part_survey_first_t, overlapHorizontal) }, \
         { "overlapVertical", NULL, MACE_TYPE_DOUBLE, 0, 32, offsetof(mace_auction_task_descriptor_part_survey_first_t, overlapVertical) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 40, offsetof(mace_auction_task_descriptor_part_survey_first_t, taskID) }, \
         { "seqNum", NULL, MACE_TYPE_INT8_T, 0, 41, offsetof(mace_auction_task_descriptor_part_survey_first_t, seqNum) }, \
         { "coordinateType", NULL, MACE_TYPE_UINT8_T, 0, 42, offsetof(mace_auction_task_descriptor_part_survey_first_t, coordinateType) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_task_descriptor_part_survey_first message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sentFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param seqNum Sequence number
 * @param sensorResolution Sensor resolution
 * @param overlapHorizontal Horizontal overlap
 * @param overlapVertical Vertical overlap
 * @param coordinateType Coordinate type. See TaskDescriptor::TaskType
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_task_descriptor_part_survey_first_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, int8_t seqNum, double sensorResolution, double overlapHorizontal, double overlapVertical, uint8_t coordinateType)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, sensorResolution);
    _mace_put_double(buf, 24, overlapHorizontal);
    _mace_put_double(buf, 32, overlapVertical);
    _mace_put_uint8_t(buf, 40, taskID);
    _mace_put_int8_t(buf, 41, seqNum);
    _mace_put_uint8_t(buf, 42, coordinateType);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN);
#else
    mace_auction_task_descriptor_part_survey_first_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.sensorResolution = sensorResolution;
    packet.overlapHorizontal = overlapHorizontal;
    packet.overlapVertical = overlapVertical;
    packet.taskID = taskID;
    packet.seqNum = seqNum;
    packet.coordinateType = coordinateType;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_CRC);
}

/**
 * @brief Pack a auction_task_descriptor_part_survey_first message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sentFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param seqNum Sequence number
 * @param sensorResolution Sensor resolution
 * @param overlapHorizontal Horizontal overlap
 * @param overlapVertical Vertical overlap
 * @param coordinateType Coordinate type. See TaskDescriptor::TaskType
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_task_descriptor_part_survey_first_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t sentFrom,uint64_t creatorID,uint8_t taskID,int8_t seqNum,double sensorResolution,double overlapHorizontal,double overlapVertical,uint8_t coordinateType)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, sensorResolution);
    _mace_put_double(buf, 24, overlapHorizontal);
    _mace_put_double(buf, 32, overlapVertical);
    _mace_put_uint8_t(buf, 40, taskID);
    _mace_put_int8_t(buf, 41, seqNum);
    _mace_put_uint8_t(buf, 42, coordinateType);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN);
#else
    mace_auction_task_descriptor_part_survey_first_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.sensorResolution = sensorResolution;
    packet.overlapHorizontal = overlapHorizontal;
    packet.overlapVertical = overlapVertical;
    packet.taskID = taskID;
    packet.seqNum = seqNum;
    packet.coordinateType = coordinateType;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_CRC);
}

/**
 * @brief Encode a auction_task_descriptor_part_survey_first struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_task_descriptor_part_survey_first C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_task_descriptor_part_survey_first_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_task_descriptor_part_survey_first_t* auction_task_descriptor_part_survey_first)
{
    return mace_msg_auction_task_descriptor_part_survey_first_pack(system_id, component_id, msg, auction_task_descriptor_part_survey_first->sentFrom, auction_task_descriptor_part_survey_first->creatorID, auction_task_descriptor_part_survey_first->taskID, auction_task_descriptor_part_survey_first->seqNum, auction_task_descriptor_part_survey_first->sensorResolution, auction_task_descriptor_part_survey_first->overlapHorizontal, auction_task_descriptor_part_survey_first->overlapVertical, auction_task_descriptor_part_survey_first->coordinateType);
}

/**
 * @brief Encode a auction_task_descriptor_part_survey_first struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_task_descriptor_part_survey_first C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_task_descriptor_part_survey_first_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_task_descriptor_part_survey_first_t* auction_task_descriptor_part_survey_first)
{
    return mace_msg_auction_task_descriptor_part_survey_first_pack_chan(system_id, component_id, chan, msg, auction_task_descriptor_part_survey_first->sentFrom, auction_task_descriptor_part_survey_first->creatorID, auction_task_descriptor_part_survey_first->taskID, auction_task_descriptor_part_survey_first->seqNum, auction_task_descriptor_part_survey_first->sensorResolution, auction_task_descriptor_part_survey_first->overlapHorizontal, auction_task_descriptor_part_survey_first->overlapVertical, auction_task_descriptor_part_survey_first->coordinateType);
}

/**
 * @brief Send a auction_task_descriptor_part_survey_first message
 * @param chan MAVLink channel to send the message
 *
 * @param sentFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param seqNum Sequence number
 * @param sensorResolution Sensor resolution
 * @param overlapHorizontal Horizontal overlap
 * @param overlapVertical Vertical overlap
 * @param coordinateType Coordinate type. See TaskDescriptor::TaskType
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_task_descriptor_part_survey_first_send(mace_channel_t chan, uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, int8_t seqNum, double sensorResolution, double overlapHorizontal, double overlapVertical, uint8_t coordinateType)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, sensorResolution);
    _mace_put_double(buf, 24, overlapHorizontal);
    _mace_put_double(buf, 32, overlapVertical);
    _mace_put_uint8_t(buf, 40, taskID);
    _mace_put_int8_t(buf, 41, seqNum);
    _mace_put_uint8_t(buf, 42, coordinateType);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST, buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_CRC);
#else
    mace_auction_task_descriptor_part_survey_first_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.sensorResolution = sensorResolution;
    packet.overlapHorizontal = overlapHorizontal;
    packet.overlapVertical = overlapVertical;
    packet.taskID = taskID;
    packet.seqNum = seqNum;
    packet.coordinateType = coordinateType;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST, (const char *)&packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_CRC);
#endif
}

/**
 * @brief Send a auction_task_descriptor_part_survey_first message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_task_descriptor_part_survey_first_send_struct(mace_channel_t chan, const mace_auction_task_descriptor_part_survey_first_t* auction_task_descriptor_part_survey_first)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_task_descriptor_part_survey_first_send(chan, auction_task_descriptor_part_survey_first->sentFrom, auction_task_descriptor_part_survey_first->creatorID, auction_task_descriptor_part_survey_first->taskID, auction_task_descriptor_part_survey_first->seqNum, auction_task_descriptor_part_survey_first->sensorResolution, auction_task_descriptor_part_survey_first->overlapHorizontal, auction_task_descriptor_part_survey_first->overlapVertical, auction_task_descriptor_part_survey_first->coordinateType);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST, (const char *)auction_task_descriptor_part_survey_first, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_task_descriptor_part_survey_first_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, int8_t seqNum, double sensorResolution, double overlapHorizontal, double overlapVertical, uint8_t coordinateType)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, sensorResolution);
    _mace_put_double(buf, 24, overlapHorizontal);
    _mace_put_double(buf, 32, overlapVertical);
    _mace_put_uint8_t(buf, 40, taskID);
    _mace_put_int8_t(buf, 41, seqNum);
    _mace_put_uint8_t(buf, 42, coordinateType);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST, buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_CRC);
#else
    mace_auction_task_descriptor_part_survey_first_t *packet = (mace_auction_task_descriptor_part_survey_first_t *)msgbuf;
    packet->sentFrom = sentFrom;
    packet->creatorID = creatorID;
    packet->sensorResolution = sensorResolution;
    packet->overlapHorizontal = overlapHorizontal;
    packet->overlapVertical = overlapVertical;
    packet->taskID = taskID;
    packet->seqNum = seqNum;
    packet->coordinateType = coordinateType;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST, (const char *)packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST UNPACKING


/**
 * @brief Get field sentFrom from auction_task_descriptor_part_survey_first message
 *
 * @return ID of agent the task is being requested from
 */
static inline uint64_t mace_msg_auction_task_descriptor_part_survey_first_get_sentFrom(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field creatorID from auction_task_descriptor_part_survey_first message
 *
 * @return Task creator ID
 */
static inline uint64_t mace_msg_auction_task_descriptor_part_survey_first_get_creatorID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field taskID from auction_task_descriptor_part_survey_first message
 *
 * @return Creator local task ID
 */
static inline uint8_t mace_msg_auction_task_descriptor_part_survey_first_get_taskID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field seqNum from auction_task_descriptor_part_survey_first message
 *
 * @return Sequence number
 */
static inline int8_t mace_msg_auction_task_descriptor_part_survey_first_get_seqNum(const mace_message_t* msg)
{
    return _MACE_RETURN_int8_t(msg,  41);
}

/**
 * @brief Get field sensorResolution from auction_task_descriptor_part_survey_first message
 *
 * @return Sensor resolution
 */
static inline double mace_msg_auction_task_descriptor_part_survey_first_get_sensorResolution(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  16);
}

/**
 * @brief Get field overlapHorizontal from auction_task_descriptor_part_survey_first message
 *
 * @return Horizontal overlap
 */
static inline double mace_msg_auction_task_descriptor_part_survey_first_get_overlapHorizontal(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  24);
}

/**
 * @brief Get field overlapVertical from auction_task_descriptor_part_survey_first message
 *
 * @return Vertical overlap
 */
static inline double mace_msg_auction_task_descriptor_part_survey_first_get_overlapVertical(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  32);
}

/**
 * @brief Get field coordinateType from auction_task_descriptor_part_survey_first message
 *
 * @return Coordinate type. See TaskDescriptor::TaskType
 */
static inline uint8_t mace_msg_auction_task_descriptor_part_survey_first_get_coordinateType(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Decode a auction_task_descriptor_part_survey_first message into a struct
 *
 * @param msg The message to decode
 * @param auction_task_descriptor_part_survey_first C-struct to decode the message contents into
 */
static inline void mace_msg_auction_task_descriptor_part_survey_first_decode(const mace_message_t* msg, mace_auction_task_descriptor_part_survey_first_t* auction_task_descriptor_part_survey_first)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_task_descriptor_part_survey_first->sentFrom = mace_msg_auction_task_descriptor_part_survey_first_get_sentFrom(msg);
    auction_task_descriptor_part_survey_first->creatorID = mace_msg_auction_task_descriptor_part_survey_first_get_creatorID(msg);
    auction_task_descriptor_part_survey_first->sensorResolution = mace_msg_auction_task_descriptor_part_survey_first_get_sensorResolution(msg);
    auction_task_descriptor_part_survey_first->overlapHorizontal = mace_msg_auction_task_descriptor_part_survey_first_get_overlapHorizontal(msg);
    auction_task_descriptor_part_survey_first->overlapVertical = mace_msg_auction_task_descriptor_part_survey_first_get_overlapVertical(msg);
    auction_task_descriptor_part_survey_first->taskID = mace_msg_auction_task_descriptor_part_survey_first_get_taskID(msg);
    auction_task_descriptor_part_survey_first->seqNum = mace_msg_auction_task_descriptor_part_survey_first_get_seqNum(msg);
    auction_task_descriptor_part_survey_first->coordinateType = mace_msg_auction_task_descriptor_part_survey_first_get_coordinateType(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN? msg->len : MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN;
        memset(auction_task_descriptor_part_survey_first, 0, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_LEN);
    memcpy(auction_task_descriptor_part_survey_first, _MACE_PAYLOAD(msg), len);
#endif
}
