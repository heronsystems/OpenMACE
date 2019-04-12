#pragma once
// MESSAGE AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY PACKING

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY 10015

MACEPACKED(
typedef struct __mace_auction_task_descriptor_send_next_data_part_survey_t {
 uint64_t sentFrom; /*< ID of agent the task is being requested from*/
 uint64_t creatorID; /*< Task creator ID*/
 double xPos; /*< x position*/
 double yPos; /*< y position*/
 uint8_t taskID; /*< Creator local task ID*/
 int8_t seqNum; /*< Sequence number*/
}) mace_auction_task_descriptor_send_next_data_part_survey_t;

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN 34
#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_MIN_LEN 34
#define MACE_MSG_ID_10015_LEN 34
#define MACE_MSG_ID_10015_MIN_LEN 34

#define MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_CRC 214
#define MACE_MSG_ID_10015_CRC 214



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY { \
    10015, \
    "AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY", \
    6, \
    {  { "sentFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_task_descriptor_send_next_data_part_survey_t, sentFrom) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_task_descriptor_send_next_data_part_survey_t, creatorID) }, \
         { "xPos", NULL, MACE_TYPE_DOUBLE, 0, 16, offsetof(mace_auction_task_descriptor_send_next_data_part_survey_t, xPos) }, \
         { "yPos", NULL, MACE_TYPE_DOUBLE, 0, 24, offsetof(mace_auction_task_descriptor_send_next_data_part_survey_t, yPos) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 32, offsetof(mace_auction_task_descriptor_send_next_data_part_survey_t, taskID) }, \
         { "seqNum", NULL, MACE_TYPE_INT8_T, 0, 33, offsetof(mace_auction_task_descriptor_send_next_data_part_survey_t, seqNum) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY { \
    "AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY", \
    6, \
    {  { "sentFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_task_descriptor_send_next_data_part_survey_t, sentFrom) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_task_descriptor_send_next_data_part_survey_t, creatorID) }, \
         { "xPos", NULL, MACE_TYPE_DOUBLE, 0, 16, offsetof(mace_auction_task_descriptor_send_next_data_part_survey_t, xPos) }, \
         { "yPos", NULL, MACE_TYPE_DOUBLE, 0, 24, offsetof(mace_auction_task_descriptor_send_next_data_part_survey_t, yPos) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 32, offsetof(mace_auction_task_descriptor_send_next_data_part_survey_t, taskID) }, \
         { "seqNum", NULL, MACE_TYPE_INT8_T, 0, 33, offsetof(mace_auction_task_descriptor_send_next_data_part_survey_t, seqNum) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_task_descriptor_send_next_data_part_survey message
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
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_task_descriptor_send_next_data_part_survey_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, int8_t seqNum, double xPos, double yPos)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, xPos);
    _mace_put_double(buf, 24, yPos);
    _mace_put_uint8_t(buf, 32, taskID);
    _mace_put_int8_t(buf, 33, seqNum);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN);
#else
    mace_auction_task_descriptor_send_next_data_part_survey_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.xPos = xPos;
    packet.yPos = yPos;
    packet.taskID = taskID;
    packet.seqNum = seqNum;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_CRC);
}

/**
 * @brief Pack a auction_task_descriptor_send_next_data_part_survey message on a channel
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
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_task_descriptor_send_next_data_part_survey_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t sentFrom,uint64_t creatorID,uint8_t taskID,int8_t seqNum,double xPos,double yPos)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, xPos);
    _mace_put_double(buf, 24, yPos);
    _mace_put_uint8_t(buf, 32, taskID);
    _mace_put_int8_t(buf, 33, seqNum);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN);
#else
    mace_auction_task_descriptor_send_next_data_part_survey_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.xPos = xPos;
    packet.yPos = yPos;
    packet.taskID = taskID;
    packet.seqNum = seqNum;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_CRC);
}

/**
 * @brief Encode a auction_task_descriptor_send_next_data_part_survey struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_task_descriptor_send_next_data_part_survey C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_task_descriptor_send_next_data_part_survey_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_task_descriptor_send_next_data_part_survey_t* auction_task_descriptor_send_next_data_part_survey)
{
    return mace_msg_auction_task_descriptor_send_next_data_part_survey_pack(system_id, component_id, msg, auction_task_descriptor_send_next_data_part_survey->sentFrom, auction_task_descriptor_send_next_data_part_survey->creatorID, auction_task_descriptor_send_next_data_part_survey->taskID, auction_task_descriptor_send_next_data_part_survey->seqNum, auction_task_descriptor_send_next_data_part_survey->xPos, auction_task_descriptor_send_next_data_part_survey->yPos);
}

/**
 * @brief Encode a auction_task_descriptor_send_next_data_part_survey struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_task_descriptor_send_next_data_part_survey C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_task_descriptor_send_next_data_part_survey_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_task_descriptor_send_next_data_part_survey_t* auction_task_descriptor_send_next_data_part_survey)
{
    return mace_msg_auction_task_descriptor_send_next_data_part_survey_pack_chan(system_id, component_id, chan, msg, auction_task_descriptor_send_next_data_part_survey->sentFrom, auction_task_descriptor_send_next_data_part_survey->creatorID, auction_task_descriptor_send_next_data_part_survey->taskID, auction_task_descriptor_send_next_data_part_survey->seqNum, auction_task_descriptor_send_next_data_part_survey->xPos, auction_task_descriptor_send_next_data_part_survey->yPos);
}

/**
 * @brief Send a auction_task_descriptor_send_next_data_part_survey message
 * @param chan MAVLink channel to send the message
 *
 * @param sentFrom ID of agent the task is being requested from
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param seqNum Sequence number
 * @param xPos x position
 * @param yPos y position
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_task_descriptor_send_next_data_part_survey_send(mace_channel_t chan, uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, int8_t seqNum, double xPos, double yPos)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN];
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, xPos);
    _mace_put_double(buf, 24, yPos);
    _mace_put_uint8_t(buf, 32, taskID);
    _mace_put_int8_t(buf, 33, seqNum);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY, buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_CRC);
#else
    mace_auction_task_descriptor_send_next_data_part_survey_t packet;
    packet.sentFrom = sentFrom;
    packet.creatorID = creatorID;
    packet.xPos = xPos;
    packet.yPos = yPos;
    packet.taskID = taskID;
    packet.seqNum = seqNum;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY, (const char *)&packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_CRC);
#endif
}

/**
 * @brief Send a auction_task_descriptor_send_next_data_part_survey message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_task_descriptor_send_next_data_part_survey_send_struct(mace_channel_t chan, const mace_auction_task_descriptor_send_next_data_part_survey_t* auction_task_descriptor_send_next_data_part_survey)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_task_descriptor_send_next_data_part_survey_send(chan, auction_task_descriptor_send_next_data_part_survey->sentFrom, auction_task_descriptor_send_next_data_part_survey->creatorID, auction_task_descriptor_send_next_data_part_survey->taskID, auction_task_descriptor_send_next_data_part_survey->seqNum, auction_task_descriptor_send_next_data_part_survey->xPos, auction_task_descriptor_send_next_data_part_survey->yPos);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY, (const char *)auction_task_descriptor_send_next_data_part_survey, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_task_descriptor_send_next_data_part_survey_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t sentFrom, uint64_t creatorID, uint8_t taskID, int8_t seqNum, double xPos, double yPos)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, sentFrom);
    _mace_put_uint64_t(buf, 8, creatorID);
    _mace_put_double(buf, 16, xPos);
    _mace_put_double(buf, 24, yPos);
    _mace_put_uint8_t(buf, 32, taskID);
    _mace_put_int8_t(buf, 33, seqNum);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY, buf, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_CRC);
#else
    mace_auction_task_descriptor_send_next_data_part_survey_t *packet = (mace_auction_task_descriptor_send_next_data_part_survey_t *)msgbuf;
    packet->sentFrom = sentFrom;
    packet->creatorID = creatorID;
    packet->xPos = xPos;
    packet->yPos = yPos;
    packet->taskID = taskID;
    packet->seqNum = seqNum;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY, (const char *)packet, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_MIN_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY UNPACKING


/**
 * @brief Get field sentFrom from auction_task_descriptor_send_next_data_part_survey message
 *
 * @return ID of agent the task is being requested from
 */
static inline uint64_t mace_msg_auction_task_descriptor_send_next_data_part_survey_get_sentFrom(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field creatorID from auction_task_descriptor_send_next_data_part_survey message
 *
 * @return Task creator ID
 */
static inline uint64_t mace_msg_auction_task_descriptor_send_next_data_part_survey_get_creatorID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field taskID from auction_task_descriptor_send_next_data_part_survey message
 *
 * @return Creator local task ID
 */
static inline uint8_t mace_msg_auction_task_descriptor_send_next_data_part_survey_get_taskID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field seqNum from auction_task_descriptor_send_next_data_part_survey message
 *
 * @return Sequence number
 */
static inline int8_t mace_msg_auction_task_descriptor_send_next_data_part_survey_get_seqNum(const mace_message_t* msg)
{
    return _MACE_RETURN_int8_t(msg,  33);
}

/**
 * @brief Get field xPos from auction_task_descriptor_send_next_data_part_survey message
 *
 * @return x position
 */
static inline double mace_msg_auction_task_descriptor_send_next_data_part_survey_get_xPos(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  16);
}

/**
 * @brief Get field yPos from auction_task_descriptor_send_next_data_part_survey message
 *
 * @return y position
 */
static inline double mace_msg_auction_task_descriptor_send_next_data_part_survey_get_yPos(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  24);
}

/**
 * @brief Decode a auction_task_descriptor_send_next_data_part_survey message into a struct
 *
 * @param msg The message to decode
 * @param auction_task_descriptor_send_next_data_part_survey C-struct to decode the message contents into
 */
static inline void mace_msg_auction_task_descriptor_send_next_data_part_survey_decode(const mace_message_t* msg, mace_auction_task_descriptor_send_next_data_part_survey_t* auction_task_descriptor_send_next_data_part_survey)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_task_descriptor_send_next_data_part_survey->sentFrom = mace_msg_auction_task_descriptor_send_next_data_part_survey_get_sentFrom(msg);
    auction_task_descriptor_send_next_data_part_survey->creatorID = mace_msg_auction_task_descriptor_send_next_data_part_survey_get_creatorID(msg);
    auction_task_descriptor_send_next_data_part_survey->xPos = mace_msg_auction_task_descriptor_send_next_data_part_survey_get_xPos(msg);
    auction_task_descriptor_send_next_data_part_survey->yPos = mace_msg_auction_task_descriptor_send_next_data_part_survey_get_yPos(msg);
    auction_task_descriptor_send_next_data_part_survey->taskID = mace_msg_auction_task_descriptor_send_next_data_part_survey_get_taskID(msg);
    auction_task_descriptor_send_next_data_part_survey->seqNum = mace_msg_auction_task_descriptor_send_next_data_part_survey_get_seqNum(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN? msg->len : MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN;
        memset(auction_task_descriptor_send_next_data_part_survey, 0, MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_SEND_NEXT_DATA_PART_SURVEY_LEN);
    memcpy(auction_task_descriptor_send_next_data_part_survey, _MACE_PAYLOAD(msg), len);
#endif
}
