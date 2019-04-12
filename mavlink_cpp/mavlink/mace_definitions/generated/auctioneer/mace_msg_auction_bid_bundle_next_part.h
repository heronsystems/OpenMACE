#pragma once
// MESSAGE AUCTION_BID_BUNDLE_NEXT_PART PACKING

#define MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART 10001

MACEPACKED(
typedef struct __mace_auction_bid_bundle_next_part_t {
 uint64_t requestFrom; /*< ID of agent requesting the bundle*/
 uint64_t agentID; /*< ID of agent generating the bundle*/
 double bundleGenTime; /*< Bundle generation time*/
 double utility; /*< Utility*/
 double work; /*< Work*/
 double cost; /*< Cost*/
 double reward; /*< Reward*/
 uint64_t creatorID; /*< Task creator ID*/
 double taskGenTime; /*< Task generation time*/
 uint8_t taskID; /*< Creator local task ID*/
 uint8_t type; /*< Task type*/
 uint8_t priority; /*< Priority*/
 int8_t seqNum; /*< Sequence number*/
}) mace_auction_bid_bundle_next_part_t;

#define MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN 76
#define MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_MIN_LEN 76
#define MACE_MSG_ID_10001_LEN 76
#define MACE_MSG_ID_10001_MIN_LEN 76

#define MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_CRC 73
#define MACE_MSG_ID_10001_CRC 73



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_BID_BUNDLE_NEXT_PART { \
    10001, \
    "AUCTION_BID_BUNDLE_NEXT_PART", \
    13, \
    {  { "requestFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_bid_bundle_next_part_t, requestFrom) }, \
         { "agentID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_bid_bundle_next_part_t, agentID) }, \
         { "bundleGenTime", NULL, MACE_TYPE_DOUBLE, 0, 16, offsetof(mace_auction_bid_bundle_next_part_t, bundleGenTime) }, \
         { "utility", NULL, MACE_TYPE_DOUBLE, 0, 24, offsetof(mace_auction_bid_bundle_next_part_t, utility) }, \
         { "work", NULL, MACE_TYPE_DOUBLE, 0, 32, offsetof(mace_auction_bid_bundle_next_part_t, work) }, \
         { "cost", NULL, MACE_TYPE_DOUBLE, 0, 40, offsetof(mace_auction_bid_bundle_next_part_t, cost) }, \
         { "reward", NULL, MACE_TYPE_DOUBLE, 0, 48, offsetof(mace_auction_bid_bundle_next_part_t, reward) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 56, offsetof(mace_auction_bid_bundle_next_part_t, creatorID) }, \
         { "taskGenTime", NULL, MACE_TYPE_DOUBLE, 0, 64, offsetof(mace_auction_bid_bundle_next_part_t, taskGenTime) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 72, offsetof(mace_auction_bid_bundle_next_part_t, taskID) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 73, offsetof(mace_auction_bid_bundle_next_part_t, type) }, \
         { "priority", NULL, MACE_TYPE_UINT8_T, 0, 74, offsetof(mace_auction_bid_bundle_next_part_t, priority) }, \
         { "seqNum", NULL, MACE_TYPE_INT8_T, 0, 75, offsetof(mace_auction_bid_bundle_next_part_t, seqNum) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_BID_BUNDLE_NEXT_PART { \
    "AUCTION_BID_BUNDLE_NEXT_PART", \
    13, \
    {  { "requestFrom", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_bid_bundle_next_part_t, requestFrom) }, \
         { "agentID", NULL, MACE_TYPE_UINT64_T, 0, 8, offsetof(mace_auction_bid_bundle_next_part_t, agentID) }, \
         { "bundleGenTime", NULL, MACE_TYPE_DOUBLE, 0, 16, offsetof(mace_auction_bid_bundle_next_part_t, bundleGenTime) }, \
         { "utility", NULL, MACE_TYPE_DOUBLE, 0, 24, offsetof(mace_auction_bid_bundle_next_part_t, utility) }, \
         { "work", NULL, MACE_TYPE_DOUBLE, 0, 32, offsetof(mace_auction_bid_bundle_next_part_t, work) }, \
         { "cost", NULL, MACE_TYPE_DOUBLE, 0, 40, offsetof(mace_auction_bid_bundle_next_part_t, cost) }, \
         { "reward", NULL, MACE_TYPE_DOUBLE, 0, 48, offsetof(mace_auction_bid_bundle_next_part_t, reward) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 56, offsetof(mace_auction_bid_bundle_next_part_t, creatorID) }, \
         { "taskGenTime", NULL, MACE_TYPE_DOUBLE, 0, 64, offsetof(mace_auction_bid_bundle_next_part_t, taskGenTime) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 72, offsetof(mace_auction_bid_bundle_next_part_t, taskID) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 73, offsetof(mace_auction_bid_bundle_next_part_t, type) }, \
         { "priority", NULL, MACE_TYPE_UINT8_T, 0, 74, offsetof(mace_auction_bid_bundle_next_part_t, priority) }, \
         { "seqNum", NULL, MACE_TYPE_INT8_T, 0, 75, offsetof(mace_auction_bid_bundle_next_part_t, seqNum) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_bid_bundle_next_part message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param requestFrom ID of agent requesting the bundle
 * @param agentID ID of agent generating the bundle
 * @param bundleGenTime Bundle generation time
 * @param utility Utility
 * @param work Work
 * @param cost Cost
 * @param reward Reward
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param taskGenTime Task generation time
 * @param type Task type
 * @param priority Priority
 * @param seqNum Sequence number
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_bid_bundle_next_part_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t requestFrom, uint64_t agentID, double bundleGenTime, double utility, double work, double cost, double reward, uint64_t creatorID, uint8_t taskID, double taskGenTime, uint8_t type, uint8_t priority, int8_t seqNum)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN];
    _mace_put_uint64_t(buf, 0, requestFrom);
    _mace_put_uint64_t(buf, 8, agentID);
    _mace_put_double(buf, 16, bundleGenTime);
    _mace_put_double(buf, 24, utility);
    _mace_put_double(buf, 32, work);
    _mace_put_double(buf, 40, cost);
    _mace_put_double(buf, 48, reward);
    _mace_put_uint64_t(buf, 56, creatorID);
    _mace_put_double(buf, 64, taskGenTime);
    _mace_put_uint8_t(buf, 72, taskID);
    _mace_put_uint8_t(buf, 73, type);
    _mace_put_uint8_t(buf, 74, priority);
    _mace_put_int8_t(buf, 75, seqNum);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN);
#else
    mace_auction_bid_bundle_next_part_t packet;
    packet.requestFrom = requestFrom;
    packet.agentID = agentID;
    packet.bundleGenTime = bundleGenTime;
    packet.utility = utility;
    packet.work = work;
    packet.cost = cost;
    packet.reward = reward;
    packet.creatorID = creatorID;
    packet.taskGenTime = taskGenTime;
    packet.taskID = taskID;
    packet.type = type;
    packet.priority = priority;
    packet.seqNum = seqNum;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_CRC);
}

/**
 * @brief Pack a auction_bid_bundle_next_part message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param requestFrom ID of agent requesting the bundle
 * @param agentID ID of agent generating the bundle
 * @param bundleGenTime Bundle generation time
 * @param utility Utility
 * @param work Work
 * @param cost Cost
 * @param reward Reward
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param taskGenTime Task generation time
 * @param type Task type
 * @param priority Priority
 * @param seqNum Sequence number
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_bid_bundle_next_part_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t requestFrom,uint64_t agentID,double bundleGenTime,double utility,double work,double cost,double reward,uint64_t creatorID,uint8_t taskID,double taskGenTime,uint8_t type,uint8_t priority,int8_t seqNum)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN];
    _mace_put_uint64_t(buf, 0, requestFrom);
    _mace_put_uint64_t(buf, 8, agentID);
    _mace_put_double(buf, 16, bundleGenTime);
    _mace_put_double(buf, 24, utility);
    _mace_put_double(buf, 32, work);
    _mace_put_double(buf, 40, cost);
    _mace_put_double(buf, 48, reward);
    _mace_put_uint64_t(buf, 56, creatorID);
    _mace_put_double(buf, 64, taskGenTime);
    _mace_put_uint8_t(buf, 72, taskID);
    _mace_put_uint8_t(buf, 73, type);
    _mace_put_uint8_t(buf, 74, priority);
    _mace_put_int8_t(buf, 75, seqNum);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN);
#else
    mace_auction_bid_bundle_next_part_t packet;
    packet.requestFrom = requestFrom;
    packet.agentID = agentID;
    packet.bundleGenTime = bundleGenTime;
    packet.utility = utility;
    packet.work = work;
    packet.cost = cost;
    packet.reward = reward;
    packet.creatorID = creatorID;
    packet.taskGenTime = taskGenTime;
    packet.taskID = taskID;
    packet.type = type;
    packet.priority = priority;
    packet.seqNum = seqNum;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_CRC);
}

/**
 * @brief Encode a auction_bid_bundle_next_part struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_bid_bundle_next_part C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_bid_bundle_next_part_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_bid_bundle_next_part_t* auction_bid_bundle_next_part)
{
    return mace_msg_auction_bid_bundle_next_part_pack(system_id, component_id, msg, auction_bid_bundle_next_part->requestFrom, auction_bid_bundle_next_part->agentID, auction_bid_bundle_next_part->bundleGenTime, auction_bid_bundle_next_part->utility, auction_bid_bundle_next_part->work, auction_bid_bundle_next_part->cost, auction_bid_bundle_next_part->reward, auction_bid_bundle_next_part->creatorID, auction_bid_bundle_next_part->taskID, auction_bid_bundle_next_part->taskGenTime, auction_bid_bundle_next_part->type, auction_bid_bundle_next_part->priority, auction_bid_bundle_next_part->seqNum);
}

/**
 * @brief Encode a auction_bid_bundle_next_part struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_bid_bundle_next_part C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_bid_bundle_next_part_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_bid_bundle_next_part_t* auction_bid_bundle_next_part)
{
    return mace_msg_auction_bid_bundle_next_part_pack_chan(system_id, component_id, chan, msg, auction_bid_bundle_next_part->requestFrom, auction_bid_bundle_next_part->agentID, auction_bid_bundle_next_part->bundleGenTime, auction_bid_bundle_next_part->utility, auction_bid_bundle_next_part->work, auction_bid_bundle_next_part->cost, auction_bid_bundle_next_part->reward, auction_bid_bundle_next_part->creatorID, auction_bid_bundle_next_part->taskID, auction_bid_bundle_next_part->taskGenTime, auction_bid_bundle_next_part->type, auction_bid_bundle_next_part->priority, auction_bid_bundle_next_part->seqNum);
}

/**
 * @brief Send a auction_bid_bundle_next_part message
 * @param chan MAVLink channel to send the message
 *
 * @param requestFrom ID of agent requesting the bundle
 * @param agentID ID of agent generating the bundle
 * @param bundleGenTime Bundle generation time
 * @param utility Utility
 * @param work Work
 * @param cost Cost
 * @param reward Reward
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param taskGenTime Task generation time
 * @param type Task type
 * @param priority Priority
 * @param seqNum Sequence number
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_bid_bundle_next_part_send(mace_channel_t chan, uint64_t requestFrom, uint64_t agentID, double bundleGenTime, double utility, double work, double cost, double reward, uint64_t creatorID, uint8_t taskID, double taskGenTime, uint8_t type, uint8_t priority, int8_t seqNum)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN];
    _mace_put_uint64_t(buf, 0, requestFrom);
    _mace_put_uint64_t(buf, 8, agentID);
    _mace_put_double(buf, 16, bundleGenTime);
    _mace_put_double(buf, 24, utility);
    _mace_put_double(buf, 32, work);
    _mace_put_double(buf, 40, cost);
    _mace_put_double(buf, 48, reward);
    _mace_put_uint64_t(buf, 56, creatorID);
    _mace_put_double(buf, 64, taskGenTime);
    _mace_put_uint8_t(buf, 72, taskID);
    _mace_put_uint8_t(buf, 73, type);
    _mace_put_uint8_t(buf, 74, priority);
    _mace_put_int8_t(buf, 75, seqNum);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART, buf, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_CRC);
#else
    mace_auction_bid_bundle_next_part_t packet;
    packet.requestFrom = requestFrom;
    packet.agentID = agentID;
    packet.bundleGenTime = bundleGenTime;
    packet.utility = utility;
    packet.work = work;
    packet.cost = cost;
    packet.reward = reward;
    packet.creatorID = creatorID;
    packet.taskGenTime = taskGenTime;
    packet.taskID = taskID;
    packet.type = type;
    packet.priority = priority;
    packet.seqNum = seqNum;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART, (const char *)&packet, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_CRC);
#endif
}

/**
 * @brief Send a auction_bid_bundle_next_part message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_bid_bundle_next_part_send_struct(mace_channel_t chan, const mace_auction_bid_bundle_next_part_t* auction_bid_bundle_next_part)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_bid_bundle_next_part_send(chan, auction_bid_bundle_next_part->requestFrom, auction_bid_bundle_next_part->agentID, auction_bid_bundle_next_part->bundleGenTime, auction_bid_bundle_next_part->utility, auction_bid_bundle_next_part->work, auction_bid_bundle_next_part->cost, auction_bid_bundle_next_part->reward, auction_bid_bundle_next_part->creatorID, auction_bid_bundle_next_part->taskID, auction_bid_bundle_next_part->taskGenTime, auction_bid_bundle_next_part->type, auction_bid_bundle_next_part->priority, auction_bid_bundle_next_part->seqNum);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART, (const char *)auction_bid_bundle_next_part, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_bid_bundle_next_part_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t requestFrom, uint64_t agentID, double bundleGenTime, double utility, double work, double cost, double reward, uint64_t creatorID, uint8_t taskID, double taskGenTime, uint8_t type, uint8_t priority, int8_t seqNum)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, requestFrom);
    _mace_put_uint64_t(buf, 8, agentID);
    _mace_put_double(buf, 16, bundleGenTime);
    _mace_put_double(buf, 24, utility);
    _mace_put_double(buf, 32, work);
    _mace_put_double(buf, 40, cost);
    _mace_put_double(buf, 48, reward);
    _mace_put_uint64_t(buf, 56, creatorID);
    _mace_put_double(buf, 64, taskGenTime);
    _mace_put_uint8_t(buf, 72, taskID);
    _mace_put_uint8_t(buf, 73, type);
    _mace_put_uint8_t(buf, 74, priority);
    _mace_put_int8_t(buf, 75, seqNum);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART, buf, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_CRC);
#else
    mace_auction_bid_bundle_next_part_t *packet = (mace_auction_bid_bundle_next_part_t *)msgbuf;
    packet->requestFrom = requestFrom;
    packet->agentID = agentID;
    packet->bundleGenTime = bundleGenTime;
    packet->utility = utility;
    packet->work = work;
    packet->cost = cost;
    packet->reward = reward;
    packet->creatorID = creatorID;
    packet->taskGenTime = taskGenTime;
    packet->taskID = taskID;
    packet->type = type;
    packet->priority = priority;
    packet->seqNum = seqNum;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART, (const char *)packet, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_BID_BUNDLE_NEXT_PART UNPACKING


/**
 * @brief Get field requestFrom from auction_bid_bundle_next_part message
 *
 * @return ID of agent requesting the bundle
 */
static inline uint64_t mace_msg_auction_bid_bundle_next_part_get_requestFrom(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field agentID from auction_bid_bundle_next_part message
 *
 * @return ID of agent generating the bundle
 */
static inline uint64_t mace_msg_auction_bid_bundle_next_part_get_agentID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field bundleGenTime from auction_bid_bundle_next_part message
 *
 * @return Bundle generation time
 */
static inline double mace_msg_auction_bid_bundle_next_part_get_bundleGenTime(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  16);
}

/**
 * @brief Get field utility from auction_bid_bundle_next_part message
 *
 * @return Utility
 */
static inline double mace_msg_auction_bid_bundle_next_part_get_utility(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  24);
}

/**
 * @brief Get field work from auction_bid_bundle_next_part message
 *
 * @return Work
 */
static inline double mace_msg_auction_bid_bundle_next_part_get_work(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  32);
}

/**
 * @brief Get field cost from auction_bid_bundle_next_part message
 *
 * @return Cost
 */
static inline double mace_msg_auction_bid_bundle_next_part_get_cost(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  40);
}

/**
 * @brief Get field reward from auction_bid_bundle_next_part message
 *
 * @return Reward
 */
static inline double mace_msg_auction_bid_bundle_next_part_get_reward(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  48);
}

/**
 * @brief Get field creatorID from auction_bid_bundle_next_part message
 *
 * @return Task creator ID
 */
static inline uint64_t mace_msg_auction_bid_bundle_next_part_get_creatorID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  56);
}

/**
 * @brief Get field taskID from auction_bid_bundle_next_part message
 *
 * @return Creator local task ID
 */
static inline uint8_t mace_msg_auction_bid_bundle_next_part_get_taskID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  72);
}

/**
 * @brief Get field taskGenTime from auction_bid_bundle_next_part message
 *
 * @return Task generation time
 */
static inline double mace_msg_auction_bid_bundle_next_part_get_taskGenTime(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  64);
}

/**
 * @brief Get field type from auction_bid_bundle_next_part message
 *
 * @return Task type
 */
static inline uint8_t mace_msg_auction_bid_bundle_next_part_get_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  73);
}

/**
 * @brief Get field priority from auction_bid_bundle_next_part message
 *
 * @return Priority
 */
static inline uint8_t mace_msg_auction_bid_bundle_next_part_get_priority(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  74);
}

/**
 * @brief Get field seqNum from auction_bid_bundle_next_part message
 *
 * @return Sequence number
 */
static inline int8_t mace_msg_auction_bid_bundle_next_part_get_seqNum(const mace_message_t* msg)
{
    return _MACE_RETURN_int8_t(msg,  75);
}

/**
 * @brief Decode a auction_bid_bundle_next_part message into a struct
 *
 * @param msg The message to decode
 * @param auction_bid_bundle_next_part C-struct to decode the message contents into
 */
static inline void mace_msg_auction_bid_bundle_next_part_decode(const mace_message_t* msg, mace_auction_bid_bundle_next_part_t* auction_bid_bundle_next_part)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_bid_bundle_next_part->requestFrom = mace_msg_auction_bid_bundle_next_part_get_requestFrom(msg);
    auction_bid_bundle_next_part->agentID = mace_msg_auction_bid_bundle_next_part_get_agentID(msg);
    auction_bid_bundle_next_part->bundleGenTime = mace_msg_auction_bid_bundle_next_part_get_bundleGenTime(msg);
    auction_bid_bundle_next_part->utility = mace_msg_auction_bid_bundle_next_part_get_utility(msg);
    auction_bid_bundle_next_part->work = mace_msg_auction_bid_bundle_next_part_get_work(msg);
    auction_bid_bundle_next_part->cost = mace_msg_auction_bid_bundle_next_part_get_cost(msg);
    auction_bid_bundle_next_part->reward = mace_msg_auction_bid_bundle_next_part_get_reward(msg);
    auction_bid_bundle_next_part->creatorID = mace_msg_auction_bid_bundle_next_part_get_creatorID(msg);
    auction_bid_bundle_next_part->taskGenTime = mace_msg_auction_bid_bundle_next_part_get_taskGenTime(msg);
    auction_bid_bundle_next_part->taskID = mace_msg_auction_bid_bundle_next_part_get_taskID(msg);
    auction_bid_bundle_next_part->type = mace_msg_auction_bid_bundle_next_part_get_type(msg);
    auction_bid_bundle_next_part->priority = mace_msg_auction_bid_bundle_next_part_get_priority(msg);
    auction_bid_bundle_next_part->seqNum = mace_msg_auction_bid_bundle_next_part_get_seqNum(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN? msg->len : MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN;
        memset(auction_bid_bundle_next_part, 0, MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_LEN);
    memcpy(auction_bid_bundle_next_part, _MACE_PAYLOAD(msg), len);
#endif
}
