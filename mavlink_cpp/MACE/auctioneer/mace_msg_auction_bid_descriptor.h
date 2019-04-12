#pragma once
// MESSAGE AUCTION_BID_DESCRIPTOR PACKING

#define MACE_MSG_ID_AUCTION_BID_DESCRIPTOR 10011

MACEPACKED(
typedef struct __mace_auction_bid_descriptor_t {
 uint64_t agentID; /*< Agent ID*/
 double bidGenTime; /*< Bid generation time*/
 double utility; /*< Utility*/
 double work; /*< Work*/
 double cost; /*< Cost*/
 double reward; /*< Reward*/
 uint64_t creatorID; /*< Task creator ID*/
 double taskGenTime; /*< Task generation time*/
 double rebroadcastTime; /*< Time this descriptor was rebroadcast*/
 uint8_t taskID; /*< Creator local task ID*/
 uint8_t type; /*< Task type*/
 uint8_t priority; /*< Priority*/
 uint8_t valid; /*< Validity*/
}) mace_auction_bid_descriptor_t;

#define MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN 76
#define MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_MIN_LEN 76
#define MACE_MSG_ID_10011_LEN 76
#define MACE_MSG_ID_10011_MIN_LEN 76

#define MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_CRC 35
#define MACE_MSG_ID_10011_CRC 35



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_BID_DESCRIPTOR { \
    10011, \
    "AUCTION_BID_DESCRIPTOR", \
    13, \
    {  { "agentID", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_bid_descriptor_t, agentID) }, \
         { "bidGenTime", NULL, MACE_TYPE_DOUBLE, 0, 8, offsetof(mace_auction_bid_descriptor_t, bidGenTime) }, \
         { "utility", NULL, MACE_TYPE_DOUBLE, 0, 16, offsetof(mace_auction_bid_descriptor_t, utility) }, \
         { "work", NULL, MACE_TYPE_DOUBLE, 0, 24, offsetof(mace_auction_bid_descriptor_t, work) }, \
         { "cost", NULL, MACE_TYPE_DOUBLE, 0, 32, offsetof(mace_auction_bid_descriptor_t, cost) }, \
         { "reward", NULL, MACE_TYPE_DOUBLE, 0, 40, offsetof(mace_auction_bid_descriptor_t, reward) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 48, offsetof(mace_auction_bid_descriptor_t, creatorID) }, \
         { "taskGenTime", NULL, MACE_TYPE_DOUBLE, 0, 56, offsetof(mace_auction_bid_descriptor_t, taskGenTime) }, \
         { "rebroadcastTime", NULL, MACE_TYPE_DOUBLE, 0, 64, offsetof(mace_auction_bid_descriptor_t, rebroadcastTime) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 72, offsetof(mace_auction_bid_descriptor_t, taskID) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 73, offsetof(mace_auction_bid_descriptor_t, type) }, \
         { "priority", NULL, MACE_TYPE_UINT8_T, 0, 74, offsetof(mace_auction_bid_descriptor_t, priority) }, \
         { "valid", NULL, MACE_TYPE_UINT8_T, 0, 75, offsetof(mace_auction_bid_descriptor_t, valid) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_BID_DESCRIPTOR { \
    "AUCTION_BID_DESCRIPTOR", \
    13, \
    {  { "agentID", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_bid_descriptor_t, agentID) }, \
         { "bidGenTime", NULL, MACE_TYPE_DOUBLE, 0, 8, offsetof(mace_auction_bid_descriptor_t, bidGenTime) }, \
         { "utility", NULL, MACE_TYPE_DOUBLE, 0, 16, offsetof(mace_auction_bid_descriptor_t, utility) }, \
         { "work", NULL, MACE_TYPE_DOUBLE, 0, 24, offsetof(mace_auction_bid_descriptor_t, work) }, \
         { "cost", NULL, MACE_TYPE_DOUBLE, 0, 32, offsetof(mace_auction_bid_descriptor_t, cost) }, \
         { "reward", NULL, MACE_TYPE_DOUBLE, 0, 40, offsetof(mace_auction_bid_descriptor_t, reward) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 48, offsetof(mace_auction_bid_descriptor_t, creatorID) }, \
         { "taskGenTime", NULL, MACE_TYPE_DOUBLE, 0, 56, offsetof(mace_auction_bid_descriptor_t, taskGenTime) }, \
         { "rebroadcastTime", NULL, MACE_TYPE_DOUBLE, 0, 64, offsetof(mace_auction_bid_descriptor_t, rebroadcastTime) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 72, offsetof(mace_auction_bid_descriptor_t, taskID) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 73, offsetof(mace_auction_bid_descriptor_t, type) }, \
         { "priority", NULL, MACE_TYPE_UINT8_T, 0, 74, offsetof(mace_auction_bid_descriptor_t, priority) }, \
         { "valid", NULL, MACE_TYPE_UINT8_T, 0, 75, offsetof(mace_auction_bid_descriptor_t, valid) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_bid_descriptor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param agentID Agent ID
 * @param bidGenTime Bid generation time
 * @param utility Utility
 * @param work Work
 * @param cost Cost
 * @param reward Reward
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param taskGenTime Task generation time
 * @param type Task type
 * @param priority Priority
 * @param valid Validity
 * @param rebroadcastTime Time this descriptor was rebroadcast
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_bid_descriptor_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t agentID, double bidGenTime, double utility, double work, double cost, double reward, uint64_t creatorID, uint8_t taskID, double taskGenTime, uint8_t type, uint8_t priority, uint8_t valid, double rebroadcastTime)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN];
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, bidGenTime);
    _mace_put_double(buf, 16, utility);
    _mace_put_double(buf, 24, work);
    _mace_put_double(buf, 32, cost);
    _mace_put_double(buf, 40, reward);
    _mace_put_uint64_t(buf, 48, creatorID);
    _mace_put_double(buf, 56, taskGenTime);
    _mace_put_double(buf, 64, rebroadcastTime);
    _mace_put_uint8_t(buf, 72, taskID);
    _mace_put_uint8_t(buf, 73, type);
    _mace_put_uint8_t(buf, 74, priority);
    _mace_put_uint8_t(buf, 75, valid);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN);
#else
    mace_auction_bid_descriptor_t packet;
    packet.agentID = agentID;
    packet.bidGenTime = bidGenTime;
    packet.utility = utility;
    packet.work = work;
    packet.cost = cost;
    packet.reward = reward;
    packet.creatorID = creatorID;
    packet.taskGenTime = taskGenTime;
    packet.rebroadcastTime = rebroadcastTime;
    packet.taskID = taskID;
    packet.type = type;
    packet.priority = priority;
    packet.valid = valid;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_BID_DESCRIPTOR;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_CRC);
}

/**
 * @brief Pack a auction_bid_descriptor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param agentID Agent ID
 * @param bidGenTime Bid generation time
 * @param utility Utility
 * @param work Work
 * @param cost Cost
 * @param reward Reward
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param taskGenTime Task generation time
 * @param type Task type
 * @param priority Priority
 * @param valid Validity
 * @param rebroadcastTime Time this descriptor was rebroadcast
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_bid_descriptor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t agentID,double bidGenTime,double utility,double work,double cost,double reward,uint64_t creatorID,uint8_t taskID,double taskGenTime,uint8_t type,uint8_t priority,uint8_t valid,double rebroadcastTime)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN];
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, bidGenTime);
    _mace_put_double(buf, 16, utility);
    _mace_put_double(buf, 24, work);
    _mace_put_double(buf, 32, cost);
    _mace_put_double(buf, 40, reward);
    _mace_put_uint64_t(buf, 48, creatorID);
    _mace_put_double(buf, 56, taskGenTime);
    _mace_put_double(buf, 64, rebroadcastTime);
    _mace_put_uint8_t(buf, 72, taskID);
    _mace_put_uint8_t(buf, 73, type);
    _mace_put_uint8_t(buf, 74, priority);
    _mace_put_uint8_t(buf, 75, valid);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN);
#else
    mace_auction_bid_descriptor_t packet;
    packet.agentID = agentID;
    packet.bidGenTime = bidGenTime;
    packet.utility = utility;
    packet.work = work;
    packet.cost = cost;
    packet.reward = reward;
    packet.creatorID = creatorID;
    packet.taskGenTime = taskGenTime;
    packet.rebroadcastTime = rebroadcastTime;
    packet.taskID = taskID;
    packet.type = type;
    packet.priority = priority;
    packet.valid = valid;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_BID_DESCRIPTOR;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_CRC);
}

/**
 * @brief Encode a auction_bid_descriptor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_bid_descriptor C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_bid_descriptor_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_bid_descriptor_t* auction_bid_descriptor)
{
    return mace_msg_auction_bid_descriptor_pack(system_id, component_id, msg, auction_bid_descriptor->agentID, auction_bid_descriptor->bidGenTime, auction_bid_descriptor->utility, auction_bid_descriptor->work, auction_bid_descriptor->cost, auction_bid_descriptor->reward, auction_bid_descriptor->creatorID, auction_bid_descriptor->taskID, auction_bid_descriptor->taskGenTime, auction_bid_descriptor->type, auction_bid_descriptor->priority, auction_bid_descriptor->valid, auction_bid_descriptor->rebroadcastTime);
}

/**
 * @brief Encode a auction_bid_descriptor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_bid_descriptor C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_bid_descriptor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_bid_descriptor_t* auction_bid_descriptor)
{
    return mace_msg_auction_bid_descriptor_pack_chan(system_id, component_id, chan, msg, auction_bid_descriptor->agentID, auction_bid_descriptor->bidGenTime, auction_bid_descriptor->utility, auction_bid_descriptor->work, auction_bid_descriptor->cost, auction_bid_descriptor->reward, auction_bid_descriptor->creatorID, auction_bid_descriptor->taskID, auction_bid_descriptor->taskGenTime, auction_bid_descriptor->type, auction_bid_descriptor->priority, auction_bid_descriptor->valid, auction_bid_descriptor->rebroadcastTime);
}

/**
 * @brief Send a auction_bid_descriptor message
 * @param chan MAVLink channel to send the message
 *
 * @param agentID Agent ID
 * @param bidGenTime Bid generation time
 * @param utility Utility
 * @param work Work
 * @param cost Cost
 * @param reward Reward
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param taskGenTime Task generation time
 * @param type Task type
 * @param priority Priority
 * @param valid Validity
 * @param rebroadcastTime Time this descriptor was rebroadcast
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_bid_descriptor_send(mace_channel_t chan, uint64_t agentID, double bidGenTime, double utility, double work, double cost, double reward, uint64_t creatorID, uint8_t taskID, double taskGenTime, uint8_t type, uint8_t priority, uint8_t valid, double rebroadcastTime)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN];
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, bidGenTime);
    _mace_put_double(buf, 16, utility);
    _mace_put_double(buf, 24, work);
    _mace_put_double(buf, 32, cost);
    _mace_put_double(buf, 40, reward);
    _mace_put_uint64_t(buf, 48, creatorID);
    _mace_put_double(buf, 56, taskGenTime);
    _mace_put_double(buf, 64, rebroadcastTime);
    _mace_put_uint8_t(buf, 72, taskID);
    _mace_put_uint8_t(buf, 73, type);
    _mace_put_uint8_t(buf, 74, priority);
    _mace_put_uint8_t(buf, 75, valid);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR, buf, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_CRC);
#else
    mace_auction_bid_descriptor_t packet;
    packet.agentID = agentID;
    packet.bidGenTime = bidGenTime;
    packet.utility = utility;
    packet.work = work;
    packet.cost = cost;
    packet.reward = reward;
    packet.creatorID = creatorID;
    packet.taskGenTime = taskGenTime;
    packet.rebroadcastTime = rebroadcastTime;
    packet.taskID = taskID;
    packet.type = type;
    packet.priority = priority;
    packet.valid = valid;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR, (const char *)&packet, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_CRC);
#endif
}

/**
 * @brief Send a auction_bid_descriptor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_bid_descriptor_send_struct(mace_channel_t chan, const mace_auction_bid_descriptor_t* auction_bid_descriptor)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_bid_descriptor_send(chan, auction_bid_descriptor->agentID, auction_bid_descriptor->bidGenTime, auction_bid_descriptor->utility, auction_bid_descriptor->work, auction_bid_descriptor->cost, auction_bid_descriptor->reward, auction_bid_descriptor->creatorID, auction_bid_descriptor->taskID, auction_bid_descriptor->taskGenTime, auction_bid_descriptor->type, auction_bid_descriptor->priority, auction_bid_descriptor->valid, auction_bid_descriptor->rebroadcastTime);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR, (const char *)auction_bid_descriptor, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_bid_descriptor_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t agentID, double bidGenTime, double utility, double work, double cost, double reward, uint64_t creatorID, uint8_t taskID, double taskGenTime, uint8_t type, uint8_t priority, uint8_t valid, double rebroadcastTime)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, bidGenTime);
    _mace_put_double(buf, 16, utility);
    _mace_put_double(buf, 24, work);
    _mace_put_double(buf, 32, cost);
    _mace_put_double(buf, 40, reward);
    _mace_put_uint64_t(buf, 48, creatorID);
    _mace_put_double(buf, 56, taskGenTime);
    _mace_put_double(buf, 64, rebroadcastTime);
    _mace_put_uint8_t(buf, 72, taskID);
    _mace_put_uint8_t(buf, 73, type);
    _mace_put_uint8_t(buf, 74, priority);
    _mace_put_uint8_t(buf, 75, valid);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR, buf, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_CRC);
#else
    mace_auction_bid_descriptor_t *packet = (mace_auction_bid_descriptor_t *)msgbuf;
    packet->agentID = agentID;
    packet->bidGenTime = bidGenTime;
    packet->utility = utility;
    packet->work = work;
    packet->cost = cost;
    packet->reward = reward;
    packet->creatorID = creatorID;
    packet->taskGenTime = taskGenTime;
    packet->rebroadcastTime = rebroadcastTime;
    packet->taskID = taskID;
    packet->type = type;
    packet->priority = priority;
    packet->valid = valid;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR, (const char *)packet, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_MIN_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_BID_DESCRIPTOR UNPACKING


/**
 * @brief Get field agentID from auction_bid_descriptor message
 *
 * @return Agent ID
 */
static inline uint64_t mace_msg_auction_bid_descriptor_get_agentID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field bidGenTime from auction_bid_descriptor message
 *
 * @return Bid generation time
 */
static inline double mace_msg_auction_bid_descriptor_get_bidGenTime(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  8);
}

/**
 * @brief Get field utility from auction_bid_descriptor message
 *
 * @return Utility
 */
static inline double mace_msg_auction_bid_descriptor_get_utility(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  16);
}

/**
 * @brief Get field work from auction_bid_descriptor message
 *
 * @return Work
 */
static inline double mace_msg_auction_bid_descriptor_get_work(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  24);
}

/**
 * @brief Get field cost from auction_bid_descriptor message
 *
 * @return Cost
 */
static inline double mace_msg_auction_bid_descriptor_get_cost(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  32);
}

/**
 * @brief Get field reward from auction_bid_descriptor message
 *
 * @return Reward
 */
static inline double mace_msg_auction_bid_descriptor_get_reward(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  40);
}

/**
 * @brief Get field creatorID from auction_bid_descriptor message
 *
 * @return Task creator ID
 */
static inline uint64_t mace_msg_auction_bid_descriptor_get_creatorID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  48);
}

/**
 * @brief Get field taskID from auction_bid_descriptor message
 *
 * @return Creator local task ID
 */
static inline uint8_t mace_msg_auction_bid_descriptor_get_taskID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  72);
}

/**
 * @brief Get field taskGenTime from auction_bid_descriptor message
 *
 * @return Task generation time
 */
static inline double mace_msg_auction_bid_descriptor_get_taskGenTime(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  56);
}

/**
 * @brief Get field type from auction_bid_descriptor message
 *
 * @return Task type
 */
static inline uint8_t mace_msg_auction_bid_descriptor_get_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  73);
}

/**
 * @brief Get field priority from auction_bid_descriptor message
 *
 * @return Priority
 */
static inline uint8_t mace_msg_auction_bid_descriptor_get_priority(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  74);
}

/**
 * @brief Get field valid from auction_bid_descriptor message
 *
 * @return Validity
 */
static inline uint8_t mace_msg_auction_bid_descriptor_get_valid(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  75);
}

/**
 * @brief Get field rebroadcastTime from auction_bid_descriptor message
 *
 * @return Time this descriptor was rebroadcast
 */
static inline double mace_msg_auction_bid_descriptor_get_rebroadcastTime(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  64);
}

/**
 * @brief Decode a auction_bid_descriptor message into a struct
 *
 * @param msg The message to decode
 * @param auction_bid_descriptor C-struct to decode the message contents into
 */
static inline void mace_msg_auction_bid_descriptor_decode(const mace_message_t* msg, mace_auction_bid_descriptor_t* auction_bid_descriptor)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_bid_descriptor->agentID = mace_msg_auction_bid_descriptor_get_agentID(msg);
    auction_bid_descriptor->bidGenTime = mace_msg_auction_bid_descriptor_get_bidGenTime(msg);
    auction_bid_descriptor->utility = mace_msg_auction_bid_descriptor_get_utility(msg);
    auction_bid_descriptor->work = mace_msg_auction_bid_descriptor_get_work(msg);
    auction_bid_descriptor->cost = mace_msg_auction_bid_descriptor_get_cost(msg);
    auction_bid_descriptor->reward = mace_msg_auction_bid_descriptor_get_reward(msg);
    auction_bid_descriptor->creatorID = mace_msg_auction_bid_descriptor_get_creatorID(msg);
    auction_bid_descriptor->taskGenTime = mace_msg_auction_bid_descriptor_get_taskGenTime(msg);
    auction_bid_descriptor->rebroadcastTime = mace_msg_auction_bid_descriptor_get_rebroadcastTime(msg);
    auction_bid_descriptor->taskID = mace_msg_auction_bid_descriptor_get_taskID(msg);
    auction_bid_descriptor->type = mace_msg_auction_bid_descriptor_get_type(msg);
    auction_bid_descriptor->priority = mace_msg_auction_bid_descriptor_get_priority(msg);
    auction_bid_descriptor->valid = mace_msg_auction_bid_descriptor_get_valid(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN? msg->len : MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN;
        memset(auction_bid_descriptor, 0, MACE_MSG_ID_AUCTION_BID_DESCRIPTOR_LEN);
    memcpy(auction_bid_descriptor, _MACE_PAYLOAD(msg), len);
#endif
}
