#pragma once
// MESSAGE AUCTION_SCRUB_BID_ACK PACKING

#define MACE_MSG_ID_AUCTION_SCRUB_BID_ACK 10014

MACEPACKED(
typedef struct __mace_auction_scrub_bid_ack_t {
 uint64_t agentID; /*< Agent ID*/
 double timestamp; /*< Timestamp*/
 uint64_t creatorID; /*< Task creator ID*/
 uint8_t taskID; /*< Creator local task ID*/
}) mace_auction_scrub_bid_ack_t;

#define MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN 25
#define MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_MIN_LEN 25
#define MACE_MSG_ID_10014_LEN 25
#define MACE_MSG_ID_10014_MIN_LEN 25

#define MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_CRC 231
#define MACE_MSG_ID_10014_CRC 231



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_SCRUB_BID_ACK { \
    10014, \
    "AUCTION_SCRUB_BID_ACK", \
    4, \
    {  { "agentID", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_scrub_bid_ack_t, agentID) }, \
         { "timestamp", NULL, MACE_TYPE_DOUBLE, 0, 8, offsetof(mace_auction_scrub_bid_ack_t, timestamp) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 16, offsetof(mace_auction_scrub_bid_ack_t, creatorID) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 24, offsetof(mace_auction_scrub_bid_ack_t, taskID) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_SCRUB_BID_ACK { \
    "AUCTION_SCRUB_BID_ACK", \
    4, \
    {  { "agentID", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_scrub_bid_ack_t, agentID) }, \
         { "timestamp", NULL, MACE_TYPE_DOUBLE, 0, 8, offsetof(mace_auction_scrub_bid_ack_t, timestamp) }, \
         { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 16, offsetof(mace_auction_scrub_bid_ack_t, creatorID) }, \
         { "taskID", NULL, MACE_TYPE_UINT8_T, 0, 24, offsetof(mace_auction_scrub_bid_ack_t, taskID) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_scrub_bid_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param agentID Agent ID
 * @param timestamp Timestamp
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_scrub_bid_ack_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t agentID, double timestamp, uint64_t creatorID, uint8_t taskID)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN];
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, timestamp);
    _mace_put_uint64_t(buf, 16, creatorID);
    _mace_put_uint8_t(buf, 24, taskID);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN);
#else
    mace_auction_scrub_bid_ack_t packet;
    packet.agentID = agentID;
    packet.timestamp = timestamp;
    packet.creatorID = creatorID;
    packet.taskID = taskID;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_SCRUB_BID_ACK;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_MIN_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_CRC);
}

/**
 * @brief Pack a auction_scrub_bid_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param agentID Agent ID
 * @param timestamp Timestamp
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_scrub_bid_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t agentID,double timestamp,uint64_t creatorID,uint8_t taskID)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN];
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, timestamp);
    _mace_put_uint64_t(buf, 16, creatorID);
    _mace_put_uint8_t(buf, 24, taskID);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN);
#else
    mace_auction_scrub_bid_ack_t packet;
    packet.agentID = agentID;
    packet.timestamp = timestamp;
    packet.creatorID = creatorID;
    packet.taskID = taskID;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_SCRUB_BID_ACK;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_MIN_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_CRC);
}

/**
 * @brief Encode a auction_scrub_bid_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_scrub_bid_ack C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_scrub_bid_ack_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_scrub_bid_ack_t* auction_scrub_bid_ack)
{
    return mace_msg_auction_scrub_bid_ack_pack(system_id, component_id, msg, auction_scrub_bid_ack->agentID, auction_scrub_bid_ack->timestamp, auction_scrub_bid_ack->creatorID, auction_scrub_bid_ack->taskID);
}

/**
 * @brief Encode a auction_scrub_bid_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_scrub_bid_ack C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_scrub_bid_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_scrub_bid_ack_t* auction_scrub_bid_ack)
{
    return mace_msg_auction_scrub_bid_ack_pack_chan(system_id, component_id, chan, msg, auction_scrub_bid_ack->agentID, auction_scrub_bid_ack->timestamp, auction_scrub_bid_ack->creatorID, auction_scrub_bid_ack->taskID);
}

/**
 * @brief Send a auction_scrub_bid_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param agentID Agent ID
 * @param timestamp Timestamp
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_scrub_bid_ack_send(mace_channel_t chan, uint64_t agentID, double timestamp, uint64_t creatorID, uint8_t taskID)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN];
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, timestamp);
    _mace_put_uint64_t(buf, 16, creatorID);
    _mace_put_uint8_t(buf, 24, taskID);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK, buf, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_MIN_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_CRC);
#else
    mace_auction_scrub_bid_ack_t packet;
    packet.agentID = agentID;
    packet.timestamp = timestamp;
    packet.creatorID = creatorID;
    packet.taskID = taskID;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK, (const char *)&packet, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_MIN_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_CRC);
#endif
}

/**
 * @brief Send a auction_scrub_bid_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_scrub_bid_ack_send_struct(mace_channel_t chan, const mace_auction_scrub_bid_ack_t* auction_scrub_bid_ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_scrub_bid_ack_send(chan, auction_scrub_bid_ack->agentID, auction_scrub_bid_ack->timestamp, auction_scrub_bid_ack->creatorID, auction_scrub_bid_ack->taskID);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK, (const char *)auction_scrub_bid_ack, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_MIN_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_scrub_bid_ack_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t agentID, double timestamp, uint64_t creatorID, uint8_t taskID)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, timestamp);
    _mace_put_uint64_t(buf, 16, creatorID);
    _mace_put_uint8_t(buf, 24, taskID);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK, buf, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_MIN_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_CRC);
#else
    mace_auction_scrub_bid_ack_t *packet = (mace_auction_scrub_bid_ack_t *)msgbuf;
    packet->agentID = agentID;
    packet->timestamp = timestamp;
    packet->creatorID = creatorID;
    packet->taskID = taskID;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK, (const char *)packet, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_MIN_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_SCRUB_BID_ACK UNPACKING


/**
 * @brief Get field agentID from auction_scrub_bid_ack message
 *
 * @return Agent ID
 */
static inline uint64_t mace_msg_auction_scrub_bid_ack_get_agentID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field timestamp from auction_scrub_bid_ack message
 *
 * @return Timestamp
 */
static inline double mace_msg_auction_scrub_bid_ack_get_timestamp(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  8);
}

/**
 * @brief Get field creatorID from auction_scrub_bid_ack message
 *
 * @return Task creator ID
 */
static inline uint64_t mace_msg_auction_scrub_bid_ack_get_creatorID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Get field taskID from auction_scrub_bid_ack message
 *
 * @return Creator local task ID
 */
static inline uint8_t mace_msg_auction_scrub_bid_ack_get_taskID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Decode a auction_scrub_bid_ack message into a struct
 *
 * @param msg The message to decode
 * @param auction_scrub_bid_ack C-struct to decode the message contents into
 */
static inline void mace_msg_auction_scrub_bid_ack_decode(const mace_message_t* msg, mace_auction_scrub_bid_ack_t* auction_scrub_bid_ack)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_scrub_bid_ack->agentID = mace_msg_auction_scrub_bid_ack_get_agentID(msg);
    auction_scrub_bid_ack->timestamp = mace_msg_auction_scrub_bid_ack_get_timestamp(msg);
    auction_scrub_bid_ack->creatorID = mace_msg_auction_scrub_bid_ack_get_creatorID(msg);
    auction_scrub_bid_ack->taskID = mace_msg_auction_scrub_bid_ack_get_taskID(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN? msg->len : MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN;
        memset(auction_scrub_bid_ack, 0, MACE_MSG_ID_AUCTION_SCRUB_BID_ACK_LEN);
    memcpy(auction_scrub_bid_ack, _MACE_PAYLOAD(msg), len);
#endif
}
