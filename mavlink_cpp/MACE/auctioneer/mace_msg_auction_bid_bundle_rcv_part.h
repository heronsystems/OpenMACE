#pragma once
// MESSAGE AUCTION_BID_BUNDLE_RCV_PART PACKING

#define MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART 10002

MACEPACKED(
typedef struct __mace_auction_bid_bundle_rcv_part_t {
 uint64_t agentID; /*< ID of agent generating the bundle*/
 double bundleGenTime; /*< Bundle generation time*/
 int8_t seqNum; /*< Last sequence number received (-1 to begin upload)*/
}) mace_auction_bid_bundle_rcv_part_t;

#define MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN 17
#define MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_MIN_LEN 17
#define MACE_MSG_ID_10002_LEN 17
#define MACE_MSG_ID_10002_MIN_LEN 17

#define MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_CRC 175
#define MACE_MSG_ID_10002_CRC 175



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_BID_BUNDLE_RCV_PART { \
    10002, \
    "AUCTION_BID_BUNDLE_RCV_PART", \
    3, \
    {  { "agentID", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_bid_bundle_rcv_part_t, agentID) }, \
         { "bundleGenTime", NULL, MACE_TYPE_DOUBLE, 0, 8, offsetof(mace_auction_bid_bundle_rcv_part_t, bundleGenTime) }, \
         { "seqNum", NULL, MACE_TYPE_INT8_T, 0, 16, offsetof(mace_auction_bid_bundle_rcv_part_t, seqNum) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_BID_BUNDLE_RCV_PART { \
    "AUCTION_BID_BUNDLE_RCV_PART", \
    3, \
    {  { "agentID", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_bid_bundle_rcv_part_t, agentID) }, \
         { "bundleGenTime", NULL, MACE_TYPE_DOUBLE, 0, 8, offsetof(mace_auction_bid_bundle_rcv_part_t, bundleGenTime) }, \
         { "seqNum", NULL, MACE_TYPE_INT8_T, 0, 16, offsetof(mace_auction_bid_bundle_rcv_part_t, seqNum) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_bid_bundle_rcv_part message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param agentID ID of agent generating the bundle
 * @param bundleGenTime Bundle generation time
 * @param seqNum Last sequence number received (-1 to begin upload)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_bid_bundle_rcv_part_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t agentID, double bundleGenTime, int8_t seqNum)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN];
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, bundleGenTime);
    _mace_put_int8_t(buf, 16, seqNum);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN);
#else
    mace_auction_bid_bundle_rcv_part_t packet;
    packet.agentID = agentID;
    packet.bundleGenTime = bundleGenTime;
    packet.seqNum = seqNum;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_CRC);
}

/**
 * @brief Pack a auction_bid_bundle_rcv_part message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param agentID ID of agent generating the bundle
 * @param bundleGenTime Bundle generation time
 * @param seqNum Last sequence number received (-1 to begin upload)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_bid_bundle_rcv_part_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t agentID,double bundleGenTime,int8_t seqNum)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN];
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, bundleGenTime);
    _mace_put_int8_t(buf, 16, seqNum);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN);
#else
    mace_auction_bid_bundle_rcv_part_t packet;
    packet.agentID = agentID;
    packet.bundleGenTime = bundleGenTime;
    packet.seqNum = seqNum;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_CRC);
}

/**
 * @brief Encode a auction_bid_bundle_rcv_part struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_bid_bundle_rcv_part C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_bid_bundle_rcv_part_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_bid_bundle_rcv_part_t* auction_bid_bundle_rcv_part)
{
    return mace_msg_auction_bid_bundle_rcv_part_pack(system_id, component_id, msg, auction_bid_bundle_rcv_part->agentID, auction_bid_bundle_rcv_part->bundleGenTime, auction_bid_bundle_rcv_part->seqNum);
}

/**
 * @brief Encode a auction_bid_bundle_rcv_part struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_bid_bundle_rcv_part C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_bid_bundle_rcv_part_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_bid_bundle_rcv_part_t* auction_bid_bundle_rcv_part)
{
    return mace_msg_auction_bid_bundle_rcv_part_pack_chan(system_id, component_id, chan, msg, auction_bid_bundle_rcv_part->agentID, auction_bid_bundle_rcv_part->bundleGenTime, auction_bid_bundle_rcv_part->seqNum);
}

/**
 * @brief Send a auction_bid_bundle_rcv_part message
 * @param chan MAVLink channel to send the message
 *
 * @param agentID ID of agent generating the bundle
 * @param bundleGenTime Bundle generation time
 * @param seqNum Last sequence number received (-1 to begin upload)
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_bid_bundle_rcv_part_send(mace_channel_t chan, uint64_t agentID, double bundleGenTime, int8_t seqNum)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN];
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, bundleGenTime);
    _mace_put_int8_t(buf, 16, seqNum);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART, buf, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_CRC);
#else
    mace_auction_bid_bundle_rcv_part_t packet;
    packet.agentID = agentID;
    packet.bundleGenTime = bundleGenTime;
    packet.seqNum = seqNum;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART, (const char *)&packet, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_CRC);
#endif
}

/**
 * @brief Send a auction_bid_bundle_rcv_part message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_bid_bundle_rcv_part_send_struct(mace_channel_t chan, const mace_auction_bid_bundle_rcv_part_t* auction_bid_bundle_rcv_part)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_bid_bundle_rcv_part_send(chan, auction_bid_bundle_rcv_part->agentID, auction_bid_bundle_rcv_part->bundleGenTime, auction_bid_bundle_rcv_part->seqNum);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART, (const char *)auction_bid_bundle_rcv_part, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_bid_bundle_rcv_part_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t agentID, double bundleGenTime, int8_t seqNum)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, bundleGenTime);
    _mace_put_int8_t(buf, 16, seqNum);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART, buf, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_CRC);
#else
    mace_auction_bid_bundle_rcv_part_t *packet = (mace_auction_bid_bundle_rcv_part_t *)msgbuf;
    packet->agentID = agentID;
    packet->bundleGenTime = bundleGenTime;
    packet->seqNum = seqNum;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART, (const char *)packet, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_MIN_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_BID_BUNDLE_RCV_PART UNPACKING


/**
 * @brief Get field agentID from auction_bid_bundle_rcv_part message
 *
 * @return ID of agent generating the bundle
 */
static inline uint64_t mace_msg_auction_bid_bundle_rcv_part_get_agentID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field bundleGenTime from auction_bid_bundle_rcv_part message
 *
 * @return Bundle generation time
 */
static inline double mace_msg_auction_bid_bundle_rcv_part_get_bundleGenTime(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  8);
}

/**
 * @brief Get field seqNum from auction_bid_bundle_rcv_part message
 *
 * @return Last sequence number received (-1 to begin upload)
 */
static inline int8_t mace_msg_auction_bid_bundle_rcv_part_get_seqNum(const mace_message_t* msg)
{
    return _MACE_RETURN_int8_t(msg,  16);
}

/**
 * @brief Decode a auction_bid_bundle_rcv_part message into a struct
 *
 * @param msg The message to decode
 * @param auction_bid_bundle_rcv_part C-struct to decode the message contents into
 */
static inline void mace_msg_auction_bid_bundle_rcv_part_decode(const mace_message_t* msg, mace_auction_bid_bundle_rcv_part_t* auction_bid_bundle_rcv_part)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_bid_bundle_rcv_part->agentID = mace_msg_auction_bid_bundle_rcv_part_get_agentID(msg);
    auction_bid_bundle_rcv_part->bundleGenTime = mace_msg_auction_bid_bundle_rcv_part_get_bundleGenTime(msg);
    auction_bid_bundle_rcv_part->seqNum = mace_msg_auction_bid_bundle_rcv_part_get_seqNum(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN? msg->len : MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN;
        memset(auction_bid_bundle_rcv_part, 0, MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_LEN);
    memcpy(auction_bid_bundle_rcv_part, _MACE_PAYLOAD(msg), len);
#endif
}
