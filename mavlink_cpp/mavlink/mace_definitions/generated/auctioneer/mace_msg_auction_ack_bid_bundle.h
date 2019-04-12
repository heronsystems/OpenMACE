#pragma once
// MESSAGE AUCTION_ACK_BID_BUNDLE PACKING

#define MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE 10004

MACEPACKED(
typedef struct __mace_auction_ack_bid_bundle_t {
 uint64_t agentID; /*< ID of agent generating the bundle*/
 double bundleGenTime; /*< Bundle generation time*/
}) mace_auction_ack_bid_bundle_t;

#define MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN 16
#define MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_MIN_LEN 16
#define MACE_MSG_ID_10004_LEN 16
#define MACE_MSG_ID_10004_MIN_LEN 16

#define MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_CRC 150
#define MACE_MSG_ID_10004_CRC 150



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_ACK_BID_BUNDLE { \
    10004, \
    "AUCTION_ACK_BID_BUNDLE", \
    2, \
    {  { "agentID", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_ack_bid_bundle_t, agentID) }, \
         { "bundleGenTime", NULL, MACE_TYPE_DOUBLE, 0, 8, offsetof(mace_auction_ack_bid_bundle_t, bundleGenTime) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_ACK_BID_BUNDLE { \
    "AUCTION_ACK_BID_BUNDLE", \
    2, \
    {  { "agentID", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_ack_bid_bundle_t, agentID) }, \
         { "bundleGenTime", NULL, MACE_TYPE_DOUBLE, 0, 8, offsetof(mace_auction_ack_bid_bundle_t, bundleGenTime) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_ack_bid_bundle message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param agentID ID of agent generating the bundle
 * @param bundleGenTime Bundle generation time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_ack_bid_bundle_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t agentID, double bundleGenTime)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN];
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, bundleGenTime);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN);
#else
    mace_auction_ack_bid_bundle_t packet;
    packet.agentID = agentID;
    packet.bundleGenTime = bundleGenTime;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_MIN_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_CRC);
}

/**
 * @brief Pack a auction_ack_bid_bundle message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param agentID ID of agent generating the bundle
 * @param bundleGenTime Bundle generation time
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_ack_bid_bundle_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t agentID,double bundleGenTime)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN];
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, bundleGenTime);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN);
#else
    mace_auction_ack_bid_bundle_t packet;
    packet.agentID = agentID;
    packet.bundleGenTime = bundleGenTime;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_MIN_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_CRC);
}

/**
 * @brief Encode a auction_ack_bid_bundle struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_ack_bid_bundle C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_ack_bid_bundle_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_ack_bid_bundle_t* auction_ack_bid_bundle)
{
    return mace_msg_auction_ack_bid_bundle_pack(system_id, component_id, msg, auction_ack_bid_bundle->agentID, auction_ack_bid_bundle->bundleGenTime);
}

/**
 * @brief Encode a auction_ack_bid_bundle struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_ack_bid_bundle C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_ack_bid_bundle_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_ack_bid_bundle_t* auction_ack_bid_bundle)
{
    return mace_msg_auction_ack_bid_bundle_pack_chan(system_id, component_id, chan, msg, auction_ack_bid_bundle->agentID, auction_ack_bid_bundle->bundleGenTime);
}

/**
 * @brief Send a auction_ack_bid_bundle message
 * @param chan MAVLink channel to send the message
 *
 * @param agentID ID of agent generating the bundle
 * @param bundleGenTime Bundle generation time
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_ack_bid_bundle_send(mace_channel_t chan, uint64_t agentID, double bundleGenTime)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN];
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, bundleGenTime);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE, buf, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_MIN_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_CRC);
#else
    mace_auction_ack_bid_bundle_t packet;
    packet.agentID = agentID;
    packet.bundleGenTime = bundleGenTime;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE, (const char *)&packet, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_MIN_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_CRC);
#endif
}

/**
 * @brief Send a auction_ack_bid_bundle message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_ack_bid_bundle_send_struct(mace_channel_t chan, const mace_auction_ack_bid_bundle_t* auction_ack_bid_bundle)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_ack_bid_bundle_send(chan, auction_ack_bid_bundle->agentID, auction_ack_bid_bundle->bundleGenTime);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE, (const char *)auction_ack_bid_bundle, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_MIN_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_ack_bid_bundle_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t agentID, double bundleGenTime)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, agentID);
    _mace_put_double(buf, 8, bundleGenTime);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE, buf, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_MIN_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_CRC);
#else
    mace_auction_ack_bid_bundle_t *packet = (mace_auction_ack_bid_bundle_t *)msgbuf;
    packet->agentID = agentID;
    packet->bundleGenTime = bundleGenTime;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE, (const char *)packet, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_MIN_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_ACK_BID_BUNDLE UNPACKING


/**
 * @brief Get field agentID from auction_ack_bid_bundle message
 *
 * @return ID of agent generating the bundle
 */
static inline uint64_t mace_msg_auction_ack_bid_bundle_get_agentID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field bundleGenTime from auction_ack_bid_bundle message
 *
 * @return Bundle generation time
 */
static inline double mace_msg_auction_ack_bid_bundle_get_bundleGenTime(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  8);
}

/**
 * @brief Decode a auction_ack_bid_bundle message into a struct
 *
 * @param msg The message to decode
 * @param auction_ack_bid_bundle C-struct to decode the message contents into
 */
static inline void mace_msg_auction_ack_bid_bundle_decode(const mace_message_t* msg, mace_auction_ack_bid_bundle_t* auction_ack_bid_bundle)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_ack_bid_bundle->agentID = mace_msg_auction_ack_bid_bundle_get_agentID(msg);
    auction_ack_bid_bundle->bundleGenTime = mace_msg_auction_ack_bid_bundle_get_bundleGenTime(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN? msg->len : MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN;
        memset(auction_ack_bid_bundle, 0, MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_LEN);
    memcpy(auction_ack_bid_bundle, _MACE_PAYLOAD(msg), len);
#endif
}
