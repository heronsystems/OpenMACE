#pragma once
// MESSAGE BOUNDARY_REQUEST_ITEM PACKING

#define MACE_MSG_ID_BOUNDARY_REQUEST_ITEM 134

MACEPACKED(
typedef struct __mace_boundary_request_item_t {
 uint16_t seq; /*< Item index within the bonudary sequence.*/
 uint8_t boundary_host_sysid; /*< System ID*/
 uint8_t boundary_host_compid; /*< Creator ID*/
 uint8_t boundary_identifier; /*< Number to identifiy boundary on host.*/
}) mace_boundary_request_item_t;

#define MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN 5
#define MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_MIN_LEN 5
#define MACE_MSG_ID_134_LEN 5
#define MACE_MSG_ID_134_MIN_LEN 5

#define MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_CRC 138
#define MACE_MSG_ID_134_CRC 138



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_BOUNDARY_REQUEST_ITEM { \
    134, \
    "BOUNDARY_REQUEST_ITEM", \
    4, \
    {  { "seq", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_boundary_request_item_t, seq) }, \
         { "boundary_host_sysid", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_boundary_request_item_t, boundary_host_sysid) }, \
         { "boundary_host_compid", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_boundary_request_item_t, boundary_host_compid) }, \
         { "boundary_identifier", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_boundary_request_item_t, boundary_identifier) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_BOUNDARY_REQUEST_ITEM { \
    "BOUNDARY_REQUEST_ITEM", \
    4, \
    {  { "seq", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_boundary_request_item_t, seq) }, \
         { "boundary_host_sysid", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_boundary_request_item_t, boundary_host_sysid) }, \
         { "boundary_host_compid", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_boundary_request_item_t, boundary_host_compid) }, \
         { "boundary_identifier", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_boundary_request_item_t, boundary_identifier) }, \
         } \
}
#endif

/**
 * @brief Pack a boundary_request_item message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param boundary_host_sysid System ID
 * @param boundary_host_compid Creator ID
 * @param boundary_identifier Number to identifiy boundary on host.
 * @param seq Item index within the bonudary sequence.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_boundary_request_item_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t boundary_host_sysid, uint8_t boundary_host_compid, uint8_t boundary_identifier, uint16_t seq)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN];
    _mace_put_uint16_t(buf, 0, seq);
    _mace_put_uint8_t(buf, 2, boundary_host_sysid);
    _mace_put_uint8_t(buf, 3, boundary_host_compid);
    _mace_put_uint8_t(buf, 4, boundary_identifier);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN);
#else
    mace_boundary_request_item_t packet;
    packet.seq = seq;
    packet.boundary_host_sysid = boundary_host_sysid;
    packet.boundary_host_compid = boundary_host_compid;
    packet.boundary_identifier = boundary_identifier;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN);
#endif

    msg->msgid = MACE_MSG_ID_BOUNDARY_REQUEST_ITEM;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_CRC);
}

/**
 * @brief Pack a boundary_request_item message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boundary_host_sysid System ID
 * @param boundary_host_compid Creator ID
 * @param boundary_identifier Number to identifiy boundary on host.
 * @param seq Item index within the bonudary sequence.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_boundary_request_item_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t boundary_host_sysid,uint8_t boundary_host_compid,uint8_t boundary_identifier,uint16_t seq)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN];
    _mace_put_uint16_t(buf, 0, seq);
    _mace_put_uint8_t(buf, 2, boundary_host_sysid);
    _mace_put_uint8_t(buf, 3, boundary_host_compid);
    _mace_put_uint8_t(buf, 4, boundary_identifier);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN);
#else
    mace_boundary_request_item_t packet;
    packet.seq = seq;
    packet.boundary_host_sysid = boundary_host_sysid;
    packet.boundary_host_compid = boundary_host_compid;
    packet.boundary_identifier = boundary_identifier;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN);
#endif

    msg->msgid = MACE_MSG_ID_BOUNDARY_REQUEST_ITEM;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_CRC);
}

/**
 * @brief Encode a boundary_request_item struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param boundary_request_item C-struct to read the message contents from
 */
static inline uint16_t mace_msg_boundary_request_item_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_boundary_request_item_t* boundary_request_item)
{
    return mace_msg_boundary_request_item_pack(system_id, component_id, msg, boundary_request_item->boundary_host_sysid, boundary_request_item->boundary_host_compid, boundary_request_item->boundary_identifier, boundary_request_item->seq);
}

/**
 * @brief Encode a boundary_request_item struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param boundary_request_item C-struct to read the message contents from
 */
static inline uint16_t mace_msg_boundary_request_item_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_boundary_request_item_t* boundary_request_item)
{
    return mace_msg_boundary_request_item_pack_chan(system_id, component_id, chan, msg, boundary_request_item->boundary_host_sysid, boundary_request_item->boundary_host_compid, boundary_request_item->boundary_identifier, boundary_request_item->seq);
}

/**
 * @brief Send a boundary_request_item message
 * @param chan MAVLink channel to send the message
 *
 * @param boundary_host_sysid System ID
 * @param boundary_host_compid Creator ID
 * @param boundary_identifier Number to identifiy boundary on host.
 * @param seq Item index within the bonudary sequence.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_boundary_request_item_send(mace_channel_t chan, uint8_t boundary_host_sysid, uint8_t boundary_host_compid, uint8_t boundary_identifier, uint16_t seq)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN];
    _mace_put_uint16_t(buf, 0, seq);
    _mace_put_uint8_t(buf, 2, boundary_host_sysid);
    _mace_put_uint8_t(buf, 3, boundary_host_compid);
    _mace_put_uint8_t(buf, 4, boundary_identifier);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM, buf, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_CRC);
#else
    mace_boundary_request_item_t packet;
    packet.seq = seq;
    packet.boundary_host_sysid = boundary_host_sysid;
    packet.boundary_host_compid = boundary_host_compid;
    packet.boundary_identifier = boundary_identifier;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM, (const char *)&packet, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_CRC);
#endif
}

/**
 * @brief Send a boundary_request_item message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_boundary_request_item_send_struct(mace_channel_t chan, const mace_boundary_request_item_t* boundary_request_item)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_boundary_request_item_send(chan, boundary_request_item->boundary_host_sysid, boundary_request_item->boundary_host_compid, boundary_request_item->boundary_identifier, boundary_request_item->seq);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM, (const char *)boundary_request_item, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_CRC);
#endif
}

#if MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_boundary_request_item_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t boundary_host_sysid, uint8_t boundary_host_compid, uint8_t boundary_identifier, uint16_t seq)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint16_t(buf, 0, seq);
    _mace_put_uint8_t(buf, 2, boundary_host_sysid);
    _mace_put_uint8_t(buf, 3, boundary_host_compid);
    _mace_put_uint8_t(buf, 4, boundary_identifier);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM, buf, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_CRC);
#else
    mace_boundary_request_item_t *packet = (mace_boundary_request_item_t *)msgbuf;
    packet->seq = seq;
    packet->boundary_host_sysid = boundary_host_sysid;
    packet->boundary_host_compid = boundary_host_compid;
    packet->boundary_identifier = boundary_identifier;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM, (const char *)packet, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_MIN_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_CRC);
#endif
}
#endif

#endif

// MESSAGE BOUNDARY_REQUEST_ITEM UNPACKING


/**
 * @brief Get field boundary_host_sysid from boundary_request_item message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_boundary_request_item_get_boundary_host_sysid(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field boundary_host_compid from boundary_request_item message
 *
 * @return Creator ID
 */
static inline uint8_t mace_msg_boundary_request_item_get_boundary_host_compid(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field boundary_identifier from boundary_request_item message
 *
 * @return Number to identifiy boundary on host.
 */
static inline uint8_t mace_msg_boundary_request_item_get_boundary_identifier(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field seq from boundary_request_item message
 *
 * @return Item index within the bonudary sequence.
 */
static inline uint16_t mace_msg_boundary_request_item_get_seq(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a boundary_request_item message into a struct
 *
 * @param msg The message to decode
 * @param boundary_request_item C-struct to decode the message contents into
 */
static inline void mace_msg_boundary_request_item_decode(const mace_message_t* msg, mace_boundary_request_item_t* boundary_request_item)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    boundary_request_item->seq = mace_msg_boundary_request_item_get_seq(msg);
    boundary_request_item->boundary_host_sysid = mace_msg_boundary_request_item_get_boundary_host_sysid(msg);
    boundary_request_item->boundary_host_compid = mace_msg_boundary_request_item_get_boundary_host_compid(msg);
    boundary_request_item->boundary_identifier = mace_msg_boundary_request_item_get_boundary_identifier(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN? msg->len : MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN;
        memset(boundary_request_item, 0, MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_LEN);
    memcpy(boundary_request_item, _MACE_PAYLOAD(msg), len);
#endif
}
