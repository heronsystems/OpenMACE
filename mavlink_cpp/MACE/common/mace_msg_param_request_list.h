#pragma once
// MESSAGE PARAM_REQUEST_LIST PACKING

#define MACE_MSG_ID_PARAM_REQUEST_LIST 10

MACEPACKED(
typedef struct __mace_param_request_list_t {
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
}) mace_param_request_list_t;

#define MACE_MSG_ID_PARAM_REQUEST_LIST_LEN 2
#define MACE_MSG_ID_PARAM_REQUEST_LIST_MIN_LEN 2
#define MACE_MSG_ID_10_LEN 2
#define MACE_MSG_ID_10_MIN_LEN 2

#define MACE_MSG_ID_PARAM_REQUEST_LIST_CRC 159
#define MACE_MSG_ID_10_CRC 159



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_PARAM_REQUEST_LIST { \
    10, \
    "PARAM_REQUEST_LIST", \
    2, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_param_request_list_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_param_request_list_t, target_component) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_PARAM_REQUEST_LIST { \
    "PARAM_REQUEST_LIST", \
    2, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_param_request_list_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_param_request_list_t, target_component) }, \
         } \
}
#endif

/**
 * @brief Pack a param_request_list message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_param_request_list_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t target_component)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_PARAM_REQUEST_LIST_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, target_component);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_PARAM_REQUEST_LIST_LEN);
#else
    mace_param_request_list_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_PARAM_REQUEST_LIST_LEN);
#endif

    msg->msgid = MACE_MSG_ID_PARAM_REQUEST_LIST;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_PARAM_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_CRC);
}

/**
 * @brief Pack a param_request_list message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_param_request_list_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t target_component)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_PARAM_REQUEST_LIST_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, target_component);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_PARAM_REQUEST_LIST_LEN);
#else
    mace_param_request_list_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_PARAM_REQUEST_LIST_LEN);
#endif

    msg->msgid = MACE_MSG_ID_PARAM_REQUEST_LIST;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_PARAM_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_CRC);
}

/**
 * @brief Encode a param_request_list struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_request_list C-struct to read the message contents from
 */
static inline uint16_t mace_msg_param_request_list_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_param_request_list_t* param_request_list)
{
    return mace_msg_param_request_list_pack(system_id, component_id, msg, param_request_list->target_system, param_request_list->target_component);
}

/**
 * @brief Encode a param_request_list struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_request_list C-struct to read the message contents from
 */
static inline uint16_t mace_msg_param_request_list_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_param_request_list_t* param_request_list)
{
    return mace_msg_param_request_list_pack_chan(system_id, component_id, chan, msg, param_request_list->target_system, param_request_list->target_component);
}

/**
 * @brief Send a param_request_list message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_param_request_list_send(mace_channel_t chan, uint8_t target_system, uint8_t target_component)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_PARAM_REQUEST_LIST_LEN];
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, target_component);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_REQUEST_LIST, buf, MACE_MSG_ID_PARAM_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_CRC);
#else
    mace_param_request_list_t packet;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_REQUEST_LIST, (const char *)&packet, MACE_MSG_ID_PARAM_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_CRC);
#endif
}

/**
 * @brief Send a param_request_list message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_param_request_list_send_struct(mace_channel_t chan, const mace_param_request_list_t* param_request_list)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_param_request_list_send(chan, param_request_list->target_system, param_request_list->target_component);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_REQUEST_LIST, (const char *)param_request_list, MACE_MSG_ID_PARAM_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_CRC);
#endif
}

#if MACE_MSG_ID_PARAM_REQUEST_LIST_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_param_request_list_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t target_component)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, target_system);
    _mace_put_uint8_t(buf, 1, target_component);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_REQUEST_LIST, buf, MACE_MSG_ID_PARAM_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_CRC);
#else
    mace_param_request_list_t *packet = (mace_param_request_list_t *)msgbuf;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_REQUEST_LIST, (const char *)packet, MACE_MSG_ID_PARAM_REQUEST_LIST_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_LEN, MACE_MSG_ID_PARAM_REQUEST_LIST_CRC);
#endif
}
#endif

#endif

// MESSAGE PARAM_REQUEST_LIST UNPACKING


/**
 * @brief Get field target_system from param_request_list message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_param_request_list_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from param_request_list message
 *
 * @return Component ID
 */
static inline uint8_t mace_msg_param_request_list_get_target_component(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a param_request_list message into a struct
 *
 * @param msg The message to decode
 * @param param_request_list C-struct to decode the message contents into
 */
static inline void mace_msg_param_request_list_decode(const mace_message_t* msg, mace_param_request_list_t* param_request_list)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    param_request_list->target_system = mace_msg_param_request_list_get_target_system(msg);
    param_request_list->target_component = mace_msg_param_request_list_get_target_component(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_PARAM_REQUEST_LIST_LEN? msg->len : MACE_MSG_ID_PARAM_REQUEST_LIST_LEN;
        memset(param_request_list, 0, MACE_MSG_ID_PARAM_REQUEST_LIST_LEN);
    memcpy(param_request_list, _MACE_PAYLOAD(msg), len);
#endif
}
