#pragma once
// MESSAGE PARAM_REQUEST_READ PACKING

#define MACE_MSG_ID_PARAM_REQUEST_READ 9

MACEPACKED(
typedef struct __mace_param_request_read_t {
 int16_t param_index; /*< Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 char param_id[16]; /*< Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
}) mace_param_request_read_t;

#define MACE_MSG_ID_PARAM_REQUEST_READ_LEN 20
#define MACE_MSG_ID_PARAM_REQUEST_READ_MIN_LEN 20
#define MACE_MSG_ID_9_LEN 20
#define MACE_MSG_ID_9_MIN_LEN 20

#define MACE_MSG_ID_PARAM_REQUEST_READ_CRC 214
#define MACE_MSG_ID_9_CRC 214

#define MACE_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN 16

#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_PARAM_REQUEST_READ { \
    9, \
    "PARAM_REQUEST_READ", \
    4, \
    {  { "param_index", NULL, MACE_TYPE_INT16_T, 0, 0, offsetof(mace_param_request_read_t, param_index) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_param_request_read_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_param_request_read_t, target_component) }, \
         { "param_id", NULL, MACE_TYPE_CHAR, 16, 4, offsetof(mace_param_request_read_t, param_id) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_PARAM_REQUEST_READ { \
    "PARAM_REQUEST_READ", \
    4, \
    {  { "param_index", NULL, MACE_TYPE_INT16_T, 0, 0, offsetof(mace_param_request_read_t, param_index) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_param_request_read_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_param_request_read_t, target_component) }, \
         { "param_id", NULL, MACE_TYPE_CHAR, 16, 4, offsetof(mace_param_request_read_t, param_id) }, \
         } \
}
#endif

/**
 * @brief Pack a param_request_read message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_index Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_param_request_read_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const char *param_id, int16_t param_index)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_PARAM_REQUEST_READ_LEN];
    _mace_put_int16_t(buf, 0, param_index);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, target_component);
    _mace_put_char_array(buf, 4, param_id, 16);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_PARAM_REQUEST_READ_LEN);
#else
    mace_param_request_read_t packet;
    packet.param_index = param_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mace_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_PARAM_REQUEST_READ_LEN);
#endif

    msg->msgid = MACE_MSG_ID_PARAM_REQUEST_READ;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_PARAM_REQUEST_READ_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_CRC);
}

/**
 * @brief Pack a param_request_read message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_index Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_param_request_read_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,const char *param_id,int16_t param_index)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_PARAM_REQUEST_READ_LEN];
    _mace_put_int16_t(buf, 0, param_index);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, target_component);
    _mace_put_char_array(buf, 4, param_id, 16);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_PARAM_REQUEST_READ_LEN);
#else
    mace_param_request_read_t packet;
    packet.param_index = param_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mace_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_PARAM_REQUEST_READ_LEN);
#endif

    msg->msgid = MACE_MSG_ID_PARAM_REQUEST_READ;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_PARAM_REQUEST_READ_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_CRC);
}

/**
 * @brief Encode a param_request_read struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_request_read C-struct to read the message contents from
 */
static inline uint16_t mace_msg_param_request_read_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_param_request_read_t* param_request_read)
{
    return mace_msg_param_request_read_pack(system_id, component_id, msg, param_request_read->target_system, param_request_read->target_component, param_request_read->param_id, param_request_read->param_index);
}

/**
 * @brief Encode a param_request_read struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_request_read C-struct to read the message contents from
 */
static inline uint16_t mace_msg_param_request_read_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_param_request_read_t* param_request_read)
{
    return mace_msg_param_request_read_pack_chan(system_id, component_id, chan, msg, param_request_read->target_system, param_request_read->target_component, param_request_read->param_id, param_request_read->param_index);
}

/**
 * @brief Send a param_request_read message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_index Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_param_request_read_send(mace_channel_t chan, uint8_t target_system, uint8_t target_component, const char *param_id, int16_t param_index)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_PARAM_REQUEST_READ_LEN];
    _mace_put_int16_t(buf, 0, param_index);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, target_component);
    _mace_put_char_array(buf, 4, param_id, 16);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_REQUEST_READ, buf, MACE_MSG_ID_PARAM_REQUEST_READ_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_CRC);
#else
    mace_param_request_read_t packet;
    packet.param_index = param_index;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mace_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_REQUEST_READ, (const char *)&packet, MACE_MSG_ID_PARAM_REQUEST_READ_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_CRC);
#endif
}

/**
 * @brief Send a param_request_read message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_param_request_read_send_struct(mace_channel_t chan, const mace_param_request_read_t* param_request_read)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_param_request_read_send(chan, param_request_read->target_system, param_request_read->target_component, param_request_read->param_id, param_request_read->param_index);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_REQUEST_READ, (const char *)param_request_read, MACE_MSG_ID_PARAM_REQUEST_READ_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_CRC);
#endif
}

#if MACE_MSG_ID_PARAM_REQUEST_READ_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_param_request_read_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t target_component, const char *param_id, int16_t param_index)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_int16_t(buf, 0, param_index);
    _mace_put_uint8_t(buf, 2, target_system);
    _mace_put_uint8_t(buf, 3, target_component);
    _mace_put_char_array(buf, 4, param_id, 16);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_REQUEST_READ, buf, MACE_MSG_ID_PARAM_REQUEST_READ_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_CRC);
#else
    mace_param_request_read_t *packet = (mace_param_request_read_t *)msgbuf;
    packet->param_index = param_index;
    packet->target_system = target_system;
    packet->target_component = target_component;
    mace_array_memcpy(packet->param_id, param_id, sizeof(char)*16);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_REQUEST_READ, (const char *)packet, MACE_MSG_ID_PARAM_REQUEST_READ_MIN_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_LEN, MACE_MSG_ID_PARAM_REQUEST_READ_CRC);
#endif
}
#endif

#endif

// MESSAGE PARAM_REQUEST_READ UNPACKING


/**
 * @brief Get field target_system from param_request_read message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_param_request_read_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field target_component from param_request_read message
 *
 * @return Component ID
 */
static inline uint8_t mace_msg_param_request_read_get_target_component(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field param_id from param_request_read message
 *
 * @return Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 */
static inline uint16_t mace_msg_param_request_read_get_param_id(const mace_message_t* msg, char *param_id)
{
    return _MACE_RETURN_char_array(msg, param_id, 16,  4);
}

/**
 * @brief Get field param_index from param_request_read message
 *
 * @return Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
 */
static inline int16_t mace_msg_param_request_read_get_param_index(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  0);
}

/**
 * @brief Decode a param_request_read message into a struct
 *
 * @param msg The message to decode
 * @param param_request_read C-struct to decode the message contents into
 */
static inline void mace_msg_param_request_read_decode(const mace_message_t* msg, mace_param_request_read_t* param_request_read)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    param_request_read->param_index = mace_msg_param_request_read_get_param_index(msg);
    param_request_read->target_system = mace_msg_param_request_read_get_target_system(msg);
    param_request_read->target_component = mace_msg_param_request_read_get_target_component(msg);
    mace_msg_param_request_read_get_param_id(msg, param_request_read->param_id);
#else
        uint8_t len = msg->len < MACE_MSG_ID_PARAM_REQUEST_READ_LEN? msg->len : MACE_MSG_ID_PARAM_REQUEST_READ_LEN;
        memset(param_request_read, 0, MACE_MSG_ID_PARAM_REQUEST_READ_LEN);
    memcpy(param_request_read, _MACE_PAYLOAD(msg), len);
#endif
}
