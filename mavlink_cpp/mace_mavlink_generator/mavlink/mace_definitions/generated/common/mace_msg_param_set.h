#pragma once
// MESSAGE PARAM_SET PACKING

#define MACE_MSG_ID_PARAM_SET 12

MACEPACKED(
typedef struct __mace_param_set_t {
 float param_value; /*< Onboard parameter value*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 char param_id[16]; /*< Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
 uint8_t param_type; /*< Onboard parameter type: see the UXV_PARAM_TYPE enum for supported data types.*/
}) mace_param_set_t;

#define MACE_MSG_ID_PARAM_SET_LEN 23
#define MACE_MSG_ID_PARAM_SET_MIN_LEN 23
#define MACE_MSG_ID_12_LEN 23
#define MACE_MSG_ID_12_MIN_LEN 23

#define MACE_MSG_ID_PARAM_SET_CRC 168
#define MACE_MSG_ID_12_CRC 168

#define MACE_MSG_PARAM_SET_FIELD_PARAM_ID_LEN 16

#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_PARAM_SET { \
    12, \
    "PARAM_SET", \
    5, \
    {  { "param_value", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_param_set_t, param_value) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_param_set_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_param_set_t, target_component) }, \
         { "param_id", NULL, MACE_TYPE_CHAR, 16, 6, offsetof(mace_param_set_t, param_id) }, \
         { "param_type", NULL, MACE_TYPE_UINT8_T, 0, 22, offsetof(mace_param_set_t, param_type) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_PARAM_SET { \
    "PARAM_SET", \
    5, \
    {  { "param_value", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_param_set_t, param_value) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_param_set_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_param_set_t, target_component) }, \
         { "param_id", NULL, MACE_TYPE_CHAR, 16, 6, offsetof(mace_param_set_t, param_id) }, \
         { "param_type", NULL, MACE_TYPE_UINT8_T, 0, 22, offsetof(mace_param_set_t, param_type) }, \
         } \
}
#endif

/**
 * @brief Pack a param_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see the UXV_PARAM_TYPE enum for supported data types.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_param_set_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const char *param_id, float param_value, uint8_t param_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_PARAM_SET_LEN];
    _mace_put_float(buf, 0, param_value);
    _mace_put_uint8_t(buf, 4, target_system);
    _mace_put_uint8_t(buf, 5, target_component);
    _mace_put_uint8_t(buf, 22, param_type);
    _mace_put_char_array(buf, 6, param_id, 16);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_PARAM_SET_LEN);
#else
    mace_param_set_t packet;
    packet.param_value = param_value;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.param_type = param_type;
    mace_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_PARAM_SET_LEN);
#endif

    msg->msgid = MACE_MSG_ID_PARAM_SET;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_PARAM_SET_MIN_LEN, MACE_MSG_ID_PARAM_SET_LEN, MACE_MSG_ID_PARAM_SET_CRC);
}

/**
 * @brief Pack a param_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see the UXV_PARAM_TYPE enum for supported data types.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_param_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,const char *param_id,float param_value,uint8_t param_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_PARAM_SET_LEN];
    _mace_put_float(buf, 0, param_value);
    _mace_put_uint8_t(buf, 4, target_system);
    _mace_put_uint8_t(buf, 5, target_component);
    _mace_put_uint8_t(buf, 22, param_type);
    _mace_put_char_array(buf, 6, param_id, 16);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_PARAM_SET_LEN);
#else
    mace_param_set_t packet;
    packet.param_value = param_value;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.param_type = param_type;
    mace_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_PARAM_SET_LEN);
#endif

    msg->msgid = MACE_MSG_ID_PARAM_SET;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_PARAM_SET_MIN_LEN, MACE_MSG_ID_PARAM_SET_LEN, MACE_MSG_ID_PARAM_SET_CRC);
}

/**
 * @brief Encode a param_set struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_set C-struct to read the message contents from
 */
static inline uint16_t mace_msg_param_set_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_param_set_t* param_set)
{
    return mace_msg_param_set_pack(system_id, component_id, msg, param_set->target_system, param_set->target_component, param_set->param_id, param_set->param_value, param_set->param_type);
}

/**
 * @brief Encode a param_set struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_set C-struct to read the message contents from
 */
static inline uint16_t mace_msg_param_set_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_param_set_t* param_set)
{
    return mace_msg_param_set_pack_chan(system_id, component_id, chan, msg, param_set->target_system, param_set->target_component, param_set->param_id, param_set->param_value, param_set->param_type);
}

/**
 * @brief Send a param_set message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see the UXV_PARAM_TYPE enum for supported data types.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_param_set_send(mace_channel_t chan, uint8_t target_system, uint8_t target_component, const char *param_id, float param_value, uint8_t param_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_PARAM_SET_LEN];
    _mace_put_float(buf, 0, param_value);
    _mace_put_uint8_t(buf, 4, target_system);
    _mace_put_uint8_t(buf, 5, target_component);
    _mace_put_uint8_t(buf, 22, param_type);
    _mace_put_char_array(buf, 6, param_id, 16);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_SET, buf, MACE_MSG_ID_PARAM_SET_MIN_LEN, MACE_MSG_ID_PARAM_SET_LEN, MACE_MSG_ID_PARAM_SET_CRC);
#else
    mace_param_set_t packet;
    packet.param_value = param_value;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.param_type = param_type;
    mace_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_SET, (const char *)&packet, MACE_MSG_ID_PARAM_SET_MIN_LEN, MACE_MSG_ID_PARAM_SET_LEN, MACE_MSG_ID_PARAM_SET_CRC);
#endif
}

/**
 * @brief Send a param_set message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_param_set_send_struct(mace_channel_t chan, const mace_param_set_t* param_set)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_param_set_send(chan, param_set->target_system, param_set->target_component, param_set->param_id, param_set->param_value, param_set->param_type);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_SET, (const char *)param_set, MACE_MSG_ID_PARAM_SET_MIN_LEN, MACE_MSG_ID_PARAM_SET_LEN, MACE_MSG_ID_PARAM_SET_CRC);
#endif
}

#if MACE_MSG_ID_PARAM_SET_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_param_set_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t target_component, const char *param_id, float param_value, uint8_t param_type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_float(buf, 0, param_value);
    _mace_put_uint8_t(buf, 4, target_system);
    _mace_put_uint8_t(buf, 5, target_component);
    _mace_put_uint8_t(buf, 22, param_type);
    _mace_put_char_array(buf, 6, param_id, 16);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_SET, buf, MACE_MSG_ID_PARAM_SET_MIN_LEN, MACE_MSG_ID_PARAM_SET_LEN, MACE_MSG_ID_PARAM_SET_CRC);
#else
    mace_param_set_t *packet = (mace_param_set_t *)msgbuf;
    packet->param_value = param_value;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->param_type = param_type;
    mace_array_memcpy(packet->param_id, param_id, sizeof(char)*16);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_PARAM_SET, (const char *)packet, MACE_MSG_ID_PARAM_SET_MIN_LEN, MACE_MSG_ID_PARAM_SET_LEN, MACE_MSG_ID_PARAM_SET_CRC);
#endif
}
#endif

#endif

// MESSAGE PARAM_SET UNPACKING


/**
 * @brief Get field target_system from param_set message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_param_set_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from param_set message
 *
 * @return Component ID
 */
static inline uint8_t mace_msg_param_set_get_target_component(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field param_id from param_set message
 *
 * @return Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 */
static inline uint16_t mace_msg_param_set_get_param_id(const mace_message_t* msg, char *param_id)
{
    return _MACE_RETURN_char_array(msg, param_id, 16,  6);
}

/**
 * @brief Get field param_value from param_set message
 *
 * @return Onboard parameter value
 */
static inline float mace_msg_param_set_get_param_value(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  0);
}

/**
 * @brief Get field param_type from param_set message
 *
 * @return Onboard parameter type: see the UXV_PARAM_TYPE enum for supported data types.
 */
static inline uint8_t mace_msg_param_set_get_param_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Decode a param_set message into a struct
 *
 * @param msg The message to decode
 * @param param_set C-struct to decode the message contents into
 */
static inline void mace_msg_param_set_decode(const mace_message_t* msg, mace_param_set_t* param_set)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    param_set->param_value = mace_msg_param_set_get_param_value(msg);
    param_set->target_system = mace_msg_param_set_get_target_system(msg);
    param_set->target_component = mace_msg_param_set_get_target_component(msg);
    mace_msg_param_set_get_param_id(msg, param_set->param_id);
    param_set->param_type = mace_msg_param_set_get_param_type(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_PARAM_SET_LEN? msg->len : MACE_MSG_ID_PARAM_SET_LEN;
        memset(param_set, 0, MACE_MSG_ID_PARAM_SET_LEN);
    memcpy(param_set, _MACE_PAYLOAD(msg), len);
#endif
}
