#pragma once
// MESSAGE EXTENDED_SYS_STATE PACKING

#define MACE_MSG_ID_EXTENDED_SYS_STATE 245

MACEPACKED(
typedef struct __mace_extended_sys_state_t {
 uint8_t vtol_state; /*< The VTOL state if applicable. Is set to UXV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.*/
 uint8_t landed_state; /*< The landed state. Is set to UXV_LANDED_STATE_UNDEFINED if landed state is unknown.*/
}) mace_extended_sys_state_t;

#define MACE_MSG_ID_EXTENDED_SYS_STATE_LEN 2
#define MACE_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN 2
#define MACE_MSG_ID_245_LEN 2
#define MACE_MSG_ID_245_MIN_LEN 2

#define MACE_MSG_ID_EXTENDED_SYS_STATE_CRC 130
#define MACE_MSG_ID_245_CRC 130



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_EXTENDED_SYS_STATE { \
    245, \
    "EXTENDED_SYS_STATE", \
    2, \
    {  { "vtol_state", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_extended_sys_state_t, vtol_state) }, \
         { "landed_state", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_extended_sys_state_t, landed_state) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_EXTENDED_SYS_STATE { \
    "EXTENDED_SYS_STATE", \
    2, \
    {  { "vtol_state", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_extended_sys_state_t, vtol_state) }, \
         { "landed_state", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_extended_sys_state_t, landed_state) }, \
         } \
}
#endif

/**
 * @brief Pack a extended_sys_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vtol_state The VTOL state if applicable. Is set to UXV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
 * @param landed_state The landed state. Is set to UXV_LANDED_STATE_UNDEFINED if landed state is unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_extended_sys_state_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t vtol_state, uint8_t landed_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_EXTENDED_SYS_STATE_LEN];
    _mace_put_uint8_t(buf, 0, vtol_state);
    _mace_put_uint8_t(buf, 1, landed_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_EXTENDED_SYS_STATE_LEN);
#else
    mace_extended_sys_state_t packet;
    packet.vtol_state = vtol_state;
    packet.landed_state = landed_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_EXTENDED_SYS_STATE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_EXTENDED_SYS_STATE;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_CRC);
}

/**
 * @brief Pack a extended_sys_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vtol_state The VTOL state if applicable. Is set to UXV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
 * @param landed_state The landed state. Is set to UXV_LANDED_STATE_UNDEFINED if landed state is unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_extended_sys_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t vtol_state,uint8_t landed_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_EXTENDED_SYS_STATE_LEN];
    _mace_put_uint8_t(buf, 0, vtol_state);
    _mace_put_uint8_t(buf, 1, landed_state);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_EXTENDED_SYS_STATE_LEN);
#else
    mace_extended_sys_state_t packet;
    packet.vtol_state = vtol_state;
    packet.landed_state = landed_state;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_EXTENDED_SYS_STATE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_EXTENDED_SYS_STATE;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_CRC);
}

/**
 * @brief Encode a extended_sys_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param extended_sys_state C-struct to read the message contents from
 */
static inline uint16_t mace_msg_extended_sys_state_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_extended_sys_state_t* extended_sys_state)
{
    return mace_msg_extended_sys_state_pack(system_id, component_id, msg, extended_sys_state->vtol_state, extended_sys_state->landed_state);
}

/**
 * @brief Encode a extended_sys_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param extended_sys_state C-struct to read the message contents from
 */
static inline uint16_t mace_msg_extended_sys_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_extended_sys_state_t* extended_sys_state)
{
    return mace_msg_extended_sys_state_pack_chan(system_id, component_id, chan, msg, extended_sys_state->vtol_state, extended_sys_state->landed_state);
}

/**
 * @brief Send a extended_sys_state message
 * @param chan MAVLink channel to send the message
 *
 * @param vtol_state The VTOL state if applicable. Is set to UXV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
 * @param landed_state The landed state. Is set to UXV_LANDED_STATE_UNDEFINED if landed state is unknown.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_extended_sys_state_send(mace_channel_t chan, uint8_t vtol_state, uint8_t landed_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_EXTENDED_SYS_STATE_LEN];
    _mace_put_uint8_t(buf, 0, vtol_state);
    _mace_put_uint8_t(buf, 1, landed_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_EXTENDED_SYS_STATE, buf, MACE_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_CRC);
#else
    mace_extended_sys_state_t packet;
    packet.vtol_state = vtol_state;
    packet.landed_state = landed_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_EXTENDED_SYS_STATE, (const char *)&packet, MACE_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_CRC);
#endif
}

/**
 * @brief Send a extended_sys_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_extended_sys_state_send_struct(mace_channel_t chan, const mace_extended_sys_state_t* extended_sys_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_extended_sys_state_send(chan, extended_sys_state->vtol_state, extended_sys_state->landed_state);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_EXTENDED_SYS_STATE, (const char *)extended_sys_state, MACE_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_CRC);
#endif
}

#if MACE_MSG_ID_EXTENDED_SYS_STATE_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_extended_sys_state_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t vtol_state, uint8_t landed_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, vtol_state);
    _mace_put_uint8_t(buf, 1, landed_state);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_EXTENDED_SYS_STATE, buf, MACE_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_CRC);
#else
    mace_extended_sys_state_t *packet = (mace_extended_sys_state_t *)msgbuf;
    packet->vtol_state = vtol_state;
    packet->landed_state = landed_state;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_EXTENDED_SYS_STATE, (const char *)packet, MACE_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_LEN, MACE_MSG_ID_EXTENDED_SYS_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE EXTENDED_SYS_STATE UNPACKING


/**
 * @brief Get field vtol_state from extended_sys_state message
 *
 * @return The VTOL state if applicable. Is set to UXV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
 */
static inline uint8_t mace_msg_extended_sys_state_get_vtol_state(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field landed_state from extended_sys_state message
 *
 * @return The landed state. Is set to UXV_LANDED_STATE_UNDEFINED if landed state is unknown.
 */
static inline uint8_t mace_msg_extended_sys_state_get_landed_state(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a extended_sys_state message into a struct
 *
 * @param msg The message to decode
 * @param extended_sys_state C-struct to decode the message contents into
 */
static inline void mace_msg_extended_sys_state_decode(const mace_message_t* msg, mace_extended_sys_state_t* extended_sys_state)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    extended_sys_state->vtol_state = mace_msg_extended_sys_state_get_vtol_state(msg);
    extended_sys_state->landed_state = mace_msg_extended_sys_state_get_landed_state(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_EXTENDED_SYS_STATE_LEN? msg->len : MACE_MSG_ID_EXTENDED_SYS_STATE_LEN;
        memset(extended_sys_state, 0, MACE_MSG_ID_EXTENDED_SYS_STATE_LEN);
    memcpy(extended_sys_state, _MACE_PAYLOAD(msg), len);
#endif
}
