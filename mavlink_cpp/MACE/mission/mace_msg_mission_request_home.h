#pragma once
// MESSAGE MISSION_REQUEST_HOME PACKING

#define MACE_MSG_ID_MISSION_REQUEST_HOME 116

MACEPACKED(
typedef struct __mace_mission_request_home_t {
 uint8_t target_system; /*< The system which the home position is being requested from.*/
}) mace_mission_request_home_t;

#define MACE_MSG_ID_MISSION_REQUEST_HOME_LEN 1
#define MACE_MSG_ID_MISSION_REQUEST_HOME_MIN_LEN 1
#define MACE_MSG_ID_116_LEN 1
#define MACE_MSG_ID_116_MIN_LEN 1

#define MACE_MSG_ID_MISSION_REQUEST_HOME_CRC 36
#define MACE_MSG_ID_116_CRC 36



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_MISSION_REQUEST_HOME { \
    116, \
    "MISSION_REQUEST_HOME", \
    1, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_mission_request_home_t, target_system) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_MISSION_REQUEST_HOME { \
    "MISSION_REQUEST_HOME", \
    1, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_mission_request_home_t, target_system) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_request_home message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system The system which the home position is being requested from.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_request_home_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_REQUEST_HOME_LEN];
    _mace_put_uint8_t(buf, 0, target_system);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_REQUEST_HOME_LEN);
#else
    mace_mission_request_home_t packet;
    packet.target_system = target_system;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_REQUEST_HOME_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_REQUEST_HOME;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_MISSION_REQUEST_HOME_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_CRC);
}

/**
 * @brief Pack a mission_request_home message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system The system which the home position is being requested from.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_mission_request_home_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_REQUEST_HOME_LEN];
    _mace_put_uint8_t(buf, 0, target_system);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_MISSION_REQUEST_HOME_LEN);
#else
    mace_mission_request_home_t packet;
    packet.target_system = target_system;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_MISSION_REQUEST_HOME_LEN);
#endif

    msg->msgid = MACE_MSG_ID_MISSION_REQUEST_HOME;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_MISSION_REQUEST_HOME_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_CRC);
}

/**
 * @brief Encode a mission_request_home struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_home C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_request_home_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_mission_request_home_t* mission_request_home)
{
    return mace_msg_mission_request_home_pack(system_id, component_id, msg, mission_request_home->target_system);
}

/**
 * @brief Encode a mission_request_home struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_home C-struct to read the message contents from
 */
static inline uint16_t mace_msg_mission_request_home_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_mission_request_home_t* mission_request_home)
{
    return mace_msg_mission_request_home_pack_chan(system_id, component_id, chan, msg, mission_request_home->target_system);
}

/**
 * @brief Send a mission_request_home message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system The system which the home position is being requested from.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_mission_request_home_send(mace_channel_t chan, uint8_t target_system)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_MISSION_REQUEST_HOME_LEN];
    _mace_put_uint8_t(buf, 0, target_system);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_HOME, buf, MACE_MSG_ID_MISSION_REQUEST_HOME_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_CRC);
#else
    mace_mission_request_home_t packet;
    packet.target_system = target_system;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_HOME, (const char *)&packet, MACE_MSG_ID_MISSION_REQUEST_HOME_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_CRC);
#endif
}

/**
 * @brief Send a mission_request_home message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_mission_request_home_send_struct(mace_channel_t chan, const mace_mission_request_home_t* mission_request_home)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_mission_request_home_send(chan, mission_request_home->target_system);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_HOME, (const char *)mission_request_home, MACE_MSG_ID_MISSION_REQUEST_HOME_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_CRC);
#endif
}

#if MACE_MSG_ID_MISSION_REQUEST_HOME_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_mission_request_home_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, target_system);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_HOME, buf, MACE_MSG_ID_MISSION_REQUEST_HOME_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_CRC);
#else
    mace_mission_request_home_t *packet = (mace_mission_request_home_t *)msgbuf;
    packet->target_system = target_system;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_MISSION_REQUEST_HOME, (const char *)packet, MACE_MSG_ID_MISSION_REQUEST_HOME_MIN_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_LEN, MACE_MSG_ID_MISSION_REQUEST_HOME_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_REQUEST_HOME UNPACKING


/**
 * @brief Get field target_system from mission_request_home message
 *
 * @return The system which the home position is being requested from.
 */
static inline uint8_t mace_msg_mission_request_home_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a mission_request_home message into a struct
 *
 * @param msg The message to decode
 * @param mission_request_home C-struct to decode the message contents into
 */
static inline void mace_msg_mission_request_home_decode(const mace_message_t* msg, mace_mission_request_home_t* mission_request_home)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mission_request_home->target_system = mace_msg_mission_request_home_get_target_system(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_MISSION_REQUEST_HOME_LEN? msg->len : MACE_MSG_ID_MISSION_REQUEST_HOME_LEN;
        memset(mission_request_home, 0, MACE_MSG_ID_MISSION_REQUEST_HOME_LEN);
    memcpy(mission_request_home, _MACE_PAYLOAD(msg), len);
#endif
}
