#pragma once
// MESSAGE HEARTBEAT PACKING

#define MACE_MSG_ID_HEARTBEAT 0

MACEPACKED(
typedef struct __mace_heartbeat_t {
 uint8_t protocol; /*< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)*/
 uint8_t type; /*< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)*/
 uint8_t autopilot; /*< Autopilot type / class. defined in MAV_AUTOPILOT ENUM*/
 uint8_t mission_state; /*< Defines the current state of the vehicle mission. Useful for determing the next state of the vehicle per mission state.*/
 uint8_t mace_companion; /*< Boolean describing whether(T=1) or not(F=0) the vehicle is MACE companion equipped.*/
 uint8_t mavlink_version; /*< MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/
 uint8_t mavlinkID; /*< */
}) mace_heartbeat_t;

#define MACE_MSG_ID_HEARTBEAT_LEN 7
#define MACE_MSG_ID_HEARTBEAT_MIN_LEN 7
#define MACE_MSG_ID_0_LEN 7
#define MACE_MSG_ID_0_MIN_LEN 7

#define MACE_MSG_ID_HEARTBEAT_CRC 39
#define MACE_MSG_ID_0_CRC 39



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_HEARTBEAT { \
    0, \
    "HEARTBEAT", \
    7, \
    {  { "protocol", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_heartbeat_t, protocol) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_heartbeat_t, type) }, \
         { "autopilot", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_heartbeat_t, autopilot) }, \
         { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_heartbeat_t, mission_state) }, \
         { "mace_companion", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_heartbeat_t, mace_companion) }, \
         { "mavlink_version", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_heartbeat_t, mavlink_version) }, \
         { "mavlinkID", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_heartbeat_t, mavlinkID) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_HEARTBEAT { \
    "HEARTBEAT", \
    7, \
    {  { "protocol", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_heartbeat_t, protocol) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 1, offsetof(mace_heartbeat_t, type) }, \
         { "autopilot", NULL, MACE_TYPE_UINT8_T, 0, 2, offsetof(mace_heartbeat_t, autopilot) }, \
         { "mission_state", NULL, MACE_TYPE_UINT8_T, 0, 3, offsetof(mace_heartbeat_t, mission_state) }, \
         { "mace_companion", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_heartbeat_t, mace_companion) }, \
         { "mavlink_version", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_heartbeat_t, mavlink_version) }, \
         { "mavlinkID", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_heartbeat_t, mavlinkID) }, \
         } \
}
#endif

/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param protocol Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
 * @param mission_state Defines the current state of the vehicle mission. Useful for determing the next state of the vehicle per mission state.
 * @param mace_companion Boolean describing whether(T=1) or not(F=0) the vehicle is MACE companion equipped.
 * @param mavlinkID 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t protocol, uint8_t type, uint8_t autopilot, uint8_t mission_state, uint8_t mace_companion, uint8_t mavlinkID)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_HEARTBEAT_LEN];
    _mace_put_uint8_t(buf, 0, protocol);
    _mace_put_uint8_t(buf, 1, type);
    _mace_put_uint8_t(buf, 2, autopilot);
    _mace_put_uint8_t(buf, 3, mission_state);
    _mace_put_uint8_t(buf, 4, mace_companion);
    _mace_put_uint8_t(buf, 5, 3);
    _mace_put_uint8_t(buf, 6, mavlinkID);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_HEARTBEAT_LEN);
#else
    mace_heartbeat_t packet;
    packet.protocol = protocol;
    packet.type = type;
    packet.autopilot = autopilot;
    packet.mission_state = mission_state;
    packet.mace_companion = mace_companion;
    packet.mavlink_version = 3;
    packet.mavlinkID = mavlinkID;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_HEARTBEAT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_HEARTBEAT;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_HEARTBEAT_MIN_LEN, MACE_MSG_ID_HEARTBEAT_LEN, MACE_MSG_ID_HEARTBEAT_CRC);
}

/**
 * @brief Pack a heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param protocol Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
 * @param mission_state Defines the current state of the vehicle mission. Useful for determing the next state of the vehicle per mission state.
 * @param mace_companion Boolean describing whether(T=1) or not(F=0) the vehicle is MACE companion equipped.
 * @param mavlinkID 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t protocol,uint8_t type,uint8_t autopilot,uint8_t mission_state,uint8_t mace_companion,uint8_t mavlinkID)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_HEARTBEAT_LEN];
    _mace_put_uint8_t(buf, 0, protocol);
    _mace_put_uint8_t(buf, 1, type);
    _mace_put_uint8_t(buf, 2, autopilot);
    _mace_put_uint8_t(buf, 3, mission_state);
    _mace_put_uint8_t(buf, 4, mace_companion);
    _mace_put_uint8_t(buf, 5, 3);
    _mace_put_uint8_t(buf, 6, mavlinkID);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_HEARTBEAT_LEN);
#else
    mace_heartbeat_t packet;
    packet.protocol = protocol;
    packet.type = type;
    packet.autopilot = autopilot;
    packet.mission_state = mission_state;
    packet.mace_companion = mace_companion;
    packet.mavlink_version = 3;
    packet.mavlinkID = mavlinkID;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_HEARTBEAT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_HEARTBEAT;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_HEARTBEAT_MIN_LEN, MACE_MSG_ID_HEARTBEAT_LEN, MACE_MSG_ID_HEARTBEAT_CRC);
}

/**
 * @brief Encode a heartbeat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mace_msg_heartbeat_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_heartbeat_t* heartbeat)
{
    return mace_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->protocol, heartbeat->type, heartbeat->autopilot, heartbeat->mission_state, heartbeat->mace_companion, heartbeat->mavlinkID);
}

/**
 * @brief Encode a heartbeat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mace_msg_heartbeat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_heartbeat_t* heartbeat)
{
    return mace_msg_heartbeat_pack_chan(system_id, component_id, chan, msg, heartbeat->protocol, heartbeat->type, heartbeat->autopilot, heartbeat->mission_state, heartbeat->mace_companion, heartbeat->mavlinkID);
}

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param protocol Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
 * @param mission_state Defines the current state of the vehicle mission. Useful for determing the next state of the vehicle per mission state.
 * @param mace_companion Boolean describing whether(T=1) or not(F=0) the vehicle is MACE companion equipped.
 * @param mavlinkID 
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_heartbeat_send(mace_channel_t chan, uint8_t protocol, uint8_t type, uint8_t autopilot, uint8_t mission_state, uint8_t mace_companion, uint8_t mavlinkID)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_HEARTBEAT_LEN];
    _mace_put_uint8_t(buf, 0, protocol);
    _mace_put_uint8_t(buf, 1, type);
    _mace_put_uint8_t(buf, 2, autopilot);
    _mace_put_uint8_t(buf, 3, mission_state);
    _mace_put_uint8_t(buf, 4, mace_companion);
    _mace_put_uint8_t(buf, 5, 3);
    _mace_put_uint8_t(buf, 6, mavlinkID);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_HEARTBEAT, buf, MACE_MSG_ID_HEARTBEAT_MIN_LEN, MACE_MSG_ID_HEARTBEAT_LEN, MACE_MSG_ID_HEARTBEAT_CRC);
#else
    mace_heartbeat_t packet;
    packet.protocol = protocol;
    packet.type = type;
    packet.autopilot = autopilot;
    packet.mission_state = mission_state;
    packet.mace_companion = mace_companion;
    packet.mavlink_version = 3;
    packet.mavlinkID = mavlinkID;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_HEARTBEAT, (const char *)&packet, MACE_MSG_ID_HEARTBEAT_MIN_LEN, MACE_MSG_ID_HEARTBEAT_LEN, MACE_MSG_ID_HEARTBEAT_CRC);
#endif
}

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_heartbeat_send_struct(mace_channel_t chan, const mace_heartbeat_t* heartbeat)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_heartbeat_send(chan, heartbeat->protocol, heartbeat->type, heartbeat->autopilot, heartbeat->mission_state, heartbeat->mace_companion, heartbeat->mavlinkID);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_HEARTBEAT, (const char *)heartbeat, MACE_MSG_ID_HEARTBEAT_MIN_LEN, MACE_MSG_ID_HEARTBEAT_LEN, MACE_MSG_ID_HEARTBEAT_CRC);
#endif
}

#if MACE_MSG_ID_HEARTBEAT_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_heartbeat_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t protocol, uint8_t type, uint8_t autopilot, uint8_t mission_state, uint8_t mace_companion, uint8_t mavlinkID)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, protocol);
    _mace_put_uint8_t(buf, 1, type);
    _mace_put_uint8_t(buf, 2, autopilot);
    _mace_put_uint8_t(buf, 3, mission_state);
    _mace_put_uint8_t(buf, 4, mace_companion);
    _mace_put_uint8_t(buf, 5, 3);
    _mace_put_uint8_t(buf, 6, mavlinkID);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_HEARTBEAT, buf, MACE_MSG_ID_HEARTBEAT_MIN_LEN, MACE_MSG_ID_HEARTBEAT_LEN, MACE_MSG_ID_HEARTBEAT_CRC);
#else
    mace_heartbeat_t *packet = (mace_heartbeat_t *)msgbuf;
    packet->protocol = protocol;
    packet->type = type;
    packet->autopilot = autopilot;
    packet->mission_state = mission_state;
    packet->mace_companion = mace_companion;
    packet->mavlink_version = 3;
    packet->mavlinkID = mavlinkID;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_HEARTBEAT, (const char *)packet, MACE_MSG_ID_HEARTBEAT_MIN_LEN, MACE_MSG_ID_HEARTBEAT_LEN, MACE_MSG_ID_HEARTBEAT_CRC);
#endif
}
#endif

#endif

// MESSAGE HEARTBEAT UNPACKING


/**
 * @brief Get field protocol from heartbeat message
 *
 * @return Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 */
static inline uint8_t mace_msg_heartbeat_get_protocol(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field type from heartbeat message
 *
 * @return Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 */
static inline uint8_t mace_msg_heartbeat_get_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field autopilot from heartbeat message
 *
 * @return Autopilot type / class. defined in MAV_AUTOPILOT ENUM
 */
static inline uint8_t mace_msg_heartbeat_get_autopilot(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field mission_state from heartbeat message
 *
 * @return Defines the current state of the vehicle mission. Useful for determing the next state of the vehicle per mission state.
 */
static inline uint8_t mace_msg_heartbeat_get_mission_state(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field mace_companion from heartbeat message
 *
 * @return Boolean describing whether(T=1) or not(F=0) the vehicle is MACE companion equipped.
 */
static inline uint8_t mace_msg_heartbeat_get_mace_companion(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field mavlink_version from heartbeat message
 *
 * @return MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
 */
static inline uint8_t mace_msg_heartbeat_get_mavlink_version(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field mavlinkID from heartbeat message
 *
 * @return 
 */
static inline uint8_t mace_msg_heartbeat_get_mavlinkID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heartbeat C-struct to decode the message contents into
 */
static inline void mace_msg_heartbeat_decode(const mace_message_t* msg, mace_heartbeat_t* heartbeat)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    heartbeat->protocol = mace_msg_heartbeat_get_protocol(msg);
    heartbeat->type = mace_msg_heartbeat_get_type(msg);
    heartbeat->autopilot = mace_msg_heartbeat_get_autopilot(msg);
    heartbeat->mission_state = mace_msg_heartbeat_get_mission_state(msg);
    heartbeat->mace_companion = mace_msg_heartbeat_get_mace_companion(msg);
    heartbeat->mavlink_version = mace_msg_heartbeat_get_mavlink_version(msg);
    heartbeat->mavlinkID = mace_msg_heartbeat_get_mavlinkID(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_HEARTBEAT_LEN? msg->len : MACE_MSG_ID_HEARTBEAT_LEN;
        memset(heartbeat, 0, MACE_MSG_ID_HEARTBEAT_LEN);
    memcpy(heartbeat, _MACE_PAYLOAD(msg), len);
#endif
}
