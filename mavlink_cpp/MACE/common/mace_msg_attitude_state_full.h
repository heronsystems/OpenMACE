#pragma once
// MESSAGE ATTITUDE_STATE_FULL PACKING

#define MACE_MSG_ID_ATTITUDE_STATE_FULL 18

MACEPACKED(
typedef struct __mace_attitude_state_full_t {
 float roll; /*< Roll angle (rad, -pi..+pi)*/
 float pitch; /*< Pitch angle (rad, -pi..+pi)*/
 float yaw; /*< Yaw angle (rad, -pi..+pi)*/
 float rollspeed; /*< Roll angular speed (rad/s)*/
 float pitchspeed; /*< Pitch angular speed (rad/s)*/
 float yawspeed; /*< Yaw angular speed (rad/s)*/
}) mace_attitude_state_full_t;

#define MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN 24
#define MACE_MSG_ID_ATTITUDE_STATE_FULL_MIN_LEN 24
#define MACE_MSG_ID_18_LEN 24
#define MACE_MSG_ID_18_MIN_LEN 24

#define MACE_MSG_ID_ATTITUDE_STATE_FULL_CRC 251
#define MACE_MSG_ID_18_CRC 251



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_ATTITUDE_STATE_FULL { \
    18, \
    "ATTITUDE_STATE_FULL", \
    6, \
    {  { "roll", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_attitude_state_full_t, roll) }, \
         { "pitch", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_attitude_state_full_t, pitch) }, \
         { "yaw", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_attitude_state_full_t, yaw) }, \
         { "rollspeed", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_attitude_state_full_t, rollspeed) }, \
         { "pitchspeed", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_attitude_state_full_t, pitchspeed) }, \
         { "yawspeed", NULL, MACE_TYPE_FLOAT, 0, 20, offsetof(mace_attitude_state_full_t, yawspeed) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_ATTITUDE_STATE_FULL { \
    "ATTITUDE_STATE_FULL", \
    6, \
    {  { "roll", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_attitude_state_full_t, roll) }, \
         { "pitch", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_attitude_state_full_t, pitch) }, \
         { "yaw", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_attitude_state_full_t, yaw) }, \
         { "rollspeed", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_attitude_state_full_t, rollspeed) }, \
         { "pitchspeed", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_attitude_state_full_t, pitchspeed) }, \
         { "yawspeed", NULL, MACE_TYPE_FLOAT, 0, 20, offsetof(mace_attitude_state_full_t, yawspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude_state_full message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_attitude_state_full_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN];
    _mace_put_float(buf, 0, roll);
    _mace_put_float(buf, 4, pitch);
    _mace_put_float(buf, 8, yaw);
    _mace_put_float(buf, 12, rollspeed);
    _mace_put_float(buf, 16, pitchspeed);
    _mace_put_float(buf, 20, yawspeed);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN);
#else
    mace_attitude_state_full_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN);
#endif

    msg->msgid = MACE_MSG_ID_ATTITUDE_STATE_FULL;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_ATTITUDE_STATE_FULL_MIN_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_CRC);
}

/**
 * @brief Pack a attitude_state_full message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_attitude_state_full_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   float roll,float pitch,float yaw,float rollspeed,float pitchspeed,float yawspeed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN];
    _mace_put_float(buf, 0, roll);
    _mace_put_float(buf, 4, pitch);
    _mace_put_float(buf, 8, yaw);
    _mace_put_float(buf, 12, rollspeed);
    _mace_put_float(buf, 16, pitchspeed);
    _mace_put_float(buf, 20, yawspeed);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN);
#else
    mace_attitude_state_full_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN);
#endif

    msg->msgid = MACE_MSG_ID_ATTITUDE_STATE_FULL;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_ATTITUDE_STATE_FULL_MIN_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_CRC);
}

/**
 * @brief Encode a attitude_state_full struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_state_full C-struct to read the message contents from
 */
static inline uint16_t mace_msg_attitude_state_full_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_attitude_state_full_t* attitude_state_full)
{
    return mace_msg_attitude_state_full_pack(system_id, component_id, msg, attitude_state_full->roll, attitude_state_full->pitch, attitude_state_full->yaw, attitude_state_full->rollspeed, attitude_state_full->pitchspeed, attitude_state_full->yawspeed);
}

/**
 * @brief Encode a attitude_state_full struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude_state_full C-struct to read the message contents from
 */
static inline uint16_t mace_msg_attitude_state_full_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_attitude_state_full_t* attitude_state_full)
{
    return mace_msg_attitude_state_full_pack_chan(system_id, component_id, chan, msg, attitude_state_full->roll, attitude_state_full->pitch, attitude_state_full->yaw, attitude_state_full->rollspeed, attitude_state_full->pitchspeed, attitude_state_full->yawspeed);
}

/**
 * @brief Send a attitude_state_full message
 * @param chan MAVLink channel to send the message
 *
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_attitude_state_full_send(mace_channel_t chan, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN];
    _mace_put_float(buf, 0, roll);
    _mace_put_float(buf, 4, pitch);
    _mace_put_float(buf, 8, yaw);
    _mace_put_float(buf, 12, rollspeed);
    _mace_put_float(buf, 16, pitchspeed);
    _mace_put_float(buf, 20, yawspeed);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_STATE_FULL, buf, MACE_MSG_ID_ATTITUDE_STATE_FULL_MIN_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_CRC);
#else
    mace_attitude_state_full_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_STATE_FULL, (const char *)&packet, MACE_MSG_ID_ATTITUDE_STATE_FULL_MIN_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_CRC);
#endif
}

/**
 * @brief Send a attitude_state_full message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_attitude_state_full_send_struct(mace_channel_t chan, const mace_attitude_state_full_t* attitude_state_full)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_attitude_state_full_send(chan, attitude_state_full->roll, attitude_state_full->pitch, attitude_state_full->yaw, attitude_state_full->rollspeed, attitude_state_full->pitchspeed, attitude_state_full->yawspeed);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_STATE_FULL, (const char *)attitude_state_full, MACE_MSG_ID_ATTITUDE_STATE_FULL_MIN_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_CRC);
#endif
}

#if MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_attitude_state_full_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_float(buf, 0, roll);
    _mace_put_float(buf, 4, pitch);
    _mace_put_float(buf, 8, yaw);
    _mace_put_float(buf, 12, rollspeed);
    _mace_put_float(buf, 16, pitchspeed);
    _mace_put_float(buf, 20, yawspeed);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_STATE_FULL, buf, MACE_MSG_ID_ATTITUDE_STATE_FULL_MIN_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_CRC);
#else
    mace_attitude_state_full_t *packet = (mace_attitude_state_full_t *)msgbuf;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->rollspeed = rollspeed;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_STATE_FULL, (const char *)packet, MACE_MSG_ID_ATTITUDE_STATE_FULL_MIN_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN, MACE_MSG_ID_ATTITUDE_STATE_FULL_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE_STATE_FULL UNPACKING


/**
 * @brief Get field roll from attitude_state_full message
 *
 * @return Roll angle (rad, -pi..+pi)
 */
static inline float mace_msg_attitude_state_full_get_roll(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from attitude_state_full message
 *
 * @return Pitch angle (rad, -pi..+pi)
 */
static inline float mace_msg_attitude_state_full_get_pitch(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from attitude_state_full message
 *
 * @return Yaw angle (rad, -pi..+pi)
 */
static inline float mace_msg_attitude_state_full_get_yaw(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field rollspeed from attitude_state_full message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mace_msg_attitude_state_full_get_rollspeed(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitchspeed from attitude_state_full message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mace_msg_attitude_state_full_get_pitchspeed(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  16);
}

/**
 * @brief Get field yawspeed from attitude_state_full message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mace_msg_attitude_state_full_get_yawspeed(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  20);
}

/**
 * @brief Decode a attitude_state_full message into a struct
 *
 * @param msg The message to decode
 * @param attitude_state_full C-struct to decode the message contents into
 */
static inline void mace_msg_attitude_state_full_decode(const mace_message_t* msg, mace_attitude_state_full_t* attitude_state_full)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    attitude_state_full->roll = mace_msg_attitude_state_full_get_roll(msg);
    attitude_state_full->pitch = mace_msg_attitude_state_full_get_pitch(msg);
    attitude_state_full->yaw = mace_msg_attitude_state_full_get_yaw(msg);
    attitude_state_full->rollspeed = mace_msg_attitude_state_full_get_rollspeed(msg);
    attitude_state_full->pitchspeed = mace_msg_attitude_state_full_get_pitchspeed(msg);
    attitude_state_full->yawspeed = mace_msg_attitude_state_full_get_yawspeed(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN? msg->len : MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN;
        memset(attitude_state_full, 0, MACE_MSG_ID_ATTITUDE_STATE_FULL_LEN);
    memcpy(attitude_state_full, _MACE_PAYLOAD(msg), len);
#endif
}
