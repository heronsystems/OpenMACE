#pragma once
// MESSAGE ATTITUDE PACKING

#define MACE_MSG_ID_ATTITUDE 16

MACEPACKED(
typedef struct __mace_attitude_t {
 float roll; /*< Roll angle (rad, -pi..+pi)*/
 float pitch; /*< Pitch angle (rad, -pi..+pi)*/
 float yaw; /*< Yaw angle (rad, -pi..+pi)*/
}) mace_attitude_t;

#define MACE_MSG_ID_ATTITUDE_LEN 12
#define MACE_MSG_ID_ATTITUDE_MIN_LEN 12
#define MACE_MSG_ID_16_LEN 12
#define MACE_MSG_ID_16_MIN_LEN 12

#define MACE_MSG_ID_ATTITUDE_CRC 61
#define MACE_MSG_ID_16_CRC 61



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_ATTITUDE { \
    16, \
    "ATTITUDE", \
    3, \
    {  { "roll", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_attitude_t, roll) }, \
         { "pitch", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_attitude_t, pitch) }, \
         { "yaw", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_attitude_t, yaw) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_ATTITUDE { \
    "ATTITUDE", \
    3, \
    {  { "roll", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_attitude_t, roll) }, \
         { "pitch", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_attitude_t, pitch) }, \
         { "yaw", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_attitude_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_attitude_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               float roll, float pitch, float yaw)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ATTITUDE_LEN];
    _mace_put_float(buf, 0, roll);
    _mace_put_float(buf, 4, pitch);
    _mace_put_float(buf, 8, yaw);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_ATTITUDE_LEN);
#else
    mace_attitude_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_ATTITUDE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_ATTITUDE;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_ATTITUDE_MIN_LEN, MACE_MSG_ID_ATTITUDE_LEN, MACE_MSG_ID_ATTITUDE_CRC);
}

/**
 * @brief Pack a attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   float roll,float pitch,float yaw)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ATTITUDE_LEN];
    _mace_put_float(buf, 0, roll);
    _mace_put_float(buf, 4, pitch);
    _mace_put_float(buf, 8, yaw);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_ATTITUDE_LEN);
#else
    mace_attitude_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_ATTITUDE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_ATTITUDE;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_ATTITUDE_MIN_LEN, MACE_MSG_ID_ATTITUDE_LEN, MACE_MSG_ID_ATTITUDE_CRC);
}

/**
 * @brief Encode a attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude C-struct to read the message contents from
 */
static inline uint16_t mace_msg_attitude_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_attitude_t* attitude)
{
    return mace_msg_attitude_pack(system_id, component_id, msg, attitude->roll, attitude->pitch, attitude->yaw);
}

/**
 * @brief Encode a attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude C-struct to read the message contents from
 */
static inline uint16_t mace_msg_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_attitude_t* attitude)
{
    return mace_msg_attitude_pack_chan(system_id, component_id, chan, msg, attitude->roll, attitude->pitch, attitude->yaw);
}

/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_attitude_send(mace_channel_t chan, float roll, float pitch, float yaw)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ATTITUDE_LEN];
    _mace_put_float(buf, 0, roll);
    _mace_put_float(buf, 4, pitch);
    _mace_put_float(buf, 8, yaw);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE, buf, MACE_MSG_ID_ATTITUDE_MIN_LEN, MACE_MSG_ID_ATTITUDE_LEN, MACE_MSG_ID_ATTITUDE_CRC);
#else
    mace_attitude_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE, (const char *)&packet, MACE_MSG_ID_ATTITUDE_MIN_LEN, MACE_MSG_ID_ATTITUDE_LEN, MACE_MSG_ID_ATTITUDE_CRC);
#endif
}

/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_attitude_send_struct(mace_channel_t chan, const mace_attitude_t* attitude)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_attitude_send(chan, attitude->roll, attitude->pitch, attitude->yaw);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE, (const char *)attitude, MACE_MSG_ID_ATTITUDE_MIN_LEN, MACE_MSG_ID_ATTITUDE_LEN, MACE_MSG_ID_ATTITUDE_CRC);
#endif
}

#if MACE_MSG_ID_ATTITUDE_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_attitude_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  float roll, float pitch, float yaw)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_float(buf, 0, roll);
    _mace_put_float(buf, 4, pitch);
    _mace_put_float(buf, 8, yaw);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE, buf, MACE_MSG_ID_ATTITUDE_MIN_LEN, MACE_MSG_ID_ATTITUDE_LEN, MACE_MSG_ID_ATTITUDE_CRC);
#else
    mace_attitude_t *packet = (mace_attitude_t *)msgbuf;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE, (const char *)packet, MACE_MSG_ID_ATTITUDE_MIN_LEN, MACE_MSG_ID_ATTITUDE_LEN, MACE_MSG_ID_ATTITUDE_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE UNPACKING


/**
 * @brief Get field roll from attitude message
 *
 * @return Roll angle (rad, -pi..+pi)
 */
static inline float mace_msg_attitude_get_roll(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from attitude message
 *
 * @return Pitch angle (rad, -pi..+pi)
 */
static inline float mace_msg_attitude_get_pitch(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from attitude message
 *
 * @return Yaw angle (rad, -pi..+pi)
 */
static inline float mace_msg_attitude_get_yaw(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Decode a attitude message into a struct
 *
 * @param msg The message to decode
 * @param attitude C-struct to decode the message contents into
 */
static inline void mace_msg_attitude_decode(const mace_message_t* msg, mace_attitude_t* attitude)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    attitude->roll = mace_msg_attitude_get_roll(msg);
    attitude->pitch = mace_msg_attitude_get_pitch(msg);
    attitude->yaw = mace_msg_attitude_get_yaw(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_ATTITUDE_LEN? msg->len : MACE_MSG_ID_ATTITUDE_LEN;
        memset(attitude, 0, MACE_MSG_ID_ATTITUDE_LEN);
    memcpy(attitude, _MACE_PAYLOAD(msg), len);
#endif
}
