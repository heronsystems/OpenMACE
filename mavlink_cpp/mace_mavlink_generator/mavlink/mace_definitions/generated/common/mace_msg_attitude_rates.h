#pragma once
// MESSAGE ATTITUDE_RATES PACKING

#define MACE_MSG_ID_ATTITUDE_RATES 17

MACEPACKED(
typedef struct __mace_attitude_rates_t {
 float rollspeed; /*< Roll angular speed (rad/s)*/
 float pitchspeed; /*< Pitch angular speed (rad/s)*/
 float yawspeed; /*< Yaw angular speed (rad/s)*/
}) mace_attitude_rates_t;

#define MACE_MSG_ID_ATTITUDE_RATES_LEN 12
#define MACE_MSG_ID_ATTITUDE_RATES_MIN_LEN 12
#define MACE_MSG_ID_17_LEN 12
#define MACE_MSG_ID_17_MIN_LEN 12

#define MACE_MSG_ID_ATTITUDE_RATES_CRC 239
#define MACE_MSG_ID_17_CRC 239



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_ATTITUDE_RATES { \
    17, \
    "ATTITUDE_RATES", \
    3, \
    {  { "rollspeed", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_attitude_rates_t, rollspeed) }, \
         { "pitchspeed", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_attitude_rates_t, pitchspeed) }, \
         { "yawspeed", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_attitude_rates_t, yawspeed) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_ATTITUDE_RATES { \
    "ATTITUDE_RATES", \
    3, \
    {  { "rollspeed", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_attitude_rates_t, rollspeed) }, \
         { "pitchspeed", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_attitude_rates_t, pitchspeed) }, \
         { "yawspeed", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_attitude_rates_t, yawspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude_rates message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_attitude_rates_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               float rollspeed, float pitchspeed, float yawspeed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ATTITUDE_RATES_LEN];
    _mace_put_float(buf, 0, rollspeed);
    _mace_put_float(buf, 4, pitchspeed);
    _mace_put_float(buf, 8, yawspeed);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_ATTITUDE_RATES_LEN);
#else
    mace_attitude_rates_t packet;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_ATTITUDE_RATES_LEN);
#endif

    msg->msgid = MACE_MSG_ID_ATTITUDE_RATES;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_ATTITUDE_RATES_MIN_LEN, MACE_MSG_ID_ATTITUDE_RATES_LEN, MACE_MSG_ID_ATTITUDE_RATES_CRC);
}

/**
 * @brief Pack a attitude_rates message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_attitude_rates_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   float rollspeed,float pitchspeed,float yawspeed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ATTITUDE_RATES_LEN];
    _mace_put_float(buf, 0, rollspeed);
    _mace_put_float(buf, 4, pitchspeed);
    _mace_put_float(buf, 8, yawspeed);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_ATTITUDE_RATES_LEN);
#else
    mace_attitude_rates_t packet;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_ATTITUDE_RATES_LEN);
#endif

    msg->msgid = MACE_MSG_ID_ATTITUDE_RATES;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_ATTITUDE_RATES_MIN_LEN, MACE_MSG_ID_ATTITUDE_RATES_LEN, MACE_MSG_ID_ATTITUDE_RATES_CRC);
}

/**
 * @brief Encode a attitude_rates struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_rates C-struct to read the message contents from
 */
static inline uint16_t mace_msg_attitude_rates_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_attitude_rates_t* attitude_rates)
{
    return mace_msg_attitude_rates_pack(system_id, component_id, msg, attitude_rates->rollspeed, attitude_rates->pitchspeed, attitude_rates->yawspeed);
}

/**
 * @brief Encode a attitude_rates struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude_rates C-struct to read the message contents from
 */
static inline uint16_t mace_msg_attitude_rates_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_attitude_rates_t* attitude_rates)
{
    return mace_msg_attitude_rates_pack_chan(system_id, component_id, chan, msg, attitude_rates->rollspeed, attitude_rates->pitchspeed, attitude_rates->yawspeed);
}

/**
 * @brief Send a attitude_rates message
 * @param chan MAVLink channel to send the message
 *
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_attitude_rates_send(mace_channel_t chan, float rollspeed, float pitchspeed, float yawspeed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ATTITUDE_RATES_LEN];
    _mace_put_float(buf, 0, rollspeed);
    _mace_put_float(buf, 4, pitchspeed);
    _mace_put_float(buf, 8, yawspeed);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_RATES, buf, MACE_MSG_ID_ATTITUDE_RATES_MIN_LEN, MACE_MSG_ID_ATTITUDE_RATES_LEN, MACE_MSG_ID_ATTITUDE_RATES_CRC);
#else
    mace_attitude_rates_t packet;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_RATES, (const char *)&packet, MACE_MSG_ID_ATTITUDE_RATES_MIN_LEN, MACE_MSG_ID_ATTITUDE_RATES_LEN, MACE_MSG_ID_ATTITUDE_RATES_CRC);
#endif
}

/**
 * @brief Send a attitude_rates message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_attitude_rates_send_struct(mace_channel_t chan, const mace_attitude_rates_t* attitude_rates)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_attitude_rates_send(chan, attitude_rates->rollspeed, attitude_rates->pitchspeed, attitude_rates->yawspeed);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_RATES, (const char *)attitude_rates, MACE_MSG_ID_ATTITUDE_RATES_MIN_LEN, MACE_MSG_ID_ATTITUDE_RATES_LEN, MACE_MSG_ID_ATTITUDE_RATES_CRC);
#endif
}

#if MACE_MSG_ID_ATTITUDE_RATES_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_attitude_rates_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  float rollspeed, float pitchspeed, float yawspeed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_float(buf, 0, rollspeed);
    _mace_put_float(buf, 4, pitchspeed);
    _mace_put_float(buf, 8, yawspeed);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_RATES, buf, MACE_MSG_ID_ATTITUDE_RATES_MIN_LEN, MACE_MSG_ID_ATTITUDE_RATES_LEN, MACE_MSG_ID_ATTITUDE_RATES_CRC);
#else
    mace_attitude_rates_t *packet = (mace_attitude_rates_t *)msgbuf;
    packet->rollspeed = rollspeed;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_RATES, (const char *)packet, MACE_MSG_ID_ATTITUDE_RATES_MIN_LEN, MACE_MSG_ID_ATTITUDE_RATES_LEN, MACE_MSG_ID_ATTITUDE_RATES_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE_RATES UNPACKING


/**
 * @brief Get field rollspeed from attitude_rates message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mace_msg_attitude_rates_get_rollspeed(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitchspeed from attitude_rates message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mace_msg_attitude_rates_get_pitchspeed(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field yawspeed from attitude_rates message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mace_msg_attitude_rates_get_yawspeed(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Decode a attitude_rates message into a struct
 *
 * @param msg The message to decode
 * @param attitude_rates C-struct to decode the message contents into
 */
static inline void mace_msg_attitude_rates_decode(const mace_message_t* msg, mace_attitude_rates_t* attitude_rates)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    attitude_rates->rollspeed = mace_msg_attitude_rates_get_rollspeed(msg);
    attitude_rates->pitchspeed = mace_msg_attitude_rates_get_pitchspeed(msg);
    attitude_rates->yawspeed = mace_msg_attitude_rates_get_yawspeed(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_ATTITUDE_RATES_LEN? msg->len : MACE_MSG_ID_ATTITUDE_RATES_LEN;
        memset(attitude_rates, 0, MACE_MSG_ID_ATTITUDE_RATES_LEN);
    memcpy(attitude_rates, _MACE_PAYLOAD(msg), len);
#endif
}
