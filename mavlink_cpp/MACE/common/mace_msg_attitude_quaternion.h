#pragma once
// MESSAGE ATTITUDE_QUATERNION PACKING

#define MACE_MSG_ID_ATTITUDE_QUATERNION 19

MACEPACKED(
typedef struct __mace_attitude_quaternion_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float q1; /*< Quaternion component 1, w (1 in null-rotation)*/
 float q2; /*< Quaternion component 2, x (0 in null-rotation)*/
 float q3; /*< Quaternion component 3, y (0 in null-rotation)*/
 float q4; /*< Quaternion component 4, z (0 in null-rotation)*/
 float rollspeed; /*< Roll angular speed (rad/s)*/
 float pitchspeed; /*< Pitch angular speed (rad/s)*/
 float yawspeed; /*< Yaw angular speed (rad/s)*/
}) mace_attitude_quaternion_t;

#define MACE_MSG_ID_ATTITUDE_QUATERNION_LEN 32
#define MACE_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN 32
#define MACE_MSG_ID_19_LEN 32
#define MACE_MSG_ID_19_MIN_LEN 32

#define MACE_MSG_ID_ATTITUDE_QUATERNION_CRC 246
#define MACE_MSG_ID_19_CRC 246



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_ATTITUDE_QUATERNION { \
    19, \
    "ATTITUDE_QUATERNION", \
    8, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_attitude_quaternion_t, time_boot_ms) }, \
         { "q1", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_attitude_quaternion_t, q1) }, \
         { "q2", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_attitude_quaternion_t, q2) }, \
         { "q3", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_attitude_quaternion_t, q3) }, \
         { "q4", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_attitude_quaternion_t, q4) }, \
         { "rollspeed", NULL, MACE_TYPE_FLOAT, 0, 20, offsetof(mace_attitude_quaternion_t, rollspeed) }, \
         { "pitchspeed", NULL, MACE_TYPE_FLOAT, 0, 24, offsetof(mace_attitude_quaternion_t, pitchspeed) }, \
         { "yawspeed", NULL, MACE_TYPE_FLOAT, 0, 28, offsetof(mace_attitude_quaternion_t, yawspeed) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_ATTITUDE_QUATERNION { \
    "ATTITUDE_QUATERNION", \
    8, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_attitude_quaternion_t, time_boot_ms) }, \
         { "q1", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_attitude_quaternion_t, q1) }, \
         { "q2", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_attitude_quaternion_t, q2) }, \
         { "q3", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_attitude_quaternion_t, q3) }, \
         { "q4", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_attitude_quaternion_t, q4) }, \
         { "rollspeed", NULL, MACE_TYPE_FLOAT, 0, 20, offsetof(mace_attitude_quaternion_t, rollspeed) }, \
         { "pitchspeed", NULL, MACE_TYPE_FLOAT, 0, 24, offsetof(mace_attitude_quaternion_t, pitchspeed) }, \
         { "yawspeed", NULL, MACE_TYPE_FLOAT, 0, 28, offsetof(mace_attitude_quaternion_t, yawspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude_quaternion message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param q1 Quaternion component 1, w (1 in null-rotation)
 * @param q2 Quaternion component 2, x (0 in null-rotation)
 * @param q3 Quaternion component 3, y (0 in null-rotation)
 * @param q4 Quaternion component 4, z (0 in null-rotation)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_attitude_quaternion_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ATTITUDE_QUATERNION_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, q1);
    _mace_put_float(buf, 8, q2);
    _mace_put_float(buf, 12, q3);
    _mace_put_float(buf, 16, q4);
    _mace_put_float(buf, 20, rollspeed);
    _mace_put_float(buf, 24, pitchspeed);
    _mace_put_float(buf, 28, yawspeed);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_ATTITUDE_QUATERNION_LEN);
#else
    mace_attitude_quaternion_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.q4 = q4;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_ATTITUDE_QUATERNION_LEN);
#endif

    msg->msgid = MACE_MSG_ID_ATTITUDE_QUATERNION;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_CRC);
}

/**
 * @brief Pack a attitude_quaternion message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param q1 Quaternion component 1, w (1 in null-rotation)
 * @param q2 Quaternion component 2, x (0 in null-rotation)
 * @param q3 Quaternion component 3, y (0 in null-rotation)
 * @param q4 Quaternion component 4, z (0 in null-rotation)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_attitude_quaternion_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint32_t time_boot_ms,float q1,float q2,float q3,float q4,float rollspeed,float pitchspeed,float yawspeed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ATTITUDE_QUATERNION_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, q1);
    _mace_put_float(buf, 8, q2);
    _mace_put_float(buf, 12, q3);
    _mace_put_float(buf, 16, q4);
    _mace_put_float(buf, 20, rollspeed);
    _mace_put_float(buf, 24, pitchspeed);
    _mace_put_float(buf, 28, yawspeed);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_ATTITUDE_QUATERNION_LEN);
#else
    mace_attitude_quaternion_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.q4 = q4;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_ATTITUDE_QUATERNION_LEN);
#endif

    msg->msgid = MACE_MSG_ID_ATTITUDE_QUATERNION;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_CRC);
}

/**
 * @brief Encode a attitude_quaternion struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_quaternion C-struct to read the message contents from
 */
static inline uint16_t mace_msg_attitude_quaternion_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_attitude_quaternion_t* attitude_quaternion)
{
    return mace_msg_attitude_quaternion_pack(system_id, component_id, msg, attitude_quaternion->time_boot_ms, attitude_quaternion->q1, attitude_quaternion->q2, attitude_quaternion->q3, attitude_quaternion->q4, attitude_quaternion->rollspeed, attitude_quaternion->pitchspeed, attitude_quaternion->yawspeed);
}

/**
 * @brief Encode a attitude_quaternion struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude_quaternion C-struct to read the message contents from
 */
static inline uint16_t mace_msg_attitude_quaternion_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_attitude_quaternion_t* attitude_quaternion)
{
    return mace_msg_attitude_quaternion_pack_chan(system_id, component_id, chan, msg, attitude_quaternion->time_boot_ms, attitude_quaternion->q1, attitude_quaternion->q2, attitude_quaternion->q3, attitude_quaternion->q4, attitude_quaternion->rollspeed, attitude_quaternion->pitchspeed, attitude_quaternion->yawspeed);
}

/**
 * @brief Send a attitude_quaternion message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param q1 Quaternion component 1, w (1 in null-rotation)
 * @param q2 Quaternion component 2, x (0 in null-rotation)
 * @param q3 Quaternion component 3, y (0 in null-rotation)
 * @param q4 Quaternion component 4, z (0 in null-rotation)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_attitude_quaternion_send(mace_channel_t chan, uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ATTITUDE_QUATERNION_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, q1);
    _mace_put_float(buf, 8, q2);
    _mace_put_float(buf, 12, q3);
    _mace_put_float(buf, 16, q4);
    _mace_put_float(buf, 20, rollspeed);
    _mace_put_float(buf, 24, pitchspeed);
    _mace_put_float(buf, 28, yawspeed);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_QUATERNION, buf, MACE_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_CRC);
#else
    mace_attitude_quaternion_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.q1 = q1;
    packet.q2 = q2;
    packet.q3 = q3;
    packet.q4 = q4;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_QUATERNION, (const char *)&packet, MACE_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_CRC);
#endif
}

/**
 * @brief Send a attitude_quaternion message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_attitude_quaternion_send_struct(mace_channel_t chan, const mace_attitude_quaternion_t* attitude_quaternion)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_attitude_quaternion_send(chan, attitude_quaternion->time_boot_ms, attitude_quaternion->q1, attitude_quaternion->q2, attitude_quaternion->q3, attitude_quaternion->q4, attitude_quaternion->rollspeed, attitude_quaternion->pitchspeed, attitude_quaternion->yawspeed);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_QUATERNION, (const char *)attitude_quaternion, MACE_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_CRC);
#endif
}

#if MACE_MSG_ID_ATTITUDE_QUATERNION_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_attitude_quaternion_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, q1);
    _mace_put_float(buf, 8, q2);
    _mace_put_float(buf, 12, q3);
    _mace_put_float(buf, 16, q4);
    _mace_put_float(buf, 20, rollspeed);
    _mace_put_float(buf, 24, pitchspeed);
    _mace_put_float(buf, 28, yawspeed);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_QUATERNION, buf, MACE_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_CRC);
#else
    mace_attitude_quaternion_t *packet = (mace_attitude_quaternion_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->q1 = q1;
    packet->q2 = q2;
    packet->q3 = q3;
    packet->q4 = q4;
    packet->rollspeed = rollspeed;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ATTITUDE_QUATERNION, (const char *)packet, MACE_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_LEN, MACE_MSG_ID_ATTITUDE_QUATERNION_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE_QUATERNION UNPACKING


/**
 * @brief Get field time_boot_ms from attitude_quaternion message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mace_msg_attitude_quaternion_get_time_boot_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field q1 from attitude_quaternion message
 *
 * @return Quaternion component 1, w (1 in null-rotation)
 */
static inline float mace_msg_attitude_quaternion_get_q1(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field q2 from attitude_quaternion message
 *
 * @return Quaternion component 2, x (0 in null-rotation)
 */
static inline float mace_msg_attitude_quaternion_get_q2(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field q3 from attitude_quaternion message
 *
 * @return Quaternion component 3, y (0 in null-rotation)
 */
static inline float mace_msg_attitude_quaternion_get_q3(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Get field q4 from attitude_quaternion message
 *
 * @return Quaternion component 4, z (0 in null-rotation)
 */
static inline float mace_msg_attitude_quaternion_get_q4(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  16);
}

/**
 * @brief Get field rollspeed from attitude_quaternion message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mace_msg_attitude_quaternion_get_rollspeed(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitchspeed from attitude_quaternion message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mace_msg_attitude_quaternion_get_pitchspeed(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  24);
}

/**
 * @brief Get field yawspeed from attitude_quaternion message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mace_msg_attitude_quaternion_get_yawspeed(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  28);
}

/**
 * @brief Decode a attitude_quaternion message into a struct
 *
 * @param msg The message to decode
 * @param attitude_quaternion C-struct to decode the message contents into
 */
static inline void mace_msg_attitude_quaternion_decode(const mace_message_t* msg, mace_attitude_quaternion_t* attitude_quaternion)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    attitude_quaternion->time_boot_ms = mace_msg_attitude_quaternion_get_time_boot_ms(msg);
    attitude_quaternion->q1 = mace_msg_attitude_quaternion_get_q1(msg);
    attitude_quaternion->q2 = mace_msg_attitude_quaternion_get_q2(msg);
    attitude_quaternion->q3 = mace_msg_attitude_quaternion_get_q3(msg);
    attitude_quaternion->q4 = mace_msg_attitude_quaternion_get_q4(msg);
    attitude_quaternion->rollspeed = mace_msg_attitude_quaternion_get_rollspeed(msg);
    attitude_quaternion->pitchspeed = mace_msg_attitude_quaternion_get_pitchspeed(msg);
    attitude_quaternion->yawspeed = mace_msg_attitude_quaternion_get_yawspeed(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_ATTITUDE_QUATERNION_LEN? msg->len : MACE_MSG_ID_ATTITUDE_QUATERNION_LEN;
        memset(attitude_quaternion, 0, MACE_MSG_ID_ATTITUDE_QUATERNION_LEN);
    memcpy(attitude_quaternion, _MACE_PAYLOAD(msg), len);
#endif
}
