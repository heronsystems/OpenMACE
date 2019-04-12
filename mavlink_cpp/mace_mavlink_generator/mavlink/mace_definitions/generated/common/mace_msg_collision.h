#pragma once
// MESSAGE COLLISION PACKING

#define MACE_MSG_ID_COLLISION 247

MACEPACKED(
typedef struct __mace_collision_t {
 uint32_t id; /*< Unique identifier, domain based on src field*/
 float time_to_minimum_delta; /*< Estimated time until collision occurs (seconds)*/
 float altitude_minimum_delta; /*< Closest vertical distance in meters between vehicle and object*/
 float horizontal_minimum_delta; /*< Closest horizontal distance in meteres between vehicle and object*/
 uint8_t src; /*< Collision data source*/
 uint8_t action; /*< Action that is being taken to avoid this collision*/
 uint8_t threat_level; /*< How concerned the aircraft is about this collision*/
}) mace_collision_t;

#define MACE_MSG_ID_COLLISION_LEN 19
#define MACE_MSG_ID_COLLISION_MIN_LEN 19
#define MACE_MSG_ID_247_LEN 19
#define MACE_MSG_ID_247_MIN_LEN 19

#define MACE_MSG_ID_COLLISION_CRC 81
#define MACE_MSG_ID_247_CRC 81



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_COLLISION { \
    247, \
    "COLLISION", \
    7, \
    {  { "id", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_collision_t, id) }, \
         { "time_to_minimum_delta", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_collision_t, time_to_minimum_delta) }, \
         { "altitude_minimum_delta", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_collision_t, altitude_minimum_delta) }, \
         { "horizontal_minimum_delta", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_collision_t, horizontal_minimum_delta) }, \
         { "src", NULL, MACE_TYPE_UINT8_T, 0, 16, offsetof(mace_collision_t, src) }, \
         { "action", NULL, MACE_TYPE_UINT8_T, 0, 17, offsetof(mace_collision_t, action) }, \
         { "threat_level", NULL, MACE_TYPE_UINT8_T, 0, 18, offsetof(mace_collision_t, threat_level) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_COLLISION { \
    "COLLISION", \
    7, \
    {  { "id", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_collision_t, id) }, \
         { "time_to_minimum_delta", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_collision_t, time_to_minimum_delta) }, \
         { "altitude_minimum_delta", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_collision_t, altitude_minimum_delta) }, \
         { "horizontal_minimum_delta", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_collision_t, horizontal_minimum_delta) }, \
         { "src", NULL, MACE_TYPE_UINT8_T, 0, 16, offsetof(mace_collision_t, src) }, \
         { "action", NULL, MACE_TYPE_UINT8_T, 0, 17, offsetof(mace_collision_t, action) }, \
         { "threat_level", NULL, MACE_TYPE_UINT8_T, 0, 18, offsetof(mace_collision_t, threat_level) }, \
         } \
}
#endif

/**
 * @brief Pack a collision message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param src Collision data source
 * @param id Unique identifier, domain based on src field
 * @param action Action that is being taken to avoid this collision
 * @param threat_level How concerned the aircraft is about this collision
 * @param time_to_minimum_delta Estimated time until collision occurs (seconds)
 * @param altitude_minimum_delta Closest vertical distance in meters between vehicle and object
 * @param horizontal_minimum_delta Closest horizontal distance in meteres between vehicle and object
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_collision_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COLLISION_LEN];
    _mace_put_uint32_t(buf, 0, id);
    _mace_put_float(buf, 4, time_to_minimum_delta);
    _mace_put_float(buf, 8, altitude_minimum_delta);
    _mace_put_float(buf, 12, horizontal_minimum_delta);
    _mace_put_uint8_t(buf, 16, src);
    _mace_put_uint8_t(buf, 17, action);
    _mace_put_uint8_t(buf, 18, threat_level);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_COLLISION_LEN);
#else
    mace_collision_t packet;
    packet.id = id;
    packet.time_to_minimum_delta = time_to_minimum_delta;
    packet.altitude_minimum_delta = altitude_minimum_delta;
    packet.horizontal_minimum_delta = horizontal_minimum_delta;
    packet.src = src;
    packet.action = action;
    packet.threat_level = threat_level;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_COLLISION_LEN);
#endif

    msg->msgid = MACE_MSG_ID_COLLISION;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_COLLISION_MIN_LEN, MACE_MSG_ID_COLLISION_LEN, MACE_MSG_ID_COLLISION_CRC);
}

/**
 * @brief Pack a collision message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param src Collision data source
 * @param id Unique identifier, domain based on src field
 * @param action Action that is being taken to avoid this collision
 * @param threat_level How concerned the aircraft is about this collision
 * @param time_to_minimum_delta Estimated time until collision occurs (seconds)
 * @param altitude_minimum_delta Closest vertical distance in meters between vehicle and object
 * @param horizontal_minimum_delta Closest horizontal distance in meteres between vehicle and object
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_collision_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t src,uint32_t id,uint8_t action,uint8_t threat_level,float time_to_minimum_delta,float altitude_minimum_delta,float horizontal_minimum_delta)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COLLISION_LEN];
    _mace_put_uint32_t(buf, 0, id);
    _mace_put_float(buf, 4, time_to_minimum_delta);
    _mace_put_float(buf, 8, altitude_minimum_delta);
    _mace_put_float(buf, 12, horizontal_minimum_delta);
    _mace_put_uint8_t(buf, 16, src);
    _mace_put_uint8_t(buf, 17, action);
    _mace_put_uint8_t(buf, 18, threat_level);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_COLLISION_LEN);
#else
    mace_collision_t packet;
    packet.id = id;
    packet.time_to_minimum_delta = time_to_minimum_delta;
    packet.altitude_minimum_delta = altitude_minimum_delta;
    packet.horizontal_minimum_delta = horizontal_minimum_delta;
    packet.src = src;
    packet.action = action;
    packet.threat_level = threat_level;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_COLLISION_LEN);
#endif

    msg->msgid = MACE_MSG_ID_COLLISION;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_COLLISION_MIN_LEN, MACE_MSG_ID_COLLISION_LEN, MACE_MSG_ID_COLLISION_CRC);
}

/**
 * @brief Encode a collision struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param collision C-struct to read the message contents from
 */
static inline uint16_t mace_msg_collision_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_collision_t* collision)
{
    return mace_msg_collision_pack(system_id, component_id, msg, collision->src, collision->id, collision->action, collision->threat_level, collision->time_to_minimum_delta, collision->altitude_minimum_delta, collision->horizontal_minimum_delta);
}

/**
 * @brief Encode a collision struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param collision C-struct to read the message contents from
 */
static inline uint16_t mace_msg_collision_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_collision_t* collision)
{
    return mace_msg_collision_pack_chan(system_id, component_id, chan, msg, collision->src, collision->id, collision->action, collision->threat_level, collision->time_to_minimum_delta, collision->altitude_minimum_delta, collision->horizontal_minimum_delta);
}

/**
 * @brief Send a collision message
 * @param chan MAVLink channel to send the message
 *
 * @param src Collision data source
 * @param id Unique identifier, domain based on src field
 * @param action Action that is being taken to avoid this collision
 * @param threat_level How concerned the aircraft is about this collision
 * @param time_to_minimum_delta Estimated time until collision occurs (seconds)
 * @param altitude_minimum_delta Closest vertical distance in meters between vehicle and object
 * @param horizontal_minimum_delta Closest horizontal distance in meteres between vehicle and object
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_collision_send(mace_channel_t chan, uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_COLLISION_LEN];
    _mace_put_uint32_t(buf, 0, id);
    _mace_put_float(buf, 4, time_to_minimum_delta);
    _mace_put_float(buf, 8, altitude_minimum_delta);
    _mace_put_float(buf, 12, horizontal_minimum_delta);
    _mace_put_uint8_t(buf, 16, src);
    _mace_put_uint8_t(buf, 17, action);
    _mace_put_uint8_t(buf, 18, threat_level);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COLLISION, buf, MACE_MSG_ID_COLLISION_MIN_LEN, MACE_MSG_ID_COLLISION_LEN, MACE_MSG_ID_COLLISION_CRC);
#else
    mace_collision_t packet;
    packet.id = id;
    packet.time_to_minimum_delta = time_to_minimum_delta;
    packet.altitude_minimum_delta = altitude_minimum_delta;
    packet.horizontal_minimum_delta = horizontal_minimum_delta;
    packet.src = src;
    packet.action = action;
    packet.threat_level = threat_level;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COLLISION, (const char *)&packet, MACE_MSG_ID_COLLISION_MIN_LEN, MACE_MSG_ID_COLLISION_LEN, MACE_MSG_ID_COLLISION_CRC);
#endif
}

/**
 * @brief Send a collision message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_collision_send_struct(mace_channel_t chan, const mace_collision_t* collision)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_collision_send(chan, collision->src, collision->id, collision->action, collision->threat_level, collision->time_to_minimum_delta, collision->altitude_minimum_delta, collision->horizontal_minimum_delta);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COLLISION, (const char *)collision, MACE_MSG_ID_COLLISION_MIN_LEN, MACE_MSG_ID_COLLISION_LEN, MACE_MSG_ID_COLLISION_CRC);
#endif
}

#if MACE_MSG_ID_COLLISION_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_collision_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint32_t(buf, 0, id);
    _mace_put_float(buf, 4, time_to_minimum_delta);
    _mace_put_float(buf, 8, altitude_minimum_delta);
    _mace_put_float(buf, 12, horizontal_minimum_delta);
    _mace_put_uint8_t(buf, 16, src);
    _mace_put_uint8_t(buf, 17, action);
    _mace_put_uint8_t(buf, 18, threat_level);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COLLISION, buf, MACE_MSG_ID_COLLISION_MIN_LEN, MACE_MSG_ID_COLLISION_LEN, MACE_MSG_ID_COLLISION_CRC);
#else
    mace_collision_t *packet = (mace_collision_t *)msgbuf;
    packet->id = id;
    packet->time_to_minimum_delta = time_to_minimum_delta;
    packet->altitude_minimum_delta = altitude_minimum_delta;
    packet->horizontal_minimum_delta = horizontal_minimum_delta;
    packet->src = src;
    packet->action = action;
    packet->threat_level = threat_level;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_COLLISION, (const char *)packet, MACE_MSG_ID_COLLISION_MIN_LEN, MACE_MSG_ID_COLLISION_LEN, MACE_MSG_ID_COLLISION_CRC);
#endif
}
#endif

#endif

// MESSAGE COLLISION UNPACKING


/**
 * @brief Get field src from collision message
 *
 * @return Collision data source
 */
static inline uint8_t mace_msg_collision_get_src(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field id from collision message
 *
 * @return Unique identifier, domain based on src field
 */
static inline uint32_t mace_msg_collision_get_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field action from collision message
 *
 * @return Action that is being taken to avoid this collision
 */
static inline uint8_t mace_msg_collision_get_action(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field threat_level from collision message
 *
 * @return How concerned the aircraft is about this collision
 */
static inline uint8_t mace_msg_collision_get_threat_level(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field time_to_minimum_delta from collision message
 *
 * @return Estimated time until collision occurs (seconds)
 */
static inline float mace_msg_collision_get_time_to_minimum_delta(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field altitude_minimum_delta from collision message
 *
 * @return Closest vertical distance in meters between vehicle and object
 */
static inline float mace_msg_collision_get_altitude_minimum_delta(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field horizontal_minimum_delta from collision message
 *
 * @return Closest horizontal distance in meteres between vehicle and object
 */
static inline float mace_msg_collision_get_horizontal_minimum_delta(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Decode a collision message into a struct
 *
 * @param msg The message to decode
 * @param collision C-struct to decode the message contents into
 */
static inline void mace_msg_collision_decode(const mace_message_t* msg, mace_collision_t* collision)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    collision->id = mace_msg_collision_get_id(msg);
    collision->time_to_minimum_delta = mace_msg_collision_get_time_to_minimum_delta(msg);
    collision->altitude_minimum_delta = mace_msg_collision_get_altitude_minimum_delta(msg);
    collision->horizontal_minimum_delta = mace_msg_collision_get_horizontal_minimum_delta(msg);
    collision->src = mace_msg_collision_get_src(msg);
    collision->action = mace_msg_collision_get_action(msg);
    collision->threat_level = mace_msg_collision_get_threat_level(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_COLLISION_LEN? msg->len : MACE_MSG_ID_COLLISION_LEN;
        memset(collision, 0, MACE_MSG_ID_COLLISION_LEN);
    memcpy(collision, _MACE_PAYLOAD(msg), len);
#endif
}
