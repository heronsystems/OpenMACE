#pragma once
// MESSAGE ROI_AG PACKING

#define MACE_MSG_ID_ROI_AG 51

MACEPACKED(
typedef struct __mace_roi_ag_t {
 float stress_value; /*< Stress value*/
 float x; /*< PARAM5 / local: x position, global: latitude*/
 float y; /*< PARAM6 / y position: global: longitude*/
 float z; /*< PARAM7 / z position: global: altitude (relative or absolute, depending on frame.*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t point_discovery; /*< See POINT_DISCOVERY enum*/
 uint8_t stress_threshold; /*< See STRESS_THRESHOLD enum*/
 uint8_t frame; /*< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h*/
}) mace_roi_ag_t;

#define MACE_MSG_ID_ROI_AG_LEN 21
#define MACE_MSG_ID_ROI_AG_MIN_LEN 21
#define MACE_MSG_ID_51_LEN 21
#define MACE_MSG_ID_51_MIN_LEN 21

#define MACE_MSG_ID_ROI_AG_CRC 189
#define MACE_MSG_ID_51_CRC 189



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_ROI_AG { \
    51, \
    "ROI_AG", \
    9, \
    {  { "stress_value", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_roi_ag_t, stress_value) }, \
         { "x", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_roi_ag_t, x) }, \
         { "y", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_roi_ag_t, y) }, \
         { "z", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_roi_ag_t, z) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 16, offsetof(mace_roi_ag_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 17, offsetof(mace_roi_ag_t, target_component) }, \
         { "point_discovery", NULL, MACE_TYPE_UINT8_T, 0, 18, offsetof(mace_roi_ag_t, point_discovery) }, \
         { "stress_threshold", NULL, MACE_TYPE_UINT8_T, 0, 19, offsetof(mace_roi_ag_t, stress_threshold) }, \
         { "frame", NULL, MACE_TYPE_UINT8_T, 0, 20, offsetof(mace_roi_ag_t, frame) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_ROI_AG { \
    "ROI_AG", \
    9, \
    {  { "stress_value", NULL, MACE_TYPE_FLOAT, 0, 0, offsetof(mace_roi_ag_t, stress_value) }, \
         { "x", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_roi_ag_t, x) }, \
         { "y", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_roi_ag_t, y) }, \
         { "z", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_roi_ag_t, z) }, \
         { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 16, offsetof(mace_roi_ag_t, target_system) }, \
         { "target_component", NULL, MACE_TYPE_UINT8_T, 0, 17, offsetof(mace_roi_ag_t, target_component) }, \
         { "point_discovery", NULL, MACE_TYPE_UINT8_T, 0, 18, offsetof(mace_roi_ag_t, point_discovery) }, \
         { "stress_threshold", NULL, MACE_TYPE_UINT8_T, 0, 19, offsetof(mace_roi_ag_t, stress_threshold) }, \
         { "frame", NULL, MACE_TYPE_UINT8_T, 0, 20, offsetof(mace_roi_ag_t, frame) }, \
         } \
}
#endif

/**
 * @brief Pack a roi_ag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param point_discovery See POINT_DISCOVERY enum
 * @param stress_threshold See STRESS_THRESHOLD enum
 * @param stress_value Stress value
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_roi_ag_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t point_discovery, uint8_t stress_threshold, float stress_value, uint8_t frame, float x, float y, float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ROI_AG_LEN];
    _mace_put_float(buf, 0, stress_value);
    _mace_put_float(buf, 4, x);
    _mace_put_float(buf, 8, y);
    _mace_put_float(buf, 12, z);
    _mace_put_uint8_t(buf, 16, target_system);
    _mace_put_uint8_t(buf, 17, target_component);
    _mace_put_uint8_t(buf, 18, point_discovery);
    _mace_put_uint8_t(buf, 19, stress_threshold);
    _mace_put_uint8_t(buf, 20, frame);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_ROI_AG_LEN);
#else
    mace_roi_ag_t packet;
    packet.stress_value = stress_value;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.point_discovery = point_discovery;
    packet.stress_threshold = stress_threshold;
    packet.frame = frame;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_ROI_AG_LEN);
#endif

    msg->msgid = MACE_MSG_ID_ROI_AG;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_ROI_AG_MIN_LEN, MACE_MSG_ID_ROI_AG_LEN, MACE_MSG_ID_ROI_AG_CRC);
}

/**
 * @brief Pack a roi_ag message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param point_discovery See POINT_DISCOVERY enum
 * @param stress_threshold See STRESS_THRESHOLD enum
 * @param stress_value Stress value
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_roi_ag_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t point_discovery,uint8_t stress_threshold,float stress_value,uint8_t frame,float x,float y,float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ROI_AG_LEN];
    _mace_put_float(buf, 0, stress_value);
    _mace_put_float(buf, 4, x);
    _mace_put_float(buf, 8, y);
    _mace_put_float(buf, 12, z);
    _mace_put_uint8_t(buf, 16, target_system);
    _mace_put_uint8_t(buf, 17, target_component);
    _mace_put_uint8_t(buf, 18, point_discovery);
    _mace_put_uint8_t(buf, 19, stress_threshold);
    _mace_put_uint8_t(buf, 20, frame);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_ROI_AG_LEN);
#else
    mace_roi_ag_t packet;
    packet.stress_value = stress_value;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.point_discovery = point_discovery;
    packet.stress_threshold = stress_threshold;
    packet.frame = frame;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_ROI_AG_LEN);
#endif

    msg->msgid = MACE_MSG_ID_ROI_AG;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_ROI_AG_MIN_LEN, MACE_MSG_ID_ROI_AG_LEN, MACE_MSG_ID_ROI_AG_CRC);
}

/**
 * @brief Encode a roi_ag struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param roi_ag C-struct to read the message contents from
 */
static inline uint16_t mace_msg_roi_ag_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_roi_ag_t* roi_ag)
{
    return mace_msg_roi_ag_pack(system_id, component_id, msg, roi_ag->target_system, roi_ag->target_component, roi_ag->point_discovery, roi_ag->stress_threshold, roi_ag->stress_value, roi_ag->frame, roi_ag->x, roi_ag->y, roi_ag->z);
}

/**
 * @brief Encode a roi_ag struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roi_ag C-struct to read the message contents from
 */
static inline uint16_t mace_msg_roi_ag_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_roi_ag_t* roi_ag)
{
    return mace_msg_roi_ag_pack_chan(system_id, component_id, chan, msg, roi_ag->target_system, roi_ag->target_component, roi_ag->point_discovery, roi_ag->stress_threshold, roi_ag->stress_value, roi_ag->frame, roi_ag->x, roi_ag->y, roi_ag->z);
}

/**
 * @brief Send a roi_ag message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param point_discovery See POINT_DISCOVERY enum
 * @param stress_threshold See STRESS_THRESHOLD enum
 * @param stress_value Stress value
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_roi_ag_send(mace_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t point_discovery, uint8_t stress_threshold, float stress_value, uint8_t frame, float x, float y, float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_ROI_AG_LEN];
    _mace_put_float(buf, 0, stress_value);
    _mace_put_float(buf, 4, x);
    _mace_put_float(buf, 8, y);
    _mace_put_float(buf, 12, z);
    _mace_put_uint8_t(buf, 16, target_system);
    _mace_put_uint8_t(buf, 17, target_component);
    _mace_put_uint8_t(buf, 18, point_discovery);
    _mace_put_uint8_t(buf, 19, stress_threshold);
    _mace_put_uint8_t(buf, 20, frame);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ROI_AG, buf, MACE_MSG_ID_ROI_AG_MIN_LEN, MACE_MSG_ID_ROI_AG_LEN, MACE_MSG_ID_ROI_AG_CRC);
#else
    mace_roi_ag_t packet;
    packet.stress_value = stress_value;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.point_discovery = point_discovery;
    packet.stress_threshold = stress_threshold;
    packet.frame = frame;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ROI_AG, (const char *)&packet, MACE_MSG_ID_ROI_AG_MIN_LEN, MACE_MSG_ID_ROI_AG_LEN, MACE_MSG_ID_ROI_AG_CRC);
#endif
}

/**
 * @brief Send a roi_ag message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_roi_ag_send_struct(mace_channel_t chan, const mace_roi_ag_t* roi_ag)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_roi_ag_send(chan, roi_ag->target_system, roi_ag->target_component, roi_ag->point_discovery, roi_ag->stress_threshold, roi_ag->stress_value, roi_ag->frame, roi_ag->x, roi_ag->y, roi_ag->z);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ROI_AG, (const char *)roi_ag, MACE_MSG_ID_ROI_AG_MIN_LEN, MACE_MSG_ID_ROI_AG_LEN, MACE_MSG_ID_ROI_AG_CRC);
#endif
}

#if MACE_MSG_ID_ROI_AG_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_roi_ag_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t point_discovery, uint8_t stress_threshold, float stress_value, uint8_t frame, float x, float y, float z)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_float(buf, 0, stress_value);
    _mace_put_float(buf, 4, x);
    _mace_put_float(buf, 8, y);
    _mace_put_float(buf, 12, z);
    _mace_put_uint8_t(buf, 16, target_system);
    _mace_put_uint8_t(buf, 17, target_component);
    _mace_put_uint8_t(buf, 18, point_discovery);
    _mace_put_uint8_t(buf, 19, stress_threshold);
    _mace_put_uint8_t(buf, 20, frame);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ROI_AG, buf, MACE_MSG_ID_ROI_AG_MIN_LEN, MACE_MSG_ID_ROI_AG_LEN, MACE_MSG_ID_ROI_AG_CRC);
#else
    mace_roi_ag_t *packet = (mace_roi_ag_t *)msgbuf;
    packet->stress_value = stress_value;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->point_discovery = point_discovery;
    packet->stress_threshold = stress_threshold;
    packet->frame = frame;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_ROI_AG, (const char *)packet, MACE_MSG_ID_ROI_AG_MIN_LEN, MACE_MSG_ID_ROI_AG_LEN, MACE_MSG_ID_ROI_AG_CRC);
#endif
}
#endif

#endif

// MESSAGE ROI_AG UNPACKING


/**
 * @brief Get field target_system from roi_ag message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_roi_ag_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from roi_ag message
 *
 * @return Component ID
 */
static inline uint8_t mace_msg_roi_ag_get_target_component(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field point_discovery from roi_ag message
 *
 * @return See POINT_DISCOVERY enum
 */
static inline uint8_t mace_msg_roi_ag_get_point_discovery(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field stress_threshold from roi_ag message
 *
 * @return See STRESS_THRESHOLD enum
 */
static inline uint8_t mace_msg_roi_ag_get_stress_threshold(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field stress_value from roi_ag message
 *
 * @return Stress value
 */
static inline float mace_msg_roi_ag_get_stress_value(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  0);
}

/**
 * @brief Get field frame from roi_ag message
 *
 * @return The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 */
static inline uint8_t mace_msg_roi_ag_get_frame(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field x from roi_ag message
 *
 * @return PARAM5 / local: x position, global: latitude
 */
static inline float mace_msg_roi_ag_get_x(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from roi_ag message
 *
 * @return PARAM6 / y position: global: longitude
 */
static inline float mace_msg_roi_ag_get_y(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from roi_ag message
 *
 * @return PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
 */
static inline float mace_msg_roi_ag_get_z(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Decode a roi_ag message into a struct
 *
 * @param msg The message to decode
 * @param roi_ag C-struct to decode the message contents into
 */
static inline void mace_msg_roi_ag_decode(const mace_message_t* msg, mace_roi_ag_t* roi_ag)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    roi_ag->stress_value = mace_msg_roi_ag_get_stress_value(msg);
    roi_ag->x = mace_msg_roi_ag_get_x(msg);
    roi_ag->y = mace_msg_roi_ag_get_y(msg);
    roi_ag->z = mace_msg_roi_ag_get_z(msg);
    roi_ag->target_system = mace_msg_roi_ag_get_target_system(msg);
    roi_ag->target_component = mace_msg_roi_ag_get_target_component(msg);
    roi_ag->point_discovery = mace_msg_roi_ag_get_point_discovery(msg);
    roi_ag->stress_threshold = mace_msg_roi_ag_get_stress_threshold(msg);
    roi_ag->frame = mace_msg_roi_ag_get_frame(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_ROI_AG_LEN? msg->len : MACE_MSG_ID_ROI_AG_LEN;
        memset(roi_ag, 0, MACE_MSG_ID_ROI_AG_LEN);
    memcpy(roi_ag, _MACE_PAYLOAD(msg), len);
#endif
}
