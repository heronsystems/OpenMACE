#pragma once
// MESSAGE VEHICLE_ARMED PACKING

#define MACE_MSG_ID_VEHICLE_ARMED 2

MACEPACKED(
typedef struct __mace_vehicle_armed_t {
 uint8_t vehicle_armed; /*< Boolean describing whether(T=1) or not(F=0) the vehicle is armed.*/
}) mace_vehicle_armed_t;

#define MACE_MSG_ID_VEHICLE_ARMED_LEN 1
#define MACE_MSG_ID_VEHICLE_ARMED_MIN_LEN 1
#define MACE_MSG_ID_2_LEN 1
#define MACE_MSG_ID_2_MIN_LEN 1

#define MACE_MSG_ID_VEHICLE_ARMED_CRC 34
#define MACE_MSG_ID_2_CRC 34



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_VEHICLE_ARMED { \
    2, \
    "VEHICLE_ARMED", \
    1, \
    {  { "vehicle_armed", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_vehicle_armed_t, vehicle_armed) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_VEHICLE_ARMED { \
    "VEHICLE_ARMED", \
    1, \
    {  { "vehicle_armed", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_vehicle_armed_t, vehicle_armed) }, \
         } \
}
#endif

/**
 * @brief Pack a vehicle_armed message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vehicle_armed Boolean describing whether(T=1) or not(F=0) the vehicle is armed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_vehicle_armed_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t vehicle_armed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_VEHICLE_ARMED_LEN];
    _mace_put_uint8_t(buf, 0, vehicle_armed);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_VEHICLE_ARMED_LEN);
#else
    mace_vehicle_armed_t packet;
    packet.vehicle_armed = vehicle_armed;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_VEHICLE_ARMED_LEN);
#endif

    msg->msgid = MACE_MSG_ID_VEHICLE_ARMED;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_VEHICLE_ARMED_MIN_LEN, MACE_MSG_ID_VEHICLE_ARMED_LEN, MACE_MSG_ID_VEHICLE_ARMED_CRC);
}

/**
 * @brief Pack a vehicle_armed message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_armed Boolean describing whether(T=1) or not(F=0) the vehicle is armed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_vehicle_armed_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t vehicle_armed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_VEHICLE_ARMED_LEN];
    _mace_put_uint8_t(buf, 0, vehicle_armed);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_VEHICLE_ARMED_LEN);
#else
    mace_vehicle_armed_t packet;
    packet.vehicle_armed = vehicle_armed;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_VEHICLE_ARMED_LEN);
#endif

    msg->msgid = MACE_MSG_ID_VEHICLE_ARMED;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_VEHICLE_ARMED_MIN_LEN, MACE_MSG_ID_VEHICLE_ARMED_LEN, MACE_MSG_ID_VEHICLE_ARMED_CRC);
}

/**
 * @brief Encode a vehicle_armed struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_armed C-struct to read the message contents from
 */
static inline uint16_t mace_msg_vehicle_armed_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_vehicle_armed_t* vehicle_armed)
{
    return mace_msg_vehicle_armed_pack(system_id, component_id, msg, vehicle_armed->vehicle_armed);
}

/**
 * @brief Encode a vehicle_armed struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_armed C-struct to read the message contents from
 */
static inline uint16_t mace_msg_vehicle_armed_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_vehicle_armed_t* vehicle_armed)
{
    return mace_msg_vehicle_armed_pack_chan(system_id, component_id, chan, msg, vehicle_armed->vehicle_armed);
}

/**
 * @brief Send a vehicle_armed message
 * @param chan MAVLink channel to send the message
 *
 * @param vehicle_armed Boolean describing whether(T=1) or not(F=0) the vehicle is armed.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_vehicle_armed_send(mace_channel_t chan, uint8_t vehicle_armed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_VEHICLE_ARMED_LEN];
    _mace_put_uint8_t(buf, 0, vehicle_armed);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_ARMED, buf, MACE_MSG_ID_VEHICLE_ARMED_MIN_LEN, MACE_MSG_ID_VEHICLE_ARMED_LEN, MACE_MSG_ID_VEHICLE_ARMED_CRC);
#else
    mace_vehicle_armed_t packet;
    packet.vehicle_armed = vehicle_armed;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_ARMED, (const char *)&packet, MACE_MSG_ID_VEHICLE_ARMED_MIN_LEN, MACE_MSG_ID_VEHICLE_ARMED_LEN, MACE_MSG_ID_VEHICLE_ARMED_CRC);
#endif
}

/**
 * @brief Send a vehicle_armed message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_vehicle_armed_send_struct(mace_channel_t chan, const mace_vehicle_armed_t* vehicle_armed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_vehicle_armed_send(chan, vehicle_armed->vehicle_armed);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_ARMED, (const char *)vehicle_armed, MACE_MSG_ID_VEHICLE_ARMED_MIN_LEN, MACE_MSG_ID_VEHICLE_ARMED_LEN, MACE_MSG_ID_VEHICLE_ARMED_CRC);
#endif
}

#if MACE_MSG_ID_VEHICLE_ARMED_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_vehicle_armed_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t vehicle_armed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, vehicle_armed);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_ARMED, buf, MACE_MSG_ID_VEHICLE_ARMED_MIN_LEN, MACE_MSG_ID_VEHICLE_ARMED_LEN, MACE_MSG_ID_VEHICLE_ARMED_CRC);
#else
    mace_vehicle_armed_t *packet = (mace_vehicle_armed_t *)msgbuf;
    packet->vehicle_armed = vehicle_armed;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_ARMED, (const char *)packet, MACE_MSG_ID_VEHICLE_ARMED_MIN_LEN, MACE_MSG_ID_VEHICLE_ARMED_LEN, MACE_MSG_ID_VEHICLE_ARMED_CRC);
#endif
}
#endif

#endif

// MESSAGE VEHICLE_ARMED UNPACKING


/**
 * @brief Get field vehicle_armed from vehicle_armed message
 *
 * @return Boolean describing whether(T=1) or not(F=0) the vehicle is armed.
 */
static inline uint8_t mace_msg_vehicle_armed_get_vehicle_armed(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a vehicle_armed message into a struct
 *
 * @param msg The message to decode
 * @param vehicle_armed C-struct to decode the message contents into
 */
static inline void mace_msg_vehicle_armed_decode(const mace_message_t* msg, mace_vehicle_armed_t* vehicle_armed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    vehicle_armed->vehicle_armed = mace_msg_vehicle_armed_get_vehicle_armed(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_VEHICLE_ARMED_LEN? msg->len : MACE_MSG_ID_VEHICLE_ARMED_LEN;
        memset(vehicle_armed, 0, MACE_MSG_ID_VEHICLE_ARMED_LEN);
    memcpy(vehicle_armed, _MACE_PAYLOAD(msg), len);
#endif
}
