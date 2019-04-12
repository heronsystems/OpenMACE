#pragma once
// MESSAGE VEHICLE_SYNC PACKING

#define MACE_MSG_ID_VEHICLE_SYNC 50

MACEPACKED(
typedef struct __mace_vehicle_sync_t {
 uint8_t target_system; /*< System ID*/
}) mace_vehicle_sync_t;

#define MACE_MSG_ID_VEHICLE_SYNC_LEN 1
#define MACE_MSG_ID_VEHICLE_SYNC_MIN_LEN 1
#define MACE_MSG_ID_50_LEN 1
#define MACE_MSG_ID_50_MIN_LEN 1

#define MACE_MSG_ID_VEHICLE_SYNC_CRC 178
#define MACE_MSG_ID_50_CRC 178



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_VEHICLE_SYNC { \
    50, \
    "VEHICLE_SYNC", \
    1, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_vehicle_sync_t, target_system) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_VEHICLE_SYNC { \
    "VEHICLE_SYNC", \
    1, \
    {  { "target_system", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_vehicle_sync_t, target_system) }, \
         } \
}
#endif

/**
 * @brief Pack a vehicle_sync message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_vehicle_sync_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t target_system)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_VEHICLE_SYNC_LEN];
    _mace_put_uint8_t(buf, 0, target_system);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_VEHICLE_SYNC_LEN);
#else
    mace_vehicle_sync_t packet;
    packet.target_system = target_system;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_VEHICLE_SYNC_LEN);
#endif

    msg->msgid = MACE_MSG_ID_VEHICLE_SYNC;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_VEHICLE_SYNC_MIN_LEN, MACE_MSG_ID_VEHICLE_SYNC_LEN, MACE_MSG_ID_VEHICLE_SYNC_CRC);
}

/**
 * @brief Pack a vehicle_sync message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_vehicle_sync_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t target_system)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_VEHICLE_SYNC_LEN];
    _mace_put_uint8_t(buf, 0, target_system);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_VEHICLE_SYNC_LEN);
#else
    mace_vehicle_sync_t packet;
    packet.target_system = target_system;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_VEHICLE_SYNC_LEN);
#endif

    msg->msgid = MACE_MSG_ID_VEHICLE_SYNC;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_VEHICLE_SYNC_MIN_LEN, MACE_MSG_ID_VEHICLE_SYNC_LEN, MACE_MSG_ID_VEHICLE_SYNC_CRC);
}

/**
 * @brief Encode a vehicle_sync struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_sync C-struct to read the message contents from
 */
static inline uint16_t mace_msg_vehicle_sync_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_vehicle_sync_t* vehicle_sync)
{
    return mace_msg_vehicle_sync_pack(system_id, component_id, msg, vehicle_sync->target_system);
}

/**
 * @brief Encode a vehicle_sync struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_sync C-struct to read the message contents from
 */
static inline uint16_t mace_msg_vehicle_sync_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_vehicle_sync_t* vehicle_sync)
{
    return mace_msg_vehicle_sync_pack_chan(system_id, component_id, chan, msg, vehicle_sync->target_system);
}

/**
 * @brief Send a vehicle_sync message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_vehicle_sync_send(mace_channel_t chan, uint8_t target_system)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_VEHICLE_SYNC_LEN];
    _mace_put_uint8_t(buf, 0, target_system);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_SYNC, buf, MACE_MSG_ID_VEHICLE_SYNC_MIN_LEN, MACE_MSG_ID_VEHICLE_SYNC_LEN, MACE_MSG_ID_VEHICLE_SYNC_CRC);
#else
    mace_vehicle_sync_t packet;
    packet.target_system = target_system;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_SYNC, (const char *)&packet, MACE_MSG_ID_VEHICLE_SYNC_MIN_LEN, MACE_MSG_ID_VEHICLE_SYNC_LEN, MACE_MSG_ID_VEHICLE_SYNC_CRC);
#endif
}

/**
 * @brief Send a vehicle_sync message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_vehicle_sync_send_struct(mace_channel_t chan, const mace_vehicle_sync_t* vehicle_sync)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_vehicle_sync_send(chan, vehicle_sync->target_system);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_SYNC, (const char *)vehicle_sync, MACE_MSG_ID_VEHICLE_SYNC_MIN_LEN, MACE_MSG_ID_VEHICLE_SYNC_LEN, MACE_MSG_ID_VEHICLE_SYNC_CRC);
#endif
}

#if MACE_MSG_ID_VEHICLE_SYNC_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_vehicle_sync_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t target_system)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, target_system);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_SYNC, buf, MACE_MSG_ID_VEHICLE_SYNC_MIN_LEN, MACE_MSG_ID_VEHICLE_SYNC_LEN, MACE_MSG_ID_VEHICLE_SYNC_CRC);
#else
    mace_vehicle_sync_t *packet = (mace_vehicle_sync_t *)msgbuf;
    packet->target_system = target_system;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_SYNC, (const char *)packet, MACE_MSG_ID_VEHICLE_SYNC_MIN_LEN, MACE_MSG_ID_VEHICLE_SYNC_LEN, MACE_MSG_ID_VEHICLE_SYNC_CRC);
#endif
}
#endif

#endif

// MESSAGE VEHICLE_SYNC UNPACKING


/**
 * @brief Get field target_system from vehicle_sync message
 *
 * @return System ID
 */
static inline uint8_t mace_msg_vehicle_sync_get_target_system(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a vehicle_sync message into a struct
 *
 * @param msg The message to decode
 * @param vehicle_sync C-struct to decode the message contents into
 */
static inline void mace_msg_vehicle_sync_decode(const mace_message_t* msg, mace_vehicle_sync_t* vehicle_sync)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    vehicle_sync->target_system = mace_msg_vehicle_sync_get_target_system(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_VEHICLE_SYNC_LEN? msg->len : MACE_MSG_ID_VEHICLE_SYNC_LEN;
        memset(vehicle_sync, 0, MACE_MSG_ID_VEHICLE_SYNC_LEN);
    memcpy(vehicle_sync, _MACE_PAYLOAD(msg), len);
#endif
}
