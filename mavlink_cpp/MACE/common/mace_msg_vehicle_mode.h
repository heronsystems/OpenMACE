#pragma once
// MESSAGE VEHICLE_MODE PACKING

#define MACE_MSG_ID_VEHICLE_MODE 1

MACEPACKED(
typedef struct __mace_vehicle_mode_t {
 char vehicle_mode[10]; /*< The flight mode in its string form.*/
}) mace_vehicle_mode_t;

#define MACE_MSG_ID_VEHICLE_MODE_LEN 10
#define MACE_MSG_ID_VEHICLE_MODE_MIN_LEN 10
#define MACE_MSG_ID_1_LEN 10
#define MACE_MSG_ID_1_MIN_LEN 10

#define MACE_MSG_ID_VEHICLE_MODE_CRC 204
#define MACE_MSG_ID_1_CRC 204

#define MACE_MSG_VEHICLE_MODE_FIELD_VEHICLE_MODE_LEN 10

#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_VEHICLE_MODE { \
    1, \
    "VEHICLE_MODE", \
    1, \
    {  { "vehicle_mode", NULL, MACE_TYPE_CHAR, 10, 0, offsetof(mace_vehicle_mode_t, vehicle_mode) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_VEHICLE_MODE { \
    "VEHICLE_MODE", \
    1, \
    {  { "vehicle_mode", NULL, MACE_TYPE_CHAR, 10, 0, offsetof(mace_vehicle_mode_t, vehicle_mode) }, \
         } \
}
#endif

/**
 * @brief Pack a vehicle_mode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vehicle_mode The flight mode in its string form.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_vehicle_mode_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               const char *vehicle_mode)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_VEHICLE_MODE_LEN];

    _mace_put_char_array(buf, 0, vehicle_mode, 10);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_VEHICLE_MODE_LEN);
#else
    mace_vehicle_mode_t packet;

    mace_array_memcpy(packet.vehicle_mode, vehicle_mode, sizeof(char)*10);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_VEHICLE_MODE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_VEHICLE_MODE;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_VEHICLE_MODE_MIN_LEN, MACE_MSG_ID_VEHICLE_MODE_LEN, MACE_MSG_ID_VEHICLE_MODE_CRC);
}

/**
 * @brief Pack a vehicle_mode message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_mode The flight mode in its string form.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_vehicle_mode_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   const char *vehicle_mode)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_VEHICLE_MODE_LEN];

    _mace_put_char_array(buf, 0, vehicle_mode, 10);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_VEHICLE_MODE_LEN);
#else
    mace_vehicle_mode_t packet;

    mace_array_memcpy(packet.vehicle_mode, vehicle_mode, sizeof(char)*10);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_VEHICLE_MODE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_VEHICLE_MODE;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_VEHICLE_MODE_MIN_LEN, MACE_MSG_ID_VEHICLE_MODE_LEN, MACE_MSG_ID_VEHICLE_MODE_CRC);
}

/**
 * @brief Encode a vehicle_mode struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_mode C-struct to read the message contents from
 */
static inline uint16_t mace_msg_vehicle_mode_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_vehicle_mode_t* vehicle_mode)
{
    return mace_msg_vehicle_mode_pack(system_id, component_id, msg, vehicle_mode->vehicle_mode);
}

/**
 * @brief Encode a vehicle_mode struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_mode C-struct to read the message contents from
 */
static inline uint16_t mace_msg_vehicle_mode_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_vehicle_mode_t* vehicle_mode)
{
    return mace_msg_vehicle_mode_pack_chan(system_id, component_id, chan, msg, vehicle_mode->vehicle_mode);
}

/**
 * @brief Send a vehicle_mode message
 * @param chan MAVLink channel to send the message
 *
 * @param vehicle_mode The flight mode in its string form.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_vehicle_mode_send(mace_channel_t chan, const char *vehicle_mode)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_VEHICLE_MODE_LEN];

    _mace_put_char_array(buf, 0, vehicle_mode, 10);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_MODE, buf, MACE_MSG_ID_VEHICLE_MODE_MIN_LEN, MACE_MSG_ID_VEHICLE_MODE_LEN, MACE_MSG_ID_VEHICLE_MODE_CRC);
#else
    mace_vehicle_mode_t packet;

    mace_array_memcpy(packet.vehicle_mode, vehicle_mode, sizeof(char)*10);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_MODE, (const char *)&packet, MACE_MSG_ID_VEHICLE_MODE_MIN_LEN, MACE_MSG_ID_VEHICLE_MODE_LEN, MACE_MSG_ID_VEHICLE_MODE_CRC);
#endif
}

/**
 * @brief Send a vehicle_mode message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_vehicle_mode_send_struct(mace_channel_t chan, const mace_vehicle_mode_t* vehicle_mode)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_vehicle_mode_send(chan, vehicle_mode->vehicle_mode);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_MODE, (const char *)vehicle_mode, MACE_MSG_ID_VEHICLE_MODE_MIN_LEN, MACE_MSG_ID_VEHICLE_MODE_LEN, MACE_MSG_ID_VEHICLE_MODE_CRC);
#endif
}

#if MACE_MSG_ID_VEHICLE_MODE_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_vehicle_mode_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  const char *vehicle_mode)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mace_put_char_array(buf, 0, vehicle_mode, 10);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_MODE, buf, MACE_MSG_ID_VEHICLE_MODE_MIN_LEN, MACE_MSG_ID_VEHICLE_MODE_LEN, MACE_MSG_ID_VEHICLE_MODE_CRC);
#else
    mace_vehicle_mode_t *packet = (mace_vehicle_mode_t *)msgbuf;

    mace_array_memcpy(packet->vehicle_mode, vehicle_mode, sizeof(char)*10);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_VEHICLE_MODE, (const char *)packet, MACE_MSG_ID_VEHICLE_MODE_MIN_LEN, MACE_MSG_ID_VEHICLE_MODE_LEN, MACE_MSG_ID_VEHICLE_MODE_CRC);
#endif
}
#endif

#endif

// MESSAGE VEHICLE_MODE UNPACKING


/**
 * @brief Get field vehicle_mode from vehicle_mode message
 *
 * @return The flight mode in its string form.
 */
static inline uint16_t mace_msg_vehicle_mode_get_vehicle_mode(const mace_message_t* msg, char *vehicle_mode)
{
    return _MACE_RETURN_char_array(msg, vehicle_mode, 10,  0);
}

/**
 * @brief Decode a vehicle_mode message into a struct
 *
 * @param msg The message to decode
 * @param vehicle_mode C-struct to decode the message contents into
 */
static inline void mace_msg_vehicle_mode_decode(const mace_message_t* msg, mace_vehicle_mode_t* vehicle_mode)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_vehicle_mode_get_vehicle_mode(msg, vehicle_mode->vehicle_mode);
#else
        uint8_t len = msg->len < MACE_MSG_ID_VEHICLE_MODE_LEN? msg->len : MACE_MSG_ID_VEHICLE_MODE_LEN;
        memset(vehicle_mode, 0, MACE_MSG_ID_VEHICLE_MODE_LEN);
    memcpy(vehicle_mode, _MACE_PAYLOAD(msg), len);
#endif
}
