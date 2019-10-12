#pragma once
// MESSAGE POWER_STATUS PACKING

#define MACE_MSG_ID_POWER_STATUS 202

MACEPACKED(
typedef struct __mace_power_status_t {
 uint16_t Vcc; /*< 5V rail voltage in millivolts*/
 uint16_t Vservo; /*< servo rail voltage in millivolts*/
 uint16_t flags; /*< power supply status flags (see UXV_POWER_STATUS enum)*/
}) mace_power_status_t;

#define MACE_MSG_ID_POWER_STATUS_LEN 6
#define MACE_MSG_ID_POWER_STATUS_MIN_LEN 6
#define MACE_MSG_ID_202_LEN 6
#define MACE_MSG_ID_202_MIN_LEN 6

#define MACE_MSG_ID_POWER_STATUS_CRC 203
#define MACE_MSG_ID_202_CRC 203



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_POWER_STATUS { \
    202, \
    "POWER_STATUS", \
    3, \
    {  { "Vcc", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_power_status_t, Vcc) }, \
         { "Vservo", NULL, MACE_TYPE_UINT16_T, 0, 2, offsetof(mace_power_status_t, Vservo) }, \
         { "flags", NULL, MACE_TYPE_UINT16_T, 0, 4, offsetof(mace_power_status_t, flags) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_POWER_STATUS { \
    "POWER_STATUS", \
    3, \
    {  { "Vcc", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_power_status_t, Vcc) }, \
         { "Vservo", NULL, MACE_TYPE_UINT16_T, 0, 2, offsetof(mace_power_status_t, Vservo) }, \
         { "flags", NULL, MACE_TYPE_UINT16_T, 0, 4, offsetof(mace_power_status_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a power_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param Vcc 5V rail voltage in millivolts
 * @param Vservo servo rail voltage in millivolts
 * @param flags power supply status flags (see UXV_POWER_STATUS enum)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_power_status_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint16_t Vcc, uint16_t Vservo, uint16_t flags)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_POWER_STATUS_LEN];
    _mace_put_uint16_t(buf, 0, Vcc);
    _mace_put_uint16_t(buf, 2, Vservo);
    _mace_put_uint16_t(buf, 4, flags);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_POWER_STATUS_LEN);
#else
    mace_power_status_t packet;
    packet.Vcc = Vcc;
    packet.Vservo = Vservo;
    packet.flags = flags;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_POWER_STATUS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_POWER_STATUS;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_POWER_STATUS_MIN_LEN, MACE_MSG_ID_POWER_STATUS_LEN, MACE_MSG_ID_POWER_STATUS_CRC);
}

/**
 * @brief Pack a power_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param Vcc 5V rail voltage in millivolts
 * @param Vservo servo rail voltage in millivolts
 * @param flags power supply status flags (see UXV_POWER_STATUS enum)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_power_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint16_t Vcc,uint16_t Vservo,uint16_t flags)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_POWER_STATUS_LEN];
    _mace_put_uint16_t(buf, 0, Vcc);
    _mace_put_uint16_t(buf, 2, Vservo);
    _mace_put_uint16_t(buf, 4, flags);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_POWER_STATUS_LEN);
#else
    mace_power_status_t packet;
    packet.Vcc = Vcc;
    packet.Vservo = Vservo;
    packet.flags = flags;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_POWER_STATUS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_POWER_STATUS;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_POWER_STATUS_MIN_LEN, MACE_MSG_ID_POWER_STATUS_LEN, MACE_MSG_ID_POWER_STATUS_CRC);
}

/**
 * @brief Encode a power_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param power_status C-struct to read the message contents from
 */
static inline uint16_t mace_msg_power_status_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_power_status_t* power_status)
{
    return mace_msg_power_status_pack(system_id, component_id, msg, power_status->Vcc, power_status->Vservo, power_status->flags);
}

/**
 * @brief Encode a power_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param power_status C-struct to read the message contents from
 */
static inline uint16_t mace_msg_power_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_power_status_t* power_status)
{
    return mace_msg_power_status_pack_chan(system_id, component_id, chan, msg, power_status->Vcc, power_status->Vservo, power_status->flags);
}

/**
 * @brief Send a power_status message
 * @param chan MAVLink channel to send the message
 *
 * @param Vcc 5V rail voltage in millivolts
 * @param Vservo servo rail voltage in millivolts
 * @param flags power supply status flags (see UXV_POWER_STATUS enum)
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_power_status_send(mace_channel_t chan, uint16_t Vcc, uint16_t Vservo, uint16_t flags)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_POWER_STATUS_LEN];
    _mace_put_uint16_t(buf, 0, Vcc);
    _mace_put_uint16_t(buf, 2, Vservo);
    _mace_put_uint16_t(buf, 4, flags);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_POWER_STATUS, buf, MACE_MSG_ID_POWER_STATUS_MIN_LEN, MACE_MSG_ID_POWER_STATUS_LEN, MACE_MSG_ID_POWER_STATUS_CRC);
#else
    mace_power_status_t packet;
    packet.Vcc = Vcc;
    packet.Vservo = Vservo;
    packet.flags = flags;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_POWER_STATUS, (const char *)&packet, MACE_MSG_ID_POWER_STATUS_MIN_LEN, MACE_MSG_ID_POWER_STATUS_LEN, MACE_MSG_ID_POWER_STATUS_CRC);
#endif
}

/**
 * @brief Send a power_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_power_status_send_struct(mace_channel_t chan, const mace_power_status_t* power_status)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_power_status_send(chan, power_status->Vcc, power_status->Vservo, power_status->flags);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_POWER_STATUS, (const char *)power_status, MACE_MSG_ID_POWER_STATUS_MIN_LEN, MACE_MSG_ID_POWER_STATUS_LEN, MACE_MSG_ID_POWER_STATUS_CRC);
#endif
}

#if MACE_MSG_ID_POWER_STATUS_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_power_status_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint16_t Vcc, uint16_t Vservo, uint16_t flags)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint16_t(buf, 0, Vcc);
    _mace_put_uint16_t(buf, 2, Vservo);
    _mace_put_uint16_t(buf, 4, flags);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_POWER_STATUS, buf, MACE_MSG_ID_POWER_STATUS_MIN_LEN, MACE_MSG_ID_POWER_STATUS_LEN, MACE_MSG_ID_POWER_STATUS_CRC);
#else
    mace_power_status_t *packet = (mace_power_status_t *)msgbuf;
    packet->Vcc = Vcc;
    packet->Vservo = Vservo;
    packet->flags = flags;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_POWER_STATUS, (const char *)packet, MACE_MSG_ID_POWER_STATUS_MIN_LEN, MACE_MSG_ID_POWER_STATUS_LEN, MACE_MSG_ID_POWER_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE POWER_STATUS UNPACKING


/**
 * @brief Get field Vcc from power_status message
 *
 * @return 5V rail voltage in millivolts
 */
static inline uint16_t mace_msg_power_status_get_Vcc(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field Vservo from power_status message
 *
 * @return servo rail voltage in millivolts
 */
static inline uint16_t mace_msg_power_status_get_Vservo(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field flags from power_status message
 *
 * @return power supply status flags (see UXV_POWER_STATUS enum)
 */
static inline uint16_t mace_msg_power_status_get_flags(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a power_status message into a struct
 *
 * @param msg The message to decode
 * @param power_status C-struct to decode the message contents into
 */
static inline void mace_msg_power_status_decode(const mace_message_t* msg, mace_power_status_t* power_status)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    power_status->Vcc = mace_msg_power_status_get_Vcc(msg);
    power_status->Vservo = mace_msg_power_status_get_Vservo(msg);
    power_status->flags = mace_msg_power_status_get_flags(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_POWER_STATUS_LEN? msg->len : MACE_MSG_ID_POWER_STATUS_LEN;
        memset(power_status, 0, MACE_MSG_ID_POWER_STATUS_LEN);
    memcpy(power_status, _MACE_PAYLOAD(msg), len);
#endif
}
