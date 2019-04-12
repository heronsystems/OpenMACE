#pragma once
// MESSAGE STATUSTEXT PACKING

#define MACE_MSG_ID_STATUSTEXT 253

MACEPACKED(
typedef struct __mace_statustext_t {
 uint8_t severity; /*< Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.*/
 char text[50]; /*< Status text message, without null termination character*/
}) mace_statustext_t;

#define MACE_MSG_ID_STATUSTEXT_LEN 51
#define MACE_MSG_ID_STATUSTEXT_MIN_LEN 51
#define MACE_MSG_ID_253_LEN 51
#define MACE_MSG_ID_253_MIN_LEN 51

#define MACE_MSG_ID_STATUSTEXT_CRC 83
#define MACE_MSG_ID_253_CRC 83

#define MACE_MSG_STATUSTEXT_FIELD_TEXT_LEN 50

#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_STATUSTEXT { \
    253, \
    "STATUSTEXT", \
    2, \
    {  { "severity", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_statustext_t, severity) }, \
         { "text", NULL, MACE_TYPE_CHAR, 50, 1, offsetof(mace_statustext_t, text) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_STATUSTEXT { \
    "STATUSTEXT", \
    2, \
    {  { "severity", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_statustext_t, severity) }, \
         { "text", NULL, MACE_TYPE_CHAR, 50, 1, offsetof(mace_statustext_t, text) }, \
         } \
}
#endif

/**
 * @brief Pack a statustext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param severity Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 * @param text Status text message, without null termination character
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_statustext_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t severity, const char *text)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_STATUSTEXT_LEN];
    _mace_put_uint8_t(buf, 0, severity);
    _mace_put_char_array(buf, 1, text, 50);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_STATUSTEXT_LEN);
#else
    mace_statustext_t packet;
    packet.severity = severity;
    mace_array_memcpy(packet.text, text, sizeof(char)*50);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_STATUSTEXT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_STATUSTEXT;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_STATUSTEXT_MIN_LEN, MACE_MSG_ID_STATUSTEXT_LEN, MACE_MSG_ID_STATUSTEXT_CRC);
}

/**
 * @brief Pack a statustext message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param severity Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 * @param text Status text message, without null termination character
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_statustext_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t severity,const char *text)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_STATUSTEXT_LEN];
    _mace_put_uint8_t(buf, 0, severity);
    _mace_put_char_array(buf, 1, text, 50);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_STATUSTEXT_LEN);
#else
    mace_statustext_t packet;
    packet.severity = severity;
    mace_array_memcpy(packet.text, text, sizeof(char)*50);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_STATUSTEXT_LEN);
#endif

    msg->msgid = MACE_MSG_ID_STATUSTEXT;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_STATUSTEXT_MIN_LEN, MACE_MSG_ID_STATUSTEXT_LEN, MACE_MSG_ID_STATUSTEXT_CRC);
}

/**
 * @brief Encode a statustext struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param statustext C-struct to read the message contents from
 */
static inline uint16_t mace_msg_statustext_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_statustext_t* statustext)
{
    return mace_msg_statustext_pack(system_id, component_id, msg, statustext->severity, statustext->text);
}

/**
 * @brief Encode a statustext struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param statustext C-struct to read the message contents from
 */
static inline uint16_t mace_msg_statustext_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_statustext_t* statustext)
{
    return mace_msg_statustext_pack_chan(system_id, component_id, chan, msg, statustext->severity, statustext->text);
}

/**
 * @brief Send a statustext message
 * @param chan MAVLink channel to send the message
 *
 * @param severity Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 * @param text Status text message, without null termination character
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_statustext_send(mace_channel_t chan, uint8_t severity, const char *text)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_STATUSTEXT_LEN];
    _mace_put_uint8_t(buf, 0, severity);
    _mace_put_char_array(buf, 1, text, 50);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_STATUSTEXT, buf, MACE_MSG_ID_STATUSTEXT_MIN_LEN, MACE_MSG_ID_STATUSTEXT_LEN, MACE_MSG_ID_STATUSTEXT_CRC);
#else
    mace_statustext_t packet;
    packet.severity = severity;
    mace_array_memcpy(packet.text, text, sizeof(char)*50);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_STATUSTEXT, (const char *)&packet, MACE_MSG_ID_STATUSTEXT_MIN_LEN, MACE_MSG_ID_STATUSTEXT_LEN, MACE_MSG_ID_STATUSTEXT_CRC);
#endif
}

/**
 * @brief Send a statustext message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_statustext_send_struct(mace_channel_t chan, const mace_statustext_t* statustext)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_statustext_send(chan, statustext->severity, statustext->text);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_STATUSTEXT, (const char *)statustext, MACE_MSG_ID_STATUSTEXT_MIN_LEN, MACE_MSG_ID_STATUSTEXT_LEN, MACE_MSG_ID_STATUSTEXT_CRC);
#endif
}

#if MACE_MSG_ID_STATUSTEXT_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_statustext_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t severity, const char *text)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, severity);
    _mace_put_char_array(buf, 1, text, 50);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_STATUSTEXT, buf, MACE_MSG_ID_STATUSTEXT_MIN_LEN, MACE_MSG_ID_STATUSTEXT_LEN, MACE_MSG_ID_STATUSTEXT_CRC);
#else
    mace_statustext_t *packet = (mace_statustext_t *)msgbuf;
    packet->severity = severity;
    mace_array_memcpy(packet->text, text, sizeof(char)*50);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_STATUSTEXT, (const char *)packet, MACE_MSG_ID_STATUSTEXT_MIN_LEN, MACE_MSG_ID_STATUSTEXT_LEN, MACE_MSG_ID_STATUSTEXT_CRC);
#endif
}
#endif

#endif

// MESSAGE STATUSTEXT UNPACKING


/**
 * @brief Get field severity from statustext message
 *
 * @return Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 */
static inline uint8_t mace_msg_statustext_get_severity(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field text from statustext message
 *
 * @return Status text message, without null termination character
 */
static inline uint16_t mace_msg_statustext_get_text(const mace_message_t* msg, char *text)
{
    return _MACE_RETURN_char_array(msg, text, 50,  1);
}

/**
 * @brief Decode a statustext message into a struct
 *
 * @param msg The message to decode
 * @param statustext C-struct to decode the message contents into
 */
static inline void mace_msg_statustext_decode(const mace_message_t* msg, mace_statustext_t* statustext)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    statustext->severity = mace_msg_statustext_get_severity(msg);
    mace_msg_statustext_get_text(msg, statustext->text);
#else
        uint8_t len = msg->len < MACE_MSG_ID_STATUSTEXT_LEN? msg->len : MACE_MSG_ID_STATUSTEXT_LEN;
        memset(statustext, 0, MACE_MSG_ID_STATUSTEXT_LEN);
    memcpy(statustext, _MACE_PAYLOAD(msg), len);
#endif
}
