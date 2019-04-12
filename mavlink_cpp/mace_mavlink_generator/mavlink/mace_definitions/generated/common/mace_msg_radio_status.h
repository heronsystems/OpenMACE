#pragma once
// MESSAGE RADIO_STATUS PACKING

#define MACE_MSG_ID_RADIO_STATUS 200

MACEPACKED(
typedef struct __mace_radio_status_t {
 uint16_t rxerrors; /*< Receive errors*/
 uint16_t fixed; /*< Count of error corrected packets*/
 uint8_t rssi; /*< Local signal strength*/
 uint8_t remrssi; /*< Remote signal strength*/
 uint8_t txbuf; /*< Remaining free buffer space in percent.*/
 uint8_t noise; /*< Background noise level*/
 uint8_t remnoise; /*< Remote background noise level*/
}) mace_radio_status_t;

#define MACE_MSG_ID_RADIO_STATUS_LEN 9
#define MACE_MSG_ID_RADIO_STATUS_MIN_LEN 9
#define MACE_MSG_ID_200_LEN 9
#define MACE_MSG_ID_200_MIN_LEN 9

#define MACE_MSG_ID_RADIO_STATUS_CRC 185
#define MACE_MSG_ID_200_CRC 185



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_RADIO_STATUS { \
    200, \
    "RADIO_STATUS", \
    7, \
    {  { "rxerrors", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_radio_status_t, rxerrors) }, \
         { "fixed", NULL, MACE_TYPE_UINT16_T, 0, 2, offsetof(mace_radio_status_t, fixed) }, \
         { "rssi", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_radio_status_t, rssi) }, \
         { "remrssi", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_radio_status_t, remrssi) }, \
         { "txbuf", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_radio_status_t, txbuf) }, \
         { "noise", NULL, MACE_TYPE_UINT8_T, 0, 7, offsetof(mace_radio_status_t, noise) }, \
         { "remnoise", NULL, MACE_TYPE_UINT8_T, 0, 8, offsetof(mace_radio_status_t, remnoise) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_RADIO_STATUS { \
    "RADIO_STATUS", \
    7, \
    {  { "rxerrors", NULL, MACE_TYPE_UINT16_T, 0, 0, offsetof(mace_radio_status_t, rxerrors) }, \
         { "fixed", NULL, MACE_TYPE_UINT16_T, 0, 2, offsetof(mace_radio_status_t, fixed) }, \
         { "rssi", NULL, MACE_TYPE_UINT8_T, 0, 4, offsetof(mace_radio_status_t, rssi) }, \
         { "remrssi", NULL, MACE_TYPE_UINT8_T, 0, 5, offsetof(mace_radio_status_t, remrssi) }, \
         { "txbuf", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_radio_status_t, txbuf) }, \
         { "noise", NULL, MACE_TYPE_UINT8_T, 0, 7, offsetof(mace_radio_status_t, noise) }, \
         { "remnoise", NULL, MACE_TYPE_UINT8_T, 0, 8, offsetof(mace_radio_status_t, remnoise) }, \
         } \
}
#endif

/**
 * @brief Pack a radio_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rssi Local signal strength
 * @param remrssi Remote signal strength
 * @param txbuf Remaining free buffer space in percent.
 * @param noise Background noise level
 * @param remnoise Remote background noise level
 * @param rxerrors Receive errors
 * @param fixed Count of error corrected packets
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_radio_status_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_RADIO_STATUS_LEN];
    _mace_put_uint16_t(buf, 0, rxerrors);
    _mace_put_uint16_t(buf, 2, fixed);
    _mace_put_uint8_t(buf, 4, rssi);
    _mace_put_uint8_t(buf, 5, remrssi);
    _mace_put_uint8_t(buf, 6, txbuf);
    _mace_put_uint8_t(buf, 7, noise);
    _mace_put_uint8_t(buf, 8, remnoise);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_RADIO_STATUS_LEN);
#else
    mace_radio_status_t packet;
    packet.rxerrors = rxerrors;
    packet.fixed = fixed;
    packet.rssi = rssi;
    packet.remrssi = remrssi;
    packet.txbuf = txbuf;
    packet.noise = noise;
    packet.remnoise = remnoise;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_RADIO_STATUS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_RADIO_STATUS;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_RADIO_STATUS_MIN_LEN, MACE_MSG_ID_RADIO_STATUS_LEN, MACE_MSG_ID_RADIO_STATUS_CRC);
}

/**
 * @brief Pack a radio_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rssi Local signal strength
 * @param remrssi Remote signal strength
 * @param txbuf Remaining free buffer space in percent.
 * @param noise Background noise level
 * @param remnoise Remote background noise level
 * @param rxerrors Receive errors
 * @param fixed Count of error corrected packets
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_radio_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t rssi,uint8_t remrssi,uint8_t txbuf,uint8_t noise,uint8_t remnoise,uint16_t rxerrors,uint16_t fixed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_RADIO_STATUS_LEN];
    _mace_put_uint16_t(buf, 0, rxerrors);
    _mace_put_uint16_t(buf, 2, fixed);
    _mace_put_uint8_t(buf, 4, rssi);
    _mace_put_uint8_t(buf, 5, remrssi);
    _mace_put_uint8_t(buf, 6, txbuf);
    _mace_put_uint8_t(buf, 7, noise);
    _mace_put_uint8_t(buf, 8, remnoise);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_RADIO_STATUS_LEN);
#else
    mace_radio_status_t packet;
    packet.rxerrors = rxerrors;
    packet.fixed = fixed;
    packet.rssi = rssi;
    packet.remrssi = remrssi;
    packet.txbuf = txbuf;
    packet.noise = noise;
    packet.remnoise = remnoise;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_RADIO_STATUS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_RADIO_STATUS;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_RADIO_STATUS_MIN_LEN, MACE_MSG_ID_RADIO_STATUS_LEN, MACE_MSG_ID_RADIO_STATUS_CRC);
}

/**
 * @brief Encode a radio_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radio_status C-struct to read the message contents from
 */
static inline uint16_t mace_msg_radio_status_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_radio_status_t* radio_status)
{
    return mace_msg_radio_status_pack(system_id, component_id, msg, radio_status->rssi, radio_status->remrssi, radio_status->txbuf, radio_status->noise, radio_status->remnoise, radio_status->rxerrors, radio_status->fixed);
}

/**
 * @brief Encode a radio_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param radio_status C-struct to read the message contents from
 */
static inline uint16_t mace_msg_radio_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_radio_status_t* radio_status)
{
    return mace_msg_radio_status_pack_chan(system_id, component_id, chan, msg, radio_status->rssi, radio_status->remrssi, radio_status->txbuf, radio_status->noise, radio_status->remnoise, radio_status->rxerrors, radio_status->fixed);
}

/**
 * @brief Send a radio_status message
 * @param chan MAVLink channel to send the message
 *
 * @param rssi Local signal strength
 * @param remrssi Remote signal strength
 * @param txbuf Remaining free buffer space in percent.
 * @param noise Background noise level
 * @param remnoise Remote background noise level
 * @param rxerrors Receive errors
 * @param fixed Count of error corrected packets
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_radio_status_send(mace_channel_t chan, uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_RADIO_STATUS_LEN];
    _mace_put_uint16_t(buf, 0, rxerrors);
    _mace_put_uint16_t(buf, 2, fixed);
    _mace_put_uint8_t(buf, 4, rssi);
    _mace_put_uint8_t(buf, 5, remrssi);
    _mace_put_uint8_t(buf, 6, txbuf);
    _mace_put_uint8_t(buf, 7, noise);
    _mace_put_uint8_t(buf, 8, remnoise);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_RADIO_STATUS, buf, MACE_MSG_ID_RADIO_STATUS_MIN_LEN, MACE_MSG_ID_RADIO_STATUS_LEN, MACE_MSG_ID_RADIO_STATUS_CRC);
#else
    mace_radio_status_t packet;
    packet.rxerrors = rxerrors;
    packet.fixed = fixed;
    packet.rssi = rssi;
    packet.remrssi = remrssi;
    packet.txbuf = txbuf;
    packet.noise = noise;
    packet.remnoise = remnoise;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_RADIO_STATUS, (const char *)&packet, MACE_MSG_ID_RADIO_STATUS_MIN_LEN, MACE_MSG_ID_RADIO_STATUS_LEN, MACE_MSG_ID_RADIO_STATUS_CRC);
#endif
}

/**
 * @brief Send a radio_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_radio_status_send_struct(mace_channel_t chan, const mace_radio_status_t* radio_status)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_radio_status_send(chan, radio_status->rssi, radio_status->remrssi, radio_status->txbuf, radio_status->noise, radio_status->remnoise, radio_status->rxerrors, radio_status->fixed);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_RADIO_STATUS, (const char *)radio_status, MACE_MSG_ID_RADIO_STATUS_MIN_LEN, MACE_MSG_ID_RADIO_STATUS_LEN, MACE_MSG_ID_RADIO_STATUS_CRC);
#endif
}

#if MACE_MSG_ID_RADIO_STATUS_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_radio_status_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint16_t(buf, 0, rxerrors);
    _mace_put_uint16_t(buf, 2, fixed);
    _mace_put_uint8_t(buf, 4, rssi);
    _mace_put_uint8_t(buf, 5, remrssi);
    _mace_put_uint8_t(buf, 6, txbuf);
    _mace_put_uint8_t(buf, 7, noise);
    _mace_put_uint8_t(buf, 8, remnoise);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_RADIO_STATUS, buf, MACE_MSG_ID_RADIO_STATUS_MIN_LEN, MACE_MSG_ID_RADIO_STATUS_LEN, MACE_MSG_ID_RADIO_STATUS_CRC);
#else
    mace_radio_status_t *packet = (mace_radio_status_t *)msgbuf;
    packet->rxerrors = rxerrors;
    packet->fixed = fixed;
    packet->rssi = rssi;
    packet->remrssi = remrssi;
    packet->txbuf = txbuf;
    packet->noise = noise;
    packet->remnoise = remnoise;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_RADIO_STATUS, (const char *)packet, MACE_MSG_ID_RADIO_STATUS_MIN_LEN, MACE_MSG_ID_RADIO_STATUS_LEN, MACE_MSG_ID_RADIO_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE RADIO_STATUS UNPACKING


/**
 * @brief Get field rssi from radio_status message
 *
 * @return Local signal strength
 */
static inline uint8_t mace_msg_radio_status_get_rssi(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field remrssi from radio_status message
 *
 * @return Remote signal strength
 */
static inline uint8_t mace_msg_radio_status_get_remrssi(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field txbuf from radio_status message
 *
 * @return Remaining free buffer space in percent.
 */
static inline uint8_t mace_msg_radio_status_get_txbuf(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field noise from radio_status message
 *
 * @return Background noise level
 */
static inline uint8_t mace_msg_radio_status_get_noise(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field remnoise from radio_status message
 *
 * @return Remote background noise level
 */
static inline uint8_t mace_msg_radio_status_get_remnoise(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field rxerrors from radio_status message
 *
 * @return Receive errors
 */
static inline uint16_t mace_msg_radio_status_get_rxerrors(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field fixed from radio_status message
 *
 * @return Count of error corrected packets
 */
static inline uint16_t mace_msg_radio_status_get_fixed(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a radio_status message into a struct
 *
 * @param msg The message to decode
 * @param radio_status C-struct to decode the message contents into
 */
static inline void mace_msg_radio_status_decode(const mace_message_t* msg, mace_radio_status_t* radio_status)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    radio_status->rxerrors = mace_msg_radio_status_get_rxerrors(msg);
    radio_status->fixed = mace_msg_radio_status_get_fixed(msg);
    radio_status->rssi = mace_msg_radio_status_get_rssi(msg);
    radio_status->remrssi = mace_msg_radio_status_get_remrssi(msg);
    radio_status->txbuf = mace_msg_radio_status_get_txbuf(msg);
    radio_status->noise = mace_msg_radio_status_get_noise(msg);
    radio_status->remnoise = mace_msg_radio_status_get_remnoise(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_RADIO_STATUS_LEN? msg->len : MACE_MSG_ID_RADIO_STATUS_LEN;
        memset(radio_status, 0, MACE_MSG_ID_RADIO_STATUS_LEN);
    memcpy(radio_status, _MACE_PAYLOAD(msg), len);
#endif
}
