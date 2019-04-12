#pragma once
// MESSAGE GPS_STATUS PACKING

#define MACE_MSG_ID_GPS_STATUS 14

MACEPACKED(
typedef struct __mace_gps_status_t {
 uint8_t satellites_visible; /*< Number of satellites visible*/
 uint8_t satellite_prn[20]; /*< Global satellite ID*/
 uint8_t satellite_used[20]; /*< 0: Satellite not used, 1: used for localization*/
 uint8_t satellite_elevation[20]; /*< Elevation (0: right on top of receiver, 90: on the horizon) of satellite*/
 uint8_t satellite_azimuth[20]; /*< Direction of satellite, 0: 0 deg, 255: 360 deg.*/
 uint8_t satellite_snr[20]; /*< Signal to noise ratio of satellite*/
}) mace_gps_status_t;

#define MACE_MSG_ID_GPS_STATUS_LEN 101
#define MACE_MSG_ID_GPS_STATUS_MIN_LEN 101
#define MACE_MSG_ID_14_LEN 101
#define MACE_MSG_ID_14_MIN_LEN 101

#define MACE_MSG_ID_GPS_STATUS_CRC 23
#define MACE_MSG_ID_14_CRC 23

#define MACE_MSG_GPS_STATUS_FIELD_SATELLITE_PRN_LEN 20
#define MACE_MSG_GPS_STATUS_FIELD_SATELLITE_USED_LEN 20
#define MACE_MSG_GPS_STATUS_FIELD_SATELLITE_ELEVATION_LEN 20
#define MACE_MSG_GPS_STATUS_FIELD_SATELLITE_AZIMUTH_LEN 20
#define MACE_MSG_GPS_STATUS_FIELD_SATELLITE_SNR_LEN 20

#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_GPS_STATUS { \
    14, \
    "GPS_STATUS", \
    6, \
    {  { "satellites_visible", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_gps_status_t, satellites_visible) }, \
         { "satellite_prn", NULL, MACE_TYPE_UINT8_T, 20, 1, offsetof(mace_gps_status_t, satellite_prn) }, \
         { "satellite_used", NULL, MACE_TYPE_UINT8_T, 20, 21, offsetof(mace_gps_status_t, satellite_used) }, \
         { "satellite_elevation", NULL, MACE_TYPE_UINT8_T, 20, 41, offsetof(mace_gps_status_t, satellite_elevation) }, \
         { "satellite_azimuth", NULL, MACE_TYPE_UINT8_T, 20, 61, offsetof(mace_gps_status_t, satellite_azimuth) }, \
         { "satellite_snr", NULL, MACE_TYPE_UINT8_T, 20, 81, offsetof(mace_gps_status_t, satellite_snr) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_GPS_STATUS { \
    "GPS_STATUS", \
    6, \
    {  { "satellites_visible", NULL, MACE_TYPE_UINT8_T, 0, 0, offsetof(mace_gps_status_t, satellites_visible) }, \
         { "satellite_prn", NULL, MACE_TYPE_UINT8_T, 20, 1, offsetof(mace_gps_status_t, satellite_prn) }, \
         { "satellite_used", NULL, MACE_TYPE_UINT8_T, 20, 21, offsetof(mace_gps_status_t, satellite_used) }, \
         { "satellite_elevation", NULL, MACE_TYPE_UINT8_T, 20, 41, offsetof(mace_gps_status_t, satellite_elevation) }, \
         { "satellite_azimuth", NULL, MACE_TYPE_UINT8_T, 20, 61, offsetof(mace_gps_status_t, satellite_azimuth) }, \
         { "satellite_snr", NULL, MACE_TYPE_UINT8_T, 20, 81, offsetof(mace_gps_status_t, satellite_snr) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param satellites_visible Number of satellites visible
 * @param satellite_prn Global satellite ID
 * @param satellite_used 0: Satellite not used, 1: used for localization
 * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
 * @param satellite_snr Signal to noise ratio of satellite
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_gps_status_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t satellites_visible, const uint8_t *satellite_prn, const uint8_t *satellite_used, const uint8_t *satellite_elevation, const uint8_t *satellite_azimuth, const uint8_t *satellite_snr)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GPS_STATUS_LEN];
    _mace_put_uint8_t(buf, 0, satellites_visible);
    _mace_put_uint8_t_array(buf, 1, satellite_prn, 20);
    _mace_put_uint8_t_array(buf, 21, satellite_used, 20);
    _mace_put_uint8_t_array(buf, 41, satellite_elevation, 20);
    _mace_put_uint8_t_array(buf, 61, satellite_azimuth, 20);
    _mace_put_uint8_t_array(buf, 81, satellite_snr, 20);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GPS_STATUS_LEN);
#else
    mace_gps_status_t packet;
    packet.satellites_visible = satellites_visible;
    mace_array_memcpy(packet.satellite_prn, satellite_prn, sizeof(uint8_t)*20);
    mace_array_memcpy(packet.satellite_used, satellite_used, sizeof(uint8_t)*20);
    mace_array_memcpy(packet.satellite_elevation, satellite_elevation, sizeof(uint8_t)*20);
    mace_array_memcpy(packet.satellite_azimuth, satellite_azimuth, sizeof(uint8_t)*20);
    mace_array_memcpy(packet.satellite_snr, satellite_snr, sizeof(uint8_t)*20);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GPS_STATUS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GPS_STATUS;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_GPS_STATUS_MIN_LEN, MACE_MSG_ID_GPS_STATUS_LEN, MACE_MSG_ID_GPS_STATUS_CRC);
}

/**
 * @brief Pack a gps_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param satellites_visible Number of satellites visible
 * @param satellite_prn Global satellite ID
 * @param satellite_used 0: Satellite not used, 1: used for localization
 * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
 * @param satellite_snr Signal to noise ratio of satellite
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_gps_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t satellites_visible,const uint8_t *satellite_prn,const uint8_t *satellite_used,const uint8_t *satellite_elevation,const uint8_t *satellite_azimuth,const uint8_t *satellite_snr)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GPS_STATUS_LEN];
    _mace_put_uint8_t(buf, 0, satellites_visible);
    _mace_put_uint8_t_array(buf, 1, satellite_prn, 20);
    _mace_put_uint8_t_array(buf, 21, satellite_used, 20);
    _mace_put_uint8_t_array(buf, 41, satellite_elevation, 20);
    _mace_put_uint8_t_array(buf, 61, satellite_azimuth, 20);
    _mace_put_uint8_t_array(buf, 81, satellite_snr, 20);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_GPS_STATUS_LEN);
#else
    mace_gps_status_t packet;
    packet.satellites_visible = satellites_visible;
    mace_array_memcpy(packet.satellite_prn, satellite_prn, sizeof(uint8_t)*20);
    mace_array_memcpy(packet.satellite_used, satellite_used, sizeof(uint8_t)*20);
    mace_array_memcpy(packet.satellite_elevation, satellite_elevation, sizeof(uint8_t)*20);
    mace_array_memcpy(packet.satellite_azimuth, satellite_azimuth, sizeof(uint8_t)*20);
    mace_array_memcpy(packet.satellite_snr, satellite_snr, sizeof(uint8_t)*20);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_GPS_STATUS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_GPS_STATUS;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_GPS_STATUS_MIN_LEN, MACE_MSG_ID_GPS_STATUS_LEN, MACE_MSG_ID_GPS_STATUS_CRC);
}

/**
 * @brief Encode a gps_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_status C-struct to read the message contents from
 */
static inline uint16_t mace_msg_gps_status_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_gps_status_t* gps_status)
{
    return mace_msg_gps_status_pack(system_id, component_id, msg, gps_status->satellites_visible, gps_status->satellite_prn, gps_status->satellite_used, gps_status->satellite_elevation, gps_status->satellite_azimuth, gps_status->satellite_snr);
}

/**
 * @brief Encode a gps_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_status C-struct to read the message contents from
 */
static inline uint16_t mace_msg_gps_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_gps_status_t* gps_status)
{
    return mace_msg_gps_status_pack_chan(system_id, component_id, chan, msg, gps_status->satellites_visible, gps_status->satellite_prn, gps_status->satellite_used, gps_status->satellite_elevation, gps_status->satellite_azimuth, gps_status->satellite_snr);
}

/**
 * @brief Send a gps_status message
 * @param chan MAVLink channel to send the message
 *
 * @param satellites_visible Number of satellites visible
 * @param satellite_prn Global satellite ID
 * @param satellite_used 0: Satellite not used, 1: used for localization
 * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
 * @param satellite_snr Signal to noise ratio of satellite
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_gps_status_send(mace_channel_t chan, uint8_t satellites_visible, const uint8_t *satellite_prn, const uint8_t *satellite_used, const uint8_t *satellite_elevation, const uint8_t *satellite_azimuth, const uint8_t *satellite_snr)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_GPS_STATUS_LEN];
    _mace_put_uint8_t(buf, 0, satellites_visible);
    _mace_put_uint8_t_array(buf, 1, satellite_prn, 20);
    _mace_put_uint8_t_array(buf, 21, satellite_used, 20);
    _mace_put_uint8_t_array(buf, 41, satellite_elevation, 20);
    _mace_put_uint8_t_array(buf, 61, satellite_azimuth, 20);
    _mace_put_uint8_t_array(buf, 81, satellite_snr, 20);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_STATUS, buf, MACE_MSG_ID_GPS_STATUS_MIN_LEN, MACE_MSG_ID_GPS_STATUS_LEN, MACE_MSG_ID_GPS_STATUS_CRC);
#else
    mace_gps_status_t packet;
    packet.satellites_visible = satellites_visible;
    mace_array_memcpy(packet.satellite_prn, satellite_prn, sizeof(uint8_t)*20);
    mace_array_memcpy(packet.satellite_used, satellite_used, sizeof(uint8_t)*20);
    mace_array_memcpy(packet.satellite_elevation, satellite_elevation, sizeof(uint8_t)*20);
    mace_array_memcpy(packet.satellite_azimuth, satellite_azimuth, sizeof(uint8_t)*20);
    mace_array_memcpy(packet.satellite_snr, satellite_snr, sizeof(uint8_t)*20);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_STATUS, (const char *)&packet, MACE_MSG_ID_GPS_STATUS_MIN_LEN, MACE_MSG_ID_GPS_STATUS_LEN, MACE_MSG_ID_GPS_STATUS_CRC);
#endif
}

/**
 * @brief Send a gps_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_gps_status_send_struct(mace_channel_t chan, const mace_gps_status_t* gps_status)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_gps_status_send(chan, gps_status->satellites_visible, gps_status->satellite_prn, gps_status->satellite_used, gps_status->satellite_elevation, gps_status->satellite_azimuth, gps_status->satellite_snr);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_STATUS, (const char *)gps_status, MACE_MSG_ID_GPS_STATUS_MIN_LEN, MACE_MSG_ID_GPS_STATUS_LEN, MACE_MSG_ID_GPS_STATUS_CRC);
#endif
}

#if MACE_MSG_ID_GPS_STATUS_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_gps_status_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t satellites_visible, const uint8_t *satellite_prn, const uint8_t *satellite_used, const uint8_t *satellite_elevation, const uint8_t *satellite_azimuth, const uint8_t *satellite_snr)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint8_t(buf, 0, satellites_visible);
    _mace_put_uint8_t_array(buf, 1, satellite_prn, 20);
    _mace_put_uint8_t_array(buf, 21, satellite_used, 20);
    _mace_put_uint8_t_array(buf, 41, satellite_elevation, 20);
    _mace_put_uint8_t_array(buf, 61, satellite_azimuth, 20);
    _mace_put_uint8_t_array(buf, 81, satellite_snr, 20);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_STATUS, buf, MACE_MSG_ID_GPS_STATUS_MIN_LEN, MACE_MSG_ID_GPS_STATUS_LEN, MACE_MSG_ID_GPS_STATUS_CRC);
#else
    mace_gps_status_t *packet = (mace_gps_status_t *)msgbuf;
    packet->satellites_visible = satellites_visible;
    mace_array_memcpy(packet->satellite_prn, satellite_prn, sizeof(uint8_t)*20);
    mace_array_memcpy(packet->satellite_used, satellite_used, sizeof(uint8_t)*20);
    mace_array_memcpy(packet->satellite_elevation, satellite_elevation, sizeof(uint8_t)*20);
    mace_array_memcpy(packet->satellite_azimuth, satellite_azimuth, sizeof(uint8_t)*20);
    mace_array_memcpy(packet->satellite_snr, satellite_snr, sizeof(uint8_t)*20);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_GPS_STATUS, (const char *)packet, MACE_MSG_ID_GPS_STATUS_MIN_LEN, MACE_MSG_ID_GPS_STATUS_LEN, MACE_MSG_ID_GPS_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_STATUS UNPACKING


/**
 * @brief Get field satellites_visible from gps_status message
 *
 * @return Number of satellites visible
 */
static inline uint8_t mace_msg_gps_status_get_satellites_visible(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field satellite_prn from gps_status message
 *
 * @return Global satellite ID
 */
static inline uint16_t mace_msg_gps_status_get_satellite_prn(const mace_message_t* msg, uint8_t *satellite_prn)
{
    return _MACE_RETURN_uint8_t_array(msg, satellite_prn, 20,  1);
}

/**
 * @brief Get field satellite_used from gps_status message
 *
 * @return 0: Satellite not used, 1: used for localization
 */
static inline uint16_t mace_msg_gps_status_get_satellite_used(const mace_message_t* msg, uint8_t *satellite_used)
{
    return _MACE_RETURN_uint8_t_array(msg, satellite_used, 20,  21);
}

/**
 * @brief Get field satellite_elevation from gps_status message
 *
 * @return Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 */
static inline uint16_t mace_msg_gps_status_get_satellite_elevation(const mace_message_t* msg, uint8_t *satellite_elevation)
{
    return _MACE_RETURN_uint8_t_array(msg, satellite_elevation, 20,  41);
}

/**
 * @brief Get field satellite_azimuth from gps_status message
 *
 * @return Direction of satellite, 0: 0 deg, 255: 360 deg.
 */
static inline uint16_t mace_msg_gps_status_get_satellite_azimuth(const mace_message_t* msg, uint8_t *satellite_azimuth)
{
    return _MACE_RETURN_uint8_t_array(msg, satellite_azimuth, 20,  61);
}

/**
 * @brief Get field satellite_snr from gps_status message
 *
 * @return Signal to noise ratio of satellite
 */
static inline uint16_t mace_msg_gps_status_get_satellite_snr(const mace_message_t* msg, uint8_t *satellite_snr)
{
    return _MACE_RETURN_uint8_t_array(msg, satellite_snr, 20,  81);
}

/**
 * @brief Decode a gps_status message into a struct
 *
 * @param msg The message to decode
 * @param gps_status C-struct to decode the message contents into
 */
static inline void mace_msg_gps_status_decode(const mace_message_t* msg, mace_gps_status_t* gps_status)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    gps_status->satellites_visible = mace_msg_gps_status_get_satellites_visible(msg);
    mace_msg_gps_status_get_satellite_prn(msg, gps_status->satellite_prn);
    mace_msg_gps_status_get_satellite_used(msg, gps_status->satellite_used);
    mace_msg_gps_status_get_satellite_elevation(msg, gps_status->satellite_elevation);
    mace_msg_gps_status_get_satellite_azimuth(msg, gps_status->satellite_azimuth);
    mace_msg_gps_status_get_satellite_snr(msg, gps_status->satellite_snr);
#else
        uint8_t len = msg->len < MACE_MSG_ID_GPS_STATUS_LEN? msg->len : MACE_MSG_ID_GPS_STATUS_LEN;
        memset(gps_status, 0, MACE_MSG_ID_GPS_STATUS_LEN);
    memcpy(gps_status, _MACE_PAYLOAD(msg), len);
#endif
}
