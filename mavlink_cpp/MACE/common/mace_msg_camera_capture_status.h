#pragma once
// MESSAGE CAMERA_CAPTURE_STATUS PACKING

#define MACE_MSG_ID_CAMERA_CAPTURE_STATUS 262

MACEPACKED(
typedef struct __mace_camera_capture_status_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float image_interval; /*< Image capture interval in seconds*/
 float video_framerate; /*< Video frame rate in Hz*/
 uint32_t recording_time_ms; /*< Time in milliseconds since recording started*/
 float available_capacity; /*< Available storage capacity in MiB*/
 uint16_t image_resolution_h; /*< Image resolution in pixels horizontal*/
 uint16_t image_resolution_v; /*< Image resolution in pixels vertical*/
 uint16_t video_resolution_h; /*< Video resolution in pixels horizontal*/
 uint16_t video_resolution_v; /*< Video resolution in pixels vertical*/
 uint8_t camera_id; /*< Camera ID if there are multiple*/
 uint8_t image_status; /*< Current status of image capturing (0: not running, 1: interval capture in progress)*/
 uint8_t video_status; /*< Current status of video capturing (0: not running, 1: capture in progress)*/
}) mace_camera_capture_status_t;

#define MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN 31
#define MACE_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN 31
#define MACE_MSG_ID_262_LEN 31
#define MACE_MSG_ID_262_MIN_LEN 31

#define MACE_MSG_ID_CAMERA_CAPTURE_STATUS_CRC 69
#define MACE_MSG_ID_262_CRC 69



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_CAMERA_CAPTURE_STATUS { \
    262, \
    "CAMERA_CAPTURE_STATUS", \
    12, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_camera_capture_status_t, time_boot_ms) }, \
         { "image_interval", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_camera_capture_status_t, image_interval) }, \
         { "video_framerate", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_camera_capture_status_t, video_framerate) }, \
         { "recording_time_ms", NULL, MACE_TYPE_UINT32_T, 0, 12, offsetof(mace_camera_capture_status_t, recording_time_ms) }, \
         { "available_capacity", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_camera_capture_status_t, available_capacity) }, \
         { "image_resolution_h", NULL, MACE_TYPE_UINT16_T, 0, 20, offsetof(mace_camera_capture_status_t, image_resolution_h) }, \
         { "image_resolution_v", NULL, MACE_TYPE_UINT16_T, 0, 22, offsetof(mace_camera_capture_status_t, image_resolution_v) }, \
         { "video_resolution_h", NULL, MACE_TYPE_UINT16_T, 0, 24, offsetof(mace_camera_capture_status_t, video_resolution_h) }, \
         { "video_resolution_v", NULL, MACE_TYPE_UINT16_T, 0, 26, offsetof(mace_camera_capture_status_t, video_resolution_v) }, \
         { "camera_id", NULL, MACE_TYPE_UINT8_T, 0, 28, offsetof(mace_camera_capture_status_t, camera_id) }, \
         { "image_status", NULL, MACE_TYPE_UINT8_T, 0, 29, offsetof(mace_camera_capture_status_t, image_status) }, \
         { "video_status", NULL, MACE_TYPE_UINT8_T, 0, 30, offsetof(mace_camera_capture_status_t, video_status) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_CAMERA_CAPTURE_STATUS { \
    "CAMERA_CAPTURE_STATUS", \
    12, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_camera_capture_status_t, time_boot_ms) }, \
         { "image_interval", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_camera_capture_status_t, image_interval) }, \
         { "video_framerate", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_camera_capture_status_t, video_framerate) }, \
         { "recording_time_ms", NULL, MACE_TYPE_UINT32_T, 0, 12, offsetof(mace_camera_capture_status_t, recording_time_ms) }, \
         { "available_capacity", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_camera_capture_status_t, available_capacity) }, \
         { "image_resolution_h", NULL, MACE_TYPE_UINT16_T, 0, 20, offsetof(mace_camera_capture_status_t, image_resolution_h) }, \
         { "image_resolution_v", NULL, MACE_TYPE_UINT16_T, 0, 22, offsetof(mace_camera_capture_status_t, image_resolution_v) }, \
         { "video_resolution_h", NULL, MACE_TYPE_UINT16_T, 0, 24, offsetof(mace_camera_capture_status_t, video_resolution_h) }, \
         { "video_resolution_v", NULL, MACE_TYPE_UINT16_T, 0, 26, offsetof(mace_camera_capture_status_t, video_resolution_v) }, \
         { "camera_id", NULL, MACE_TYPE_UINT8_T, 0, 28, offsetof(mace_camera_capture_status_t, camera_id) }, \
         { "image_status", NULL, MACE_TYPE_UINT8_T, 0, 29, offsetof(mace_camera_capture_status_t, image_status) }, \
         { "video_status", NULL, MACE_TYPE_UINT8_T, 0, 30, offsetof(mace_camera_capture_status_t, video_status) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_capture_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param camera_id Camera ID if there are multiple
 * @param image_status Current status of image capturing (0: not running, 1: interval capture in progress)
 * @param video_status Current status of video capturing (0: not running, 1: capture in progress)
 * @param image_interval Image capture interval in seconds
 * @param video_framerate Video frame rate in Hz
 * @param image_resolution_h Image resolution in pixels horizontal
 * @param image_resolution_v Image resolution in pixels vertical
 * @param video_resolution_h Video resolution in pixels horizontal
 * @param video_resolution_v Video resolution in pixels vertical
 * @param recording_time_ms Time in milliseconds since recording started
 * @param available_capacity Available storage capacity in MiB
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_camera_capture_status_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint32_t time_boot_ms, uint8_t camera_id, uint8_t image_status, uint8_t video_status, float image_interval, float video_framerate, uint16_t image_resolution_h, uint16_t image_resolution_v, uint16_t video_resolution_h, uint16_t video_resolution_v, uint32_t recording_time_ms, float available_capacity)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, image_interval);
    _mace_put_float(buf, 8, video_framerate);
    _mace_put_uint32_t(buf, 12, recording_time_ms);
    _mace_put_float(buf, 16, available_capacity);
    _mace_put_uint16_t(buf, 20, image_resolution_h);
    _mace_put_uint16_t(buf, 22, image_resolution_v);
    _mace_put_uint16_t(buf, 24, video_resolution_h);
    _mace_put_uint16_t(buf, 26, video_resolution_v);
    _mace_put_uint8_t(buf, 28, camera_id);
    _mace_put_uint8_t(buf, 29, image_status);
    _mace_put_uint8_t(buf, 30, video_status);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
#else
    mace_camera_capture_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.image_interval = image_interval;
    packet.video_framerate = video_framerate;
    packet.recording_time_ms = recording_time_ms;
    packet.available_capacity = available_capacity;
    packet.image_resolution_h = image_resolution_h;
    packet.image_resolution_v = image_resolution_v;
    packet.video_resolution_h = video_resolution_h;
    packet.video_resolution_v = video_resolution_v;
    packet.camera_id = camera_id;
    packet.image_status = image_status;
    packet.video_status = video_status;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_CAMERA_CAPTURE_STATUS;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
}

/**
 * @brief Pack a camera_capture_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param camera_id Camera ID if there are multiple
 * @param image_status Current status of image capturing (0: not running, 1: interval capture in progress)
 * @param video_status Current status of video capturing (0: not running, 1: capture in progress)
 * @param image_interval Image capture interval in seconds
 * @param video_framerate Video frame rate in Hz
 * @param image_resolution_h Image resolution in pixels horizontal
 * @param image_resolution_v Image resolution in pixels vertical
 * @param video_resolution_h Video resolution in pixels horizontal
 * @param video_resolution_v Video resolution in pixels vertical
 * @param recording_time_ms Time in milliseconds since recording started
 * @param available_capacity Available storage capacity in MiB
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_camera_capture_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t camera_id,uint8_t image_status,uint8_t video_status,float image_interval,float video_framerate,uint16_t image_resolution_h,uint16_t image_resolution_v,uint16_t video_resolution_h,uint16_t video_resolution_v,uint32_t recording_time_ms,float available_capacity)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, image_interval);
    _mace_put_float(buf, 8, video_framerate);
    _mace_put_uint32_t(buf, 12, recording_time_ms);
    _mace_put_float(buf, 16, available_capacity);
    _mace_put_uint16_t(buf, 20, image_resolution_h);
    _mace_put_uint16_t(buf, 22, image_resolution_v);
    _mace_put_uint16_t(buf, 24, video_resolution_h);
    _mace_put_uint16_t(buf, 26, video_resolution_v);
    _mace_put_uint8_t(buf, 28, camera_id);
    _mace_put_uint8_t(buf, 29, image_status);
    _mace_put_uint8_t(buf, 30, video_status);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
#else
    mace_camera_capture_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.image_interval = image_interval;
    packet.video_framerate = video_framerate;
    packet.recording_time_ms = recording_time_ms;
    packet.available_capacity = available_capacity;
    packet.image_resolution_h = image_resolution_h;
    packet.image_resolution_v = image_resolution_v;
    packet.video_resolution_h = video_resolution_h;
    packet.video_resolution_v = video_resolution_v;
    packet.camera_id = camera_id;
    packet.image_status = image_status;
    packet.video_status = video_status;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_CAMERA_CAPTURE_STATUS;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
}

/**
 * @brief Encode a camera_capture_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_capture_status C-struct to read the message contents from
 */
static inline uint16_t mace_msg_camera_capture_status_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_camera_capture_status_t* camera_capture_status)
{
    return mace_msg_camera_capture_status_pack(system_id, component_id, msg, camera_capture_status->time_boot_ms, camera_capture_status->camera_id, camera_capture_status->image_status, camera_capture_status->video_status, camera_capture_status->image_interval, camera_capture_status->video_framerate, camera_capture_status->image_resolution_h, camera_capture_status->image_resolution_v, camera_capture_status->video_resolution_h, camera_capture_status->video_resolution_v, camera_capture_status->recording_time_ms, camera_capture_status->available_capacity);
}

/**
 * @brief Encode a camera_capture_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_capture_status C-struct to read the message contents from
 */
static inline uint16_t mace_msg_camera_capture_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_camera_capture_status_t* camera_capture_status)
{
    return mace_msg_camera_capture_status_pack_chan(system_id, component_id, chan, msg, camera_capture_status->time_boot_ms, camera_capture_status->camera_id, camera_capture_status->image_status, camera_capture_status->video_status, camera_capture_status->image_interval, camera_capture_status->video_framerate, camera_capture_status->image_resolution_h, camera_capture_status->image_resolution_v, camera_capture_status->video_resolution_h, camera_capture_status->video_resolution_v, camera_capture_status->recording_time_ms, camera_capture_status->available_capacity);
}

/**
 * @brief Send a camera_capture_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param camera_id Camera ID if there are multiple
 * @param image_status Current status of image capturing (0: not running, 1: interval capture in progress)
 * @param video_status Current status of video capturing (0: not running, 1: capture in progress)
 * @param image_interval Image capture interval in seconds
 * @param video_framerate Video frame rate in Hz
 * @param image_resolution_h Image resolution in pixels horizontal
 * @param image_resolution_v Image resolution in pixels vertical
 * @param video_resolution_h Video resolution in pixels horizontal
 * @param video_resolution_v Video resolution in pixels vertical
 * @param recording_time_ms Time in milliseconds since recording started
 * @param available_capacity Available storage capacity in MiB
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_camera_capture_status_send(mace_channel_t chan, uint32_t time_boot_ms, uint8_t camera_id, uint8_t image_status, uint8_t video_status, float image_interval, float video_framerate, uint16_t image_resolution_h, uint16_t image_resolution_v, uint16_t video_resolution_h, uint16_t video_resolution_v, uint32_t recording_time_ms, float available_capacity)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, image_interval);
    _mace_put_float(buf, 8, video_framerate);
    _mace_put_uint32_t(buf, 12, recording_time_ms);
    _mace_put_float(buf, 16, available_capacity);
    _mace_put_uint16_t(buf, 20, image_resolution_h);
    _mace_put_uint16_t(buf, 22, image_resolution_v);
    _mace_put_uint16_t(buf, 24, video_resolution_h);
    _mace_put_uint16_t(buf, 26, video_resolution_v);
    _mace_put_uint8_t(buf, 28, camera_id);
    _mace_put_uint8_t(buf, 29, image_status);
    _mace_put_uint8_t(buf, 30, video_status);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_CAPTURE_STATUS, buf, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
#else
    mace_camera_capture_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.image_interval = image_interval;
    packet.video_framerate = video_framerate;
    packet.recording_time_ms = recording_time_ms;
    packet.available_capacity = available_capacity;
    packet.image_resolution_h = image_resolution_h;
    packet.image_resolution_v = image_resolution_v;
    packet.video_resolution_h = video_resolution_h;
    packet.video_resolution_v = video_resolution_v;
    packet.camera_id = camera_id;
    packet.image_status = image_status;
    packet.video_status = video_status;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_CAPTURE_STATUS, (const char *)&packet, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
#endif
}

/**
 * @brief Send a camera_capture_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_camera_capture_status_send_struct(mace_channel_t chan, const mace_camera_capture_status_t* camera_capture_status)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_camera_capture_status_send(chan, camera_capture_status->time_boot_ms, camera_capture_status->camera_id, camera_capture_status->image_status, camera_capture_status->video_status, camera_capture_status->image_interval, camera_capture_status->video_framerate, camera_capture_status->image_resolution_h, camera_capture_status->image_resolution_v, camera_capture_status->video_resolution_h, camera_capture_status->video_resolution_v, camera_capture_status->recording_time_ms, camera_capture_status->available_capacity);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_CAPTURE_STATUS, (const char *)camera_capture_status, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
#endif
}

#if MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_camera_capture_status_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint32_t time_boot_ms, uint8_t camera_id, uint8_t image_status, uint8_t video_status, float image_interval, float video_framerate, uint16_t image_resolution_h, uint16_t image_resolution_v, uint16_t video_resolution_h, uint16_t video_resolution_v, uint32_t recording_time_ms, float available_capacity)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, image_interval);
    _mace_put_float(buf, 8, video_framerate);
    _mace_put_uint32_t(buf, 12, recording_time_ms);
    _mace_put_float(buf, 16, available_capacity);
    _mace_put_uint16_t(buf, 20, image_resolution_h);
    _mace_put_uint16_t(buf, 22, image_resolution_v);
    _mace_put_uint16_t(buf, 24, video_resolution_h);
    _mace_put_uint16_t(buf, 26, video_resolution_v);
    _mace_put_uint8_t(buf, 28, camera_id);
    _mace_put_uint8_t(buf, 29, image_status);
    _mace_put_uint8_t(buf, 30, video_status);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_CAPTURE_STATUS, buf, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
#else
    mace_camera_capture_status_t *packet = (mace_camera_capture_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->image_interval = image_interval;
    packet->video_framerate = video_framerate;
    packet->recording_time_ms = recording_time_ms;
    packet->available_capacity = available_capacity;
    packet->image_resolution_h = image_resolution_h;
    packet->image_resolution_v = image_resolution_v;
    packet->video_resolution_h = video_resolution_h;
    packet->video_resolution_v = video_resolution_v;
    packet->camera_id = camera_id;
    packet->image_status = image_status;
    packet->video_status = video_status;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_CAPTURE_STATUS, (const char *)packet, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_CAPTURE_STATUS UNPACKING


/**
 * @brief Get field time_boot_ms from camera_capture_status message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mace_msg_camera_capture_status_get_time_boot_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field camera_id from camera_capture_status message
 *
 * @return Camera ID if there are multiple
 */
static inline uint8_t mace_msg_camera_capture_status_get_camera_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field image_status from camera_capture_status message
 *
 * @return Current status of image capturing (0: not running, 1: interval capture in progress)
 */
static inline uint8_t mace_msg_camera_capture_status_get_image_status(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field video_status from camera_capture_status message
 *
 * @return Current status of video capturing (0: not running, 1: capture in progress)
 */
static inline uint8_t mace_msg_camera_capture_status_get_video_status(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field image_interval from camera_capture_status message
 *
 * @return Image capture interval in seconds
 */
static inline float mace_msg_camera_capture_status_get_image_interval(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field video_framerate from camera_capture_status message
 *
 * @return Video frame rate in Hz
 */
static inline float mace_msg_camera_capture_status_get_video_framerate(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field image_resolution_h from camera_capture_status message
 *
 * @return Image resolution in pixels horizontal
 */
static inline uint16_t mace_msg_camera_capture_status_get_image_resolution_h(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field image_resolution_v from camera_capture_status message
 *
 * @return Image resolution in pixels vertical
 */
static inline uint16_t mace_msg_camera_capture_status_get_image_resolution_v(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field video_resolution_h from camera_capture_status message
 *
 * @return Video resolution in pixels horizontal
 */
static inline uint16_t mace_msg_camera_capture_status_get_video_resolution_h(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field video_resolution_v from camera_capture_status message
 *
 * @return Video resolution in pixels vertical
 */
static inline uint16_t mace_msg_camera_capture_status_get_video_resolution_v(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field recording_time_ms from camera_capture_status message
 *
 * @return Time in milliseconds since recording started
 */
static inline uint32_t mace_msg_camera_capture_status_get_recording_time_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field available_capacity from camera_capture_status message
 *
 * @return Available storage capacity in MiB
 */
static inline float mace_msg_camera_capture_status_get_available_capacity(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  16);
}

/**
 * @brief Decode a camera_capture_status message into a struct
 *
 * @param msg The message to decode
 * @param camera_capture_status C-struct to decode the message contents into
 */
static inline void mace_msg_camera_capture_status_decode(const mace_message_t* msg, mace_camera_capture_status_t* camera_capture_status)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    camera_capture_status->time_boot_ms = mace_msg_camera_capture_status_get_time_boot_ms(msg);
    camera_capture_status->image_interval = mace_msg_camera_capture_status_get_image_interval(msg);
    camera_capture_status->video_framerate = mace_msg_camera_capture_status_get_video_framerate(msg);
    camera_capture_status->recording_time_ms = mace_msg_camera_capture_status_get_recording_time_ms(msg);
    camera_capture_status->available_capacity = mace_msg_camera_capture_status_get_available_capacity(msg);
    camera_capture_status->image_resolution_h = mace_msg_camera_capture_status_get_image_resolution_h(msg);
    camera_capture_status->image_resolution_v = mace_msg_camera_capture_status_get_image_resolution_v(msg);
    camera_capture_status->video_resolution_h = mace_msg_camera_capture_status_get_video_resolution_h(msg);
    camera_capture_status->video_resolution_v = mace_msg_camera_capture_status_get_video_resolution_v(msg);
    camera_capture_status->camera_id = mace_msg_camera_capture_status_get_camera_id(msg);
    camera_capture_status->image_status = mace_msg_camera_capture_status_get_image_status(msg);
    camera_capture_status->video_status = mace_msg_camera_capture_status_get_video_status(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN? msg->len : MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN;
        memset(camera_capture_status, 0, MACE_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
    memcpy(camera_capture_status, _MACE_PAYLOAD(msg), len);
#endif
}
