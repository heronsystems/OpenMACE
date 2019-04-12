#pragma once
// MESSAGE CAMERA_IMAGE_CAPTURED PACKING

#define MACE_MSG_ID_CAMERA_IMAGE_CAPTURED 263

MACEPACKED(
typedef struct __mace_camera_image_captured_t {
 uint64_t time_utc; /*< Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.*/
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 int32_t lat; /*< Latitude, expressed as degrees * 1E7 where image was taken*/
 int32_t lon; /*< Longitude, expressed as degrees * 1E7 where capture was taken*/
 int32_t alt; /*< Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken*/
 int32_t relative_alt; /*< Altitude above ground in meters, expressed as * 1E3 where image was taken*/
 float q[4]; /*< Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)*/
 int32_t image_index; /*< Zero based index of this image (image count since armed -1)*/
 uint8_t camera_id; /*< Camera ID if there are multiple*/
 int8_t capture_result; /*< Boolean indicating success (1) or failure (0) while capturing this image.*/
 char file_url[205]; /*< URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.*/
}) mace_camera_image_captured_t;

#define MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN 255
#define MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN 255
#define MACE_MSG_ID_263_LEN 255
#define MACE_MSG_ID_263_MIN_LEN 255

#define MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC 133
#define MACE_MSG_ID_263_CRC 133

#define MACE_MSG_CAMERA_IMAGE_CAPTURED_FIELD_Q_LEN 4
#define MACE_MSG_CAMERA_IMAGE_CAPTURED_FIELD_FILE_URL_LEN 205

#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_CAMERA_IMAGE_CAPTURED { \
    263, \
    "CAMERA_IMAGE_CAPTURED", \
    11, \
    {  { "time_utc", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_camera_image_captured_t, time_utc) }, \
         { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 8, offsetof(mace_camera_image_captured_t, time_boot_ms) }, \
         { "lat", NULL, MACE_TYPE_INT32_T, 0, 12, offsetof(mace_camera_image_captured_t, lat) }, \
         { "lon", NULL, MACE_TYPE_INT32_T, 0, 16, offsetof(mace_camera_image_captured_t, lon) }, \
         { "alt", NULL, MACE_TYPE_INT32_T, 0, 20, offsetof(mace_camera_image_captured_t, alt) }, \
         { "relative_alt", NULL, MACE_TYPE_INT32_T, 0, 24, offsetof(mace_camera_image_captured_t, relative_alt) }, \
         { "q", NULL, MACE_TYPE_FLOAT, 4, 28, offsetof(mace_camera_image_captured_t, q) }, \
         { "image_index", NULL, MACE_TYPE_INT32_T, 0, 44, offsetof(mace_camera_image_captured_t, image_index) }, \
         { "camera_id", NULL, MACE_TYPE_UINT8_T, 0, 48, offsetof(mace_camera_image_captured_t, camera_id) }, \
         { "capture_result", NULL, MACE_TYPE_INT8_T, 0, 49, offsetof(mace_camera_image_captured_t, capture_result) }, \
         { "file_url", NULL, MACE_TYPE_CHAR, 205, 50, offsetof(mace_camera_image_captured_t, file_url) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_CAMERA_IMAGE_CAPTURED { \
    "CAMERA_IMAGE_CAPTURED", \
    11, \
    {  { "time_utc", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_camera_image_captured_t, time_utc) }, \
         { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 8, offsetof(mace_camera_image_captured_t, time_boot_ms) }, \
         { "lat", NULL, MACE_TYPE_INT32_T, 0, 12, offsetof(mace_camera_image_captured_t, lat) }, \
         { "lon", NULL, MACE_TYPE_INT32_T, 0, 16, offsetof(mace_camera_image_captured_t, lon) }, \
         { "alt", NULL, MACE_TYPE_INT32_T, 0, 20, offsetof(mace_camera_image_captured_t, alt) }, \
         { "relative_alt", NULL, MACE_TYPE_INT32_T, 0, 24, offsetof(mace_camera_image_captured_t, relative_alt) }, \
         { "q", NULL, MACE_TYPE_FLOAT, 4, 28, offsetof(mace_camera_image_captured_t, q) }, \
         { "image_index", NULL, MACE_TYPE_INT32_T, 0, 44, offsetof(mace_camera_image_captured_t, image_index) }, \
         { "camera_id", NULL, MACE_TYPE_UINT8_T, 0, 48, offsetof(mace_camera_image_captured_t, camera_id) }, \
         { "capture_result", NULL, MACE_TYPE_INT8_T, 0, 49, offsetof(mace_camera_image_captured_t, capture_result) }, \
         { "file_url", NULL, MACE_TYPE_CHAR, 205, 50, offsetof(mace_camera_image_captured_t, file_url) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_image_captured message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param time_utc Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
 * @param camera_id Camera ID if there are multiple
 * @param lat Latitude, expressed as degrees * 1E7 where image was taken
 * @param lon Longitude, expressed as degrees * 1E7 where capture was taken
 * @param alt Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
 * @param relative_alt Altitude above ground in meters, expressed as * 1E3 where image was taken
 * @param q Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
 * @param image_index Zero based index of this image (image count since armed -1)
 * @param capture_result Boolean indicating success (1) or failure (0) while capturing this image.
 * @param file_url URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_camera_image_captured_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float *q, int32_t image_index, int8_t capture_result, const char *file_url)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN];
    _mace_put_uint64_t(buf, 0, time_utc);
    _mace_put_uint32_t(buf, 8, time_boot_ms);
    _mace_put_int32_t(buf, 12, lat);
    _mace_put_int32_t(buf, 16, lon);
    _mace_put_int32_t(buf, 20, alt);
    _mace_put_int32_t(buf, 24, relative_alt);
    _mace_put_int32_t(buf, 44, image_index);
    _mace_put_uint8_t(buf, 48, camera_id);
    _mace_put_int8_t(buf, 49, capture_result);
    _mace_put_float_array(buf, 28, q, 4);
    _mace_put_char_array(buf, 50, file_url, 205);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN);
#else
    mace_camera_image_captured_t packet;
    packet.time_utc = time_utc;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.image_index = image_index;
    packet.camera_id = camera_id;
    packet.capture_result = capture_result;
    mace_array_memcpy(packet.q, q, sizeof(float)*4);
    mace_array_memcpy(packet.file_url, file_url, sizeof(char)*205);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN);
#endif

    msg->msgid = MACE_MSG_ID_CAMERA_IMAGE_CAPTURED;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
}

/**
 * @brief Pack a camera_image_captured message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param time_utc Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
 * @param camera_id Camera ID if there are multiple
 * @param lat Latitude, expressed as degrees * 1E7 where image was taken
 * @param lon Longitude, expressed as degrees * 1E7 where capture was taken
 * @param alt Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
 * @param relative_alt Altitude above ground in meters, expressed as * 1E3 where image was taken
 * @param q Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
 * @param image_index Zero based index of this image (image count since armed -1)
 * @param capture_result Boolean indicating success (1) or failure (0) while capturing this image.
 * @param file_url URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_camera_image_captured_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint32_t time_boot_ms,uint64_t time_utc,uint8_t camera_id,int32_t lat,int32_t lon,int32_t alt,int32_t relative_alt,const float *q,int32_t image_index,int8_t capture_result,const char *file_url)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN];
    _mace_put_uint64_t(buf, 0, time_utc);
    _mace_put_uint32_t(buf, 8, time_boot_ms);
    _mace_put_int32_t(buf, 12, lat);
    _mace_put_int32_t(buf, 16, lon);
    _mace_put_int32_t(buf, 20, alt);
    _mace_put_int32_t(buf, 24, relative_alt);
    _mace_put_int32_t(buf, 44, image_index);
    _mace_put_uint8_t(buf, 48, camera_id);
    _mace_put_int8_t(buf, 49, capture_result);
    _mace_put_float_array(buf, 28, q, 4);
    _mace_put_char_array(buf, 50, file_url, 205);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN);
#else
    mace_camera_image_captured_t packet;
    packet.time_utc = time_utc;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.image_index = image_index;
    packet.camera_id = camera_id;
    packet.capture_result = capture_result;
    mace_array_memcpy(packet.q, q, sizeof(float)*4);
    mace_array_memcpy(packet.file_url, file_url, sizeof(char)*205);
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN);
#endif

    msg->msgid = MACE_MSG_ID_CAMERA_IMAGE_CAPTURED;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
}

/**
 * @brief Encode a camera_image_captured struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_image_captured C-struct to read the message contents from
 */
static inline uint16_t mace_msg_camera_image_captured_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_camera_image_captured_t* camera_image_captured)
{
    return mace_msg_camera_image_captured_pack(system_id, component_id, msg, camera_image_captured->time_boot_ms, camera_image_captured->time_utc, camera_image_captured->camera_id, camera_image_captured->lat, camera_image_captured->lon, camera_image_captured->alt, camera_image_captured->relative_alt, camera_image_captured->q, camera_image_captured->image_index, camera_image_captured->capture_result, camera_image_captured->file_url);
}

/**
 * @brief Encode a camera_image_captured struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_image_captured C-struct to read the message contents from
 */
static inline uint16_t mace_msg_camera_image_captured_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_camera_image_captured_t* camera_image_captured)
{
    return mace_msg_camera_image_captured_pack_chan(system_id, component_id, chan, msg, camera_image_captured->time_boot_ms, camera_image_captured->time_utc, camera_image_captured->camera_id, camera_image_captured->lat, camera_image_captured->lon, camera_image_captured->alt, camera_image_captured->relative_alt, camera_image_captured->q, camera_image_captured->image_index, camera_image_captured->capture_result, camera_image_captured->file_url);
}

/**
 * @brief Send a camera_image_captured message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param time_utc Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
 * @param camera_id Camera ID if there are multiple
 * @param lat Latitude, expressed as degrees * 1E7 where image was taken
 * @param lon Longitude, expressed as degrees * 1E7 where capture was taken
 * @param alt Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
 * @param relative_alt Altitude above ground in meters, expressed as * 1E3 where image was taken
 * @param q Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
 * @param image_index Zero based index of this image (image count since armed -1)
 * @param capture_result Boolean indicating success (1) or failure (0) while capturing this image.
 * @param file_url URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_camera_image_captured_send(mace_channel_t chan, uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float *q, int32_t image_index, int8_t capture_result, const char *file_url)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN];
    _mace_put_uint64_t(buf, 0, time_utc);
    _mace_put_uint32_t(buf, 8, time_boot_ms);
    _mace_put_int32_t(buf, 12, lat);
    _mace_put_int32_t(buf, 16, lon);
    _mace_put_int32_t(buf, 20, alt);
    _mace_put_int32_t(buf, 24, relative_alt);
    _mace_put_int32_t(buf, 44, image_index);
    _mace_put_uint8_t(buf, 48, camera_id);
    _mace_put_int8_t(buf, 49, capture_result);
    _mace_put_float_array(buf, 28, q, 4);
    _mace_put_char_array(buf, 50, file_url, 205);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED, buf, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
#else
    mace_camera_image_captured_t packet;
    packet.time_utc = time_utc;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.image_index = image_index;
    packet.camera_id = camera_id;
    packet.capture_result = capture_result;
    mace_array_memcpy(packet.q, q, sizeof(float)*4);
    mace_array_memcpy(packet.file_url, file_url, sizeof(char)*205);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED, (const char *)&packet, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
#endif
}

/**
 * @brief Send a camera_image_captured message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_camera_image_captured_send_struct(mace_channel_t chan, const mace_camera_image_captured_t* camera_image_captured)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_camera_image_captured_send(chan, camera_image_captured->time_boot_ms, camera_image_captured->time_utc, camera_image_captured->camera_id, camera_image_captured->lat, camera_image_captured->lon, camera_image_captured->alt, camera_image_captured->relative_alt, camera_image_captured->q, camera_image_captured->image_index, camera_image_captured->capture_result, camera_image_captured->file_url);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED, (const char *)camera_image_captured, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
#endif
}

#if MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_camera_image_captured_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float *q, int32_t image_index, int8_t capture_result, const char *file_url)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, time_utc);
    _mace_put_uint32_t(buf, 8, time_boot_ms);
    _mace_put_int32_t(buf, 12, lat);
    _mace_put_int32_t(buf, 16, lon);
    _mace_put_int32_t(buf, 20, alt);
    _mace_put_int32_t(buf, 24, relative_alt);
    _mace_put_int32_t(buf, 44, image_index);
    _mace_put_uint8_t(buf, 48, camera_id);
    _mace_put_int8_t(buf, 49, capture_result);
    _mace_put_float_array(buf, 28, q, 4);
    _mace_put_char_array(buf, 50, file_url, 205);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED, buf, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
#else
    mace_camera_image_captured_t *packet = (mace_camera_image_captured_t *)msgbuf;
    packet->time_utc = time_utc;
    packet->time_boot_ms = time_boot_ms;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->relative_alt = relative_alt;
    packet->image_index = image_index;
    packet->camera_id = camera_id;
    packet->capture_result = capture_result;
    mace_array_memcpy(packet->q, q, sizeof(float)*4);
    mace_array_memcpy(packet->file_url, file_url, sizeof(char)*205);
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED, (const char *)packet, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_IMAGE_CAPTURED UNPACKING


/**
 * @brief Get field time_boot_ms from camera_image_captured message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mace_msg_camera_image_captured_get_time_boot_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field time_utc from camera_image_captured message
 *
 * @return Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
 */
static inline uint64_t mace_msg_camera_image_captured_get_time_utc(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field camera_id from camera_image_captured message
 *
 * @return Camera ID if there are multiple
 */
static inline uint8_t mace_msg_camera_image_captured_get_camera_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field lat from camera_image_captured message
 *
 * @return Latitude, expressed as degrees * 1E7 where image was taken
 */
static inline int32_t mace_msg_camera_image_captured_get_lat(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field lon from camera_image_captured message
 *
 * @return Longitude, expressed as degrees * 1E7 where capture was taken
 */
static inline int32_t mace_msg_camera_image_captured_get_lon(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field alt from camera_image_captured message
 *
 * @return Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
 */
static inline int32_t mace_msg_camera_image_captured_get_alt(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field relative_alt from camera_image_captured message
 *
 * @return Altitude above ground in meters, expressed as * 1E3 where image was taken
 */
static inline int32_t mace_msg_camera_image_captured_get_relative_alt(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field q from camera_image_captured message
 *
 * @return Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
 */
static inline uint16_t mace_msg_camera_image_captured_get_q(const mace_message_t* msg, float *q)
{
    return _MACE_RETURN_float_array(msg, q, 4,  28);
}

/**
 * @brief Get field image_index from camera_image_captured message
 *
 * @return Zero based index of this image (image count since armed -1)
 */
static inline int32_t mace_msg_camera_image_captured_get_image_index(const mace_message_t* msg)
{
    return _MACE_RETURN_int32_t(msg,  44);
}

/**
 * @brief Get field capture_result from camera_image_captured message
 *
 * @return Boolean indicating success (1) or failure (0) while capturing this image.
 */
static inline int8_t mace_msg_camera_image_captured_get_capture_result(const mace_message_t* msg)
{
    return _MACE_RETURN_int8_t(msg,  49);
}

/**
 * @brief Get field file_url from camera_image_captured message
 *
 * @return URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
 */
static inline uint16_t mace_msg_camera_image_captured_get_file_url(const mace_message_t* msg, char *file_url)
{
    return _MACE_RETURN_char_array(msg, file_url, 205,  50);
}

/**
 * @brief Decode a camera_image_captured message into a struct
 *
 * @param msg The message to decode
 * @param camera_image_captured C-struct to decode the message contents into
 */
static inline void mace_msg_camera_image_captured_decode(const mace_message_t* msg, mace_camera_image_captured_t* camera_image_captured)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    camera_image_captured->time_utc = mace_msg_camera_image_captured_get_time_utc(msg);
    camera_image_captured->time_boot_ms = mace_msg_camera_image_captured_get_time_boot_ms(msg);
    camera_image_captured->lat = mace_msg_camera_image_captured_get_lat(msg);
    camera_image_captured->lon = mace_msg_camera_image_captured_get_lon(msg);
    camera_image_captured->alt = mace_msg_camera_image_captured_get_alt(msg);
    camera_image_captured->relative_alt = mace_msg_camera_image_captured_get_relative_alt(msg);
    mace_msg_camera_image_captured_get_q(msg, camera_image_captured->q);
    camera_image_captured->image_index = mace_msg_camera_image_captured_get_image_index(msg);
    camera_image_captured->camera_id = mace_msg_camera_image_captured_get_camera_id(msg);
    camera_image_captured->capture_result = mace_msg_camera_image_captured_get_capture_result(msg);
    mace_msg_camera_image_captured_get_file_url(msg, camera_image_captured->file_url);
#else
        uint8_t len = msg->len < MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN? msg->len : MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN;
        memset(camera_image_captured, 0, MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN);
    memcpy(camera_image_captured, _MACE_PAYLOAD(msg), len);
#endif
}
