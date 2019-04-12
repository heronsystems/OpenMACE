#pragma once
// MESSAGE CAMERA_SETTINGS PACKING

#define MACE_MSG_ID_CAMERA_SETTINGS 260

MACEPACKED(
typedef struct __mace_camera_settings_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float aperture; /*< Aperture is 1/value*/
 float shutter_speed; /*< Shutter speed in s*/
 float iso_sensitivity; /*< ISO sensitivity*/
 float white_balance; /*< Color temperature in degrees Kelvin*/
 uint8_t camera_id; /*< Camera ID if there are multiple*/
 uint8_t aperture_locked; /*< Aperture locked (0: auto, 1: locked)*/
 uint8_t shutter_speed_locked; /*< Shutter speed locked (0: auto, 1: locked)*/
 uint8_t iso_sensitivity_locked; /*< ISO sensitivity locked (0: auto, 1: locked)*/
 uint8_t white_balance_locked; /*< Color temperature locked (0: auto, 1: locked)*/
 uint8_t mode_id; /*< Reserved for a camera mode ID*/
 uint8_t color_mode_id; /*< Reserved for a color mode ID*/
 uint8_t image_format_id; /*< Reserved for image format ID*/
}) mace_camera_settings_t;

#define MACE_MSG_ID_CAMERA_SETTINGS_LEN 28
#define MACE_MSG_ID_CAMERA_SETTINGS_MIN_LEN 28
#define MACE_MSG_ID_260_LEN 28
#define MACE_MSG_ID_260_MIN_LEN 28

#define MACE_MSG_ID_CAMERA_SETTINGS_CRC 8
#define MACE_MSG_ID_260_CRC 8



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_CAMERA_SETTINGS { \
    260, \
    "CAMERA_SETTINGS", \
    13, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_camera_settings_t, time_boot_ms) }, \
         { "aperture", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_camera_settings_t, aperture) }, \
         { "shutter_speed", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_camera_settings_t, shutter_speed) }, \
         { "iso_sensitivity", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_camera_settings_t, iso_sensitivity) }, \
         { "white_balance", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_camera_settings_t, white_balance) }, \
         { "camera_id", NULL, MACE_TYPE_UINT8_T, 0, 20, offsetof(mace_camera_settings_t, camera_id) }, \
         { "aperture_locked", NULL, MACE_TYPE_UINT8_T, 0, 21, offsetof(mace_camera_settings_t, aperture_locked) }, \
         { "shutter_speed_locked", NULL, MACE_TYPE_UINT8_T, 0, 22, offsetof(mace_camera_settings_t, shutter_speed_locked) }, \
         { "iso_sensitivity_locked", NULL, MACE_TYPE_UINT8_T, 0, 23, offsetof(mace_camera_settings_t, iso_sensitivity_locked) }, \
         { "white_balance_locked", NULL, MACE_TYPE_UINT8_T, 0, 24, offsetof(mace_camera_settings_t, white_balance_locked) }, \
         { "mode_id", NULL, MACE_TYPE_UINT8_T, 0, 25, offsetof(mace_camera_settings_t, mode_id) }, \
         { "color_mode_id", NULL, MACE_TYPE_UINT8_T, 0, 26, offsetof(mace_camera_settings_t, color_mode_id) }, \
         { "image_format_id", NULL, MACE_TYPE_UINT8_T, 0, 27, offsetof(mace_camera_settings_t, image_format_id) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_CAMERA_SETTINGS { \
    "CAMERA_SETTINGS", \
    13, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_camera_settings_t, time_boot_ms) }, \
         { "aperture", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_camera_settings_t, aperture) }, \
         { "shutter_speed", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_camera_settings_t, shutter_speed) }, \
         { "iso_sensitivity", NULL, MACE_TYPE_FLOAT, 0, 12, offsetof(mace_camera_settings_t, iso_sensitivity) }, \
         { "white_balance", NULL, MACE_TYPE_FLOAT, 0, 16, offsetof(mace_camera_settings_t, white_balance) }, \
         { "camera_id", NULL, MACE_TYPE_UINT8_T, 0, 20, offsetof(mace_camera_settings_t, camera_id) }, \
         { "aperture_locked", NULL, MACE_TYPE_UINT8_T, 0, 21, offsetof(mace_camera_settings_t, aperture_locked) }, \
         { "shutter_speed_locked", NULL, MACE_TYPE_UINT8_T, 0, 22, offsetof(mace_camera_settings_t, shutter_speed_locked) }, \
         { "iso_sensitivity_locked", NULL, MACE_TYPE_UINT8_T, 0, 23, offsetof(mace_camera_settings_t, iso_sensitivity_locked) }, \
         { "white_balance_locked", NULL, MACE_TYPE_UINT8_T, 0, 24, offsetof(mace_camera_settings_t, white_balance_locked) }, \
         { "mode_id", NULL, MACE_TYPE_UINT8_T, 0, 25, offsetof(mace_camera_settings_t, mode_id) }, \
         { "color_mode_id", NULL, MACE_TYPE_UINT8_T, 0, 26, offsetof(mace_camera_settings_t, color_mode_id) }, \
         { "image_format_id", NULL, MACE_TYPE_UINT8_T, 0, 27, offsetof(mace_camera_settings_t, image_format_id) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_settings message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param camera_id Camera ID if there are multiple
 * @param aperture Aperture is 1/value
 * @param aperture_locked Aperture locked (0: auto, 1: locked)
 * @param shutter_speed Shutter speed in s
 * @param shutter_speed_locked Shutter speed locked (0: auto, 1: locked)
 * @param iso_sensitivity ISO sensitivity
 * @param iso_sensitivity_locked ISO sensitivity locked (0: auto, 1: locked)
 * @param white_balance Color temperature in degrees Kelvin
 * @param white_balance_locked Color temperature locked (0: auto, 1: locked)
 * @param mode_id Reserved for a camera mode ID
 * @param color_mode_id Reserved for a color mode ID
 * @param image_format_id Reserved for image format ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_camera_settings_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint32_t time_boot_ms, uint8_t camera_id, float aperture, uint8_t aperture_locked, float shutter_speed, uint8_t shutter_speed_locked, float iso_sensitivity, uint8_t iso_sensitivity_locked, float white_balance, uint8_t white_balance_locked, uint8_t mode_id, uint8_t color_mode_id, uint8_t image_format_id)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_CAMERA_SETTINGS_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, aperture);
    _mace_put_float(buf, 8, shutter_speed);
    _mace_put_float(buf, 12, iso_sensitivity);
    _mace_put_float(buf, 16, white_balance);
    _mace_put_uint8_t(buf, 20, camera_id);
    _mace_put_uint8_t(buf, 21, aperture_locked);
    _mace_put_uint8_t(buf, 22, shutter_speed_locked);
    _mace_put_uint8_t(buf, 23, iso_sensitivity_locked);
    _mace_put_uint8_t(buf, 24, white_balance_locked);
    _mace_put_uint8_t(buf, 25, mode_id);
    _mace_put_uint8_t(buf, 26, color_mode_id);
    _mace_put_uint8_t(buf, 27, image_format_id);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_CAMERA_SETTINGS_LEN);
#else
    mace_camera_settings_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.aperture = aperture;
    packet.shutter_speed = shutter_speed;
    packet.iso_sensitivity = iso_sensitivity;
    packet.white_balance = white_balance;
    packet.camera_id = camera_id;
    packet.aperture_locked = aperture_locked;
    packet.shutter_speed_locked = shutter_speed_locked;
    packet.iso_sensitivity_locked = iso_sensitivity_locked;
    packet.white_balance_locked = white_balance_locked;
    packet.mode_id = mode_id;
    packet.color_mode_id = color_mode_id;
    packet.image_format_id = image_format_id;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_CAMERA_SETTINGS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_CAMERA_SETTINGS;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MACE_MSG_ID_CAMERA_SETTINGS_LEN, MACE_MSG_ID_CAMERA_SETTINGS_CRC);
}

/**
 * @brief Pack a camera_settings message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param camera_id Camera ID if there are multiple
 * @param aperture Aperture is 1/value
 * @param aperture_locked Aperture locked (0: auto, 1: locked)
 * @param shutter_speed Shutter speed in s
 * @param shutter_speed_locked Shutter speed locked (0: auto, 1: locked)
 * @param iso_sensitivity ISO sensitivity
 * @param iso_sensitivity_locked ISO sensitivity locked (0: auto, 1: locked)
 * @param white_balance Color temperature in degrees Kelvin
 * @param white_balance_locked Color temperature locked (0: auto, 1: locked)
 * @param mode_id Reserved for a camera mode ID
 * @param color_mode_id Reserved for a color mode ID
 * @param image_format_id Reserved for image format ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_camera_settings_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t camera_id,float aperture,uint8_t aperture_locked,float shutter_speed,uint8_t shutter_speed_locked,float iso_sensitivity,uint8_t iso_sensitivity_locked,float white_balance,uint8_t white_balance_locked,uint8_t mode_id,uint8_t color_mode_id,uint8_t image_format_id)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_CAMERA_SETTINGS_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, aperture);
    _mace_put_float(buf, 8, shutter_speed);
    _mace_put_float(buf, 12, iso_sensitivity);
    _mace_put_float(buf, 16, white_balance);
    _mace_put_uint8_t(buf, 20, camera_id);
    _mace_put_uint8_t(buf, 21, aperture_locked);
    _mace_put_uint8_t(buf, 22, shutter_speed_locked);
    _mace_put_uint8_t(buf, 23, iso_sensitivity_locked);
    _mace_put_uint8_t(buf, 24, white_balance_locked);
    _mace_put_uint8_t(buf, 25, mode_id);
    _mace_put_uint8_t(buf, 26, color_mode_id);
    _mace_put_uint8_t(buf, 27, image_format_id);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_CAMERA_SETTINGS_LEN);
#else
    mace_camera_settings_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.aperture = aperture;
    packet.shutter_speed = shutter_speed;
    packet.iso_sensitivity = iso_sensitivity;
    packet.white_balance = white_balance;
    packet.camera_id = camera_id;
    packet.aperture_locked = aperture_locked;
    packet.shutter_speed_locked = shutter_speed_locked;
    packet.iso_sensitivity_locked = iso_sensitivity_locked;
    packet.white_balance_locked = white_balance_locked;
    packet.mode_id = mode_id;
    packet.color_mode_id = color_mode_id;
    packet.image_format_id = image_format_id;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_CAMERA_SETTINGS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_CAMERA_SETTINGS;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MACE_MSG_ID_CAMERA_SETTINGS_LEN, MACE_MSG_ID_CAMERA_SETTINGS_CRC);
}

/**
 * @brief Encode a camera_settings struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_settings C-struct to read the message contents from
 */
static inline uint16_t mace_msg_camera_settings_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_camera_settings_t* camera_settings)
{
    return mace_msg_camera_settings_pack(system_id, component_id, msg, camera_settings->time_boot_ms, camera_settings->camera_id, camera_settings->aperture, camera_settings->aperture_locked, camera_settings->shutter_speed, camera_settings->shutter_speed_locked, camera_settings->iso_sensitivity, camera_settings->iso_sensitivity_locked, camera_settings->white_balance, camera_settings->white_balance_locked, camera_settings->mode_id, camera_settings->color_mode_id, camera_settings->image_format_id);
}

/**
 * @brief Encode a camera_settings struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_settings C-struct to read the message contents from
 */
static inline uint16_t mace_msg_camera_settings_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_camera_settings_t* camera_settings)
{
    return mace_msg_camera_settings_pack_chan(system_id, component_id, chan, msg, camera_settings->time_boot_ms, camera_settings->camera_id, camera_settings->aperture, camera_settings->aperture_locked, camera_settings->shutter_speed, camera_settings->shutter_speed_locked, camera_settings->iso_sensitivity, camera_settings->iso_sensitivity_locked, camera_settings->white_balance, camera_settings->white_balance_locked, camera_settings->mode_id, camera_settings->color_mode_id, camera_settings->image_format_id);
}

/**
 * @brief Send a camera_settings message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param camera_id Camera ID if there are multiple
 * @param aperture Aperture is 1/value
 * @param aperture_locked Aperture locked (0: auto, 1: locked)
 * @param shutter_speed Shutter speed in s
 * @param shutter_speed_locked Shutter speed locked (0: auto, 1: locked)
 * @param iso_sensitivity ISO sensitivity
 * @param iso_sensitivity_locked ISO sensitivity locked (0: auto, 1: locked)
 * @param white_balance Color temperature in degrees Kelvin
 * @param white_balance_locked Color temperature locked (0: auto, 1: locked)
 * @param mode_id Reserved for a camera mode ID
 * @param color_mode_id Reserved for a color mode ID
 * @param image_format_id Reserved for image format ID
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_camera_settings_send(mace_channel_t chan, uint32_t time_boot_ms, uint8_t camera_id, float aperture, uint8_t aperture_locked, float shutter_speed, uint8_t shutter_speed_locked, float iso_sensitivity, uint8_t iso_sensitivity_locked, float white_balance, uint8_t white_balance_locked, uint8_t mode_id, uint8_t color_mode_id, uint8_t image_format_id)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_CAMERA_SETTINGS_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, aperture);
    _mace_put_float(buf, 8, shutter_speed);
    _mace_put_float(buf, 12, iso_sensitivity);
    _mace_put_float(buf, 16, white_balance);
    _mace_put_uint8_t(buf, 20, camera_id);
    _mace_put_uint8_t(buf, 21, aperture_locked);
    _mace_put_uint8_t(buf, 22, shutter_speed_locked);
    _mace_put_uint8_t(buf, 23, iso_sensitivity_locked);
    _mace_put_uint8_t(buf, 24, white_balance_locked);
    _mace_put_uint8_t(buf, 25, mode_id);
    _mace_put_uint8_t(buf, 26, color_mode_id);
    _mace_put_uint8_t(buf, 27, image_format_id);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_SETTINGS, buf, MACE_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MACE_MSG_ID_CAMERA_SETTINGS_LEN, MACE_MSG_ID_CAMERA_SETTINGS_CRC);
#else
    mace_camera_settings_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.aperture = aperture;
    packet.shutter_speed = shutter_speed;
    packet.iso_sensitivity = iso_sensitivity;
    packet.white_balance = white_balance;
    packet.camera_id = camera_id;
    packet.aperture_locked = aperture_locked;
    packet.shutter_speed_locked = shutter_speed_locked;
    packet.iso_sensitivity_locked = iso_sensitivity_locked;
    packet.white_balance_locked = white_balance_locked;
    packet.mode_id = mode_id;
    packet.color_mode_id = color_mode_id;
    packet.image_format_id = image_format_id;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_SETTINGS, (const char *)&packet, MACE_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MACE_MSG_ID_CAMERA_SETTINGS_LEN, MACE_MSG_ID_CAMERA_SETTINGS_CRC);
#endif
}

/**
 * @brief Send a camera_settings message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_camera_settings_send_struct(mace_channel_t chan, const mace_camera_settings_t* camera_settings)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_camera_settings_send(chan, camera_settings->time_boot_ms, camera_settings->camera_id, camera_settings->aperture, camera_settings->aperture_locked, camera_settings->shutter_speed, camera_settings->shutter_speed_locked, camera_settings->iso_sensitivity, camera_settings->iso_sensitivity_locked, camera_settings->white_balance, camera_settings->white_balance_locked, camera_settings->mode_id, camera_settings->color_mode_id, camera_settings->image_format_id);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_SETTINGS, (const char *)camera_settings, MACE_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MACE_MSG_ID_CAMERA_SETTINGS_LEN, MACE_MSG_ID_CAMERA_SETTINGS_CRC);
#endif
}

#if MACE_MSG_ID_CAMERA_SETTINGS_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_camera_settings_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint32_t time_boot_ms, uint8_t camera_id, float aperture, uint8_t aperture_locked, float shutter_speed, uint8_t shutter_speed_locked, float iso_sensitivity, uint8_t iso_sensitivity_locked, float white_balance, uint8_t white_balance_locked, uint8_t mode_id, uint8_t color_mode_id, uint8_t image_format_id)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, aperture);
    _mace_put_float(buf, 8, shutter_speed);
    _mace_put_float(buf, 12, iso_sensitivity);
    _mace_put_float(buf, 16, white_balance);
    _mace_put_uint8_t(buf, 20, camera_id);
    _mace_put_uint8_t(buf, 21, aperture_locked);
    _mace_put_uint8_t(buf, 22, shutter_speed_locked);
    _mace_put_uint8_t(buf, 23, iso_sensitivity_locked);
    _mace_put_uint8_t(buf, 24, white_balance_locked);
    _mace_put_uint8_t(buf, 25, mode_id);
    _mace_put_uint8_t(buf, 26, color_mode_id);
    _mace_put_uint8_t(buf, 27, image_format_id);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_SETTINGS, buf, MACE_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MACE_MSG_ID_CAMERA_SETTINGS_LEN, MACE_MSG_ID_CAMERA_SETTINGS_CRC);
#else
    mace_camera_settings_t *packet = (mace_camera_settings_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->aperture = aperture;
    packet->shutter_speed = shutter_speed;
    packet->iso_sensitivity = iso_sensitivity;
    packet->white_balance = white_balance;
    packet->camera_id = camera_id;
    packet->aperture_locked = aperture_locked;
    packet->shutter_speed_locked = shutter_speed_locked;
    packet->iso_sensitivity_locked = iso_sensitivity_locked;
    packet->white_balance_locked = white_balance_locked;
    packet->mode_id = mode_id;
    packet->color_mode_id = color_mode_id;
    packet->image_format_id = image_format_id;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_CAMERA_SETTINGS, (const char *)packet, MACE_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MACE_MSG_ID_CAMERA_SETTINGS_LEN, MACE_MSG_ID_CAMERA_SETTINGS_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_SETTINGS UNPACKING


/**
 * @brief Get field time_boot_ms from camera_settings message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mace_msg_camera_settings_get_time_boot_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field camera_id from camera_settings message
 *
 * @return Camera ID if there are multiple
 */
static inline uint8_t mace_msg_camera_settings_get_camera_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field aperture from camera_settings message
 *
 * @return Aperture is 1/value
 */
static inline float mace_msg_camera_settings_get_aperture(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field aperture_locked from camera_settings message
 *
 * @return Aperture locked (0: auto, 1: locked)
 */
static inline uint8_t mace_msg_camera_settings_get_aperture_locked(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field shutter_speed from camera_settings message
 *
 * @return Shutter speed in s
 */
static inline float mace_msg_camera_settings_get_shutter_speed(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field shutter_speed_locked from camera_settings message
 *
 * @return Shutter speed locked (0: auto, 1: locked)
 */
static inline uint8_t mace_msg_camera_settings_get_shutter_speed_locked(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field iso_sensitivity from camera_settings message
 *
 * @return ISO sensitivity
 */
static inline float mace_msg_camera_settings_get_iso_sensitivity(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  12);
}

/**
 * @brief Get field iso_sensitivity_locked from camera_settings message
 *
 * @return ISO sensitivity locked (0: auto, 1: locked)
 */
static inline uint8_t mace_msg_camera_settings_get_iso_sensitivity_locked(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field white_balance from camera_settings message
 *
 * @return Color temperature in degrees Kelvin
 */
static inline float mace_msg_camera_settings_get_white_balance(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  16);
}

/**
 * @brief Get field white_balance_locked from camera_settings message
 *
 * @return Color temperature locked (0: auto, 1: locked)
 */
static inline uint8_t mace_msg_camera_settings_get_white_balance_locked(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field mode_id from camera_settings message
 *
 * @return Reserved for a camera mode ID
 */
static inline uint8_t mace_msg_camera_settings_get_mode_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field color_mode_id from camera_settings message
 *
 * @return Reserved for a color mode ID
 */
static inline uint8_t mace_msg_camera_settings_get_color_mode_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field image_format_id from camera_settings message
 *
 * @return Reserved for image format ID
 */
static inline uint8_t mace_msg_camera_settings_get_image_format_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Decode a camera_settings message into a struct
 *
 * @param msg The message to decode
 * @param camera_settings C-struct to decode the message contents into
 */
static inline void mace_msg_camera_settings_decode(const mace_message_t* msg, mace_camera_settings_t* camera_settings)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    camera_settings->time_boot_ms = mace_msg_camera_settings_get_time_boot_ms(msg);
    camera_settings->aperture = mace_msg_camera_settings_get_aperture(msg);
    camera_settings->shutter_speed = mace_msg_camera_settings_get_shutter_speed(msg);
    camera_settings->iso_sensitivity = mace_msg_camera_settings_get_iso_sensitivity(msg);
    camera_settings->white_balance = mace_msg_camera_settings_get_white_balance(msg);
    camera_settings->camera_id = mace_msg_camera_settings_get_camera_id(msg);
    camera_settings->aperture_locked = mace_msg_camera_settings_get_aperture_locked(msg);
    camera_settings->shutter_speed_locked = mace_msg_camera_settings_get_shutter_speed_locked(msg);
    camera_settings->iso_sensitivity_locked = mace_msg_camera_settings_get_iso_sensitivity_locked(msg);
    camera_settings->white_balance_locked = mace_msg_camera_settings_get_white_balance_locked(msg);
    camera_settings->mode_id = mace_msg_camera_settings_get_mode_id(msg);
    camera_settings->color_mode_id = mace_msg_camera_settings_get_color_mode_id(msg);
    camera_settings->image_format_id = mace_msg_camera_settings_get_image_format_id(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_CAMERA_SETTINGS_LEN? msg->len : MACE_MSG_ID_CAMERA_SETTINGS_LEN;
        memset(camera_settings, 0, MACE_MSG_ID_CAMERA_SETTINGS_LEN);
    memcpy(camera_settings, _MACE_PAYLOAD(msg), len);
#endif
}
