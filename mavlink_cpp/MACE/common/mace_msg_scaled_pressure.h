#pragma once
// MESSAGE SCALED_PRESSURE PACKING

#define MACE_MSG_ID_SCALED_PRESSURE 15

MACEPACKED(
typedef struct __mace_scaled_pressure_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float press_abs; /*< Absolute pressure (hectopascal)*/
 float press_diff; /*< Differential pressure 1 (hectopascal)*/
 int16_t temperature; /*< Temperature measurement (0.01 degrees celsius)*/
}) mace_scaled_pressure_t;

#define MACE_MSG_ID_SCALED_PRESSURE_LEN 14
#define MACE_MSG_ID_SCALED_PRESSURE_MIN_LEN 14
#define MACE_MSG_ID_15_LEN 14
#define MACE_MSG_ID_15_MIN_LEN 14

#define MACE_MSG_ID_SCALED_PRESSURE_CRC 115
#define MACE_MSG_ID_15_CRC 115



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_SCALED_PRESSURE { \
    15, \
    "SCALED_PRESSURE", \
    4, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_scaled_pressure_t, time_boot_ms) }, \
         { "press_abs", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_scaled_pressure_t, press_abs) }, \
         { "press_diff", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_scaled_pressure_t, press_diff) }, \
         { "temperature", NULL, MACE_TYPE_INT16_T, 0, 12, offsetof(mace_scaled_pressure_t, temperature) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_SCALED_PRESSURE { \
    "SCALED_PRESSURE", \
    4, \
    {  { "time_boot_ms", NULL, MACE_TYPE_UINT32_T, 0, 0, offsetof(mace_scaled_pressure_t, time_boot_ms) }, \
         { "press_abs", NULL, MACE_TYPE_FLOAT, 0, 4, offsetof(mace_scaled_pressure_t, press_abs) }, \
         { "press_diff", NULL, MACE_TYPE_FLOAT, 0, 8, offsetof(mace_scaled_pressure_t, press_diff) }, \
         { "temperature", NULL, MACE_TYPE_INT16_T, 0, 12, offsetof(mace_scaled_pressure_t, temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a scaled_pressure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_scaled_pressure_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_SCALED_PRESSURE_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, press_abs);
    _mace_put_float(buf, 8, press_diff);
    _mace_put_int16_t(buf, 12, temperature);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_SCALED_PRESSURE_LEN);
#else
    mace_scaled_pressure_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.press_abs = press_abs;
    packet.press_diff = press_diff;
    packet.temperature = temperature;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_SCALED_PRESSURE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_SCALED_PRESSURE;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_SCALED_PRESSURE_MIN_LEN, MACE_MSG_ID_SCALED_PRESSURE_LEN, MACE_MSG_ID_SCALED_PRESSURE_CRC);
}

/**
 * @brief Pack a scaled_pressure message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_scaled_pressure_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint32_t time_boot_ms,float press_abs,float press_diff,int16_t temperature)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_SCALED_PRESSURE_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, press_abs);
    _mace_put_float(buf, 8, press_diff);
    _mace_put_int16_t(buf, 12, temperature);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_SCALED_PRESSURE_LEN);
#else
    mace_scaled_pressure_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.press_abs = press_abs;
    packet.press_diff = press_diff;
    packet.temperature = temperature;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_SCALED_PRESSURE_LEN);
#endif

    msg->msgid = MACE_MSG_ID_SCALED_PRESSURE;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_SCALED_PRESSURE_MIN_LEN, MACE_MSG_ID_SCALED_PRESSURE_LEN, MACE_MSG_ID_SCALED_PRESSURE_CRC);
}

/**
 * @brief Encode a scaled_pressure struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param scaled_pressure C-struct to read the message contents from
 */
static inline uint16_t mace_msg_scaled_pressure_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_scaled_pressure_t* scaled_pressure)
{
    return mace_msg_scaled_pressure_pack(system_id, component_id, msg, scaled_pressure->time_boot_ms, scaled_pressure->press_abs, scaled_pressure->press_diff, scaled_pressure->temperature);
}

/**
 * @brief Encode a scaled_pressure struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param scaled_pressure C-struct to read the message contents from
 */
static inline uint16_t mace_msg_scaled_pressure_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_scaled_pressure_t* scaled_pressure)
{
    return mace_msg_scaled_pressure_pack_chan(system_id, component_id, chan, msg, scaled_pressure->time_boot_ms, scaled_pressure->press_abs, scaled_pressure->press_diff, scaled_pressure->temperature);
}

/**
 * @brief Send a scaled_pressure message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_scaled_pressure_send(mace_channel_t chan, uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_SCALED_PRESSURE_LEN];
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, press_abs);
    _mace_put_float(buf, 8, press_diff);
    _mace_put_int16_t(buf, 12, temperature);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SCALED_PRESSURE, buf, MACE_MSG_ID_SCALED_PRESSURE_MIN_LEN, MACE_MSG_ID_SCALED_PRESSURE_LEN, MACE_MSG_ID_SCALED_PRESSURE_CRC);
#else
    mace_scaled_pressure_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.press_abs = press_abs;
    packet.press_diff = press_diff;
    packet.temperature = temperature;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SCALED_PRESSURE, (const char *)&packet, MACE_MSG_ID_SCALED_PRESSURE_MIN_LEN, MACE_MSG_ID_SCALED_PRESSURE_LEN, MACE_MSG_ID_SCALED_PRESSURE_CRC);
#endif
}

/**
 * @brief Send a scaled_pressure message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_scaled_pressure_send_struct(mace_channel_t chan, const mace_scaled_pressure_t* scaled_pressure)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_scaled_pressure_send(chan, scaled_pressure->time_boot_ms, scaled_pressure->press_abs, scaled_pressure->press_diff, scaled_pressure->temperature);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SCALED_PRESSURE, (const char *)scaled_pressure, MACE_MSG_ID_SCALED_PRESSURE_MIN_LEN, MACE_MSG_ID_SCALED_PRESSURE_LEN, MACE_MSG_ID_SCALED_PRESSURE_CRC);
#endif
}

#if MACE_MSG_ID_SCALED_PRESSURE_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_scaled_pressure_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint32_t(buf, 0, time_boot_ms);
    _mace_put_float(buf, 4, press_abs);
    _mace_put_float(buf, 8, press_diff);
    _mace_put_int16_t(buf, 12, temperature);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SCALED_PRESSURE, buf, MACE_MSG_ID_SCALED_PRESSURE_MIN_LEN, MACE_MSG_ID_SCALED_PRESSURE_LEN, MACE_MSG_ID_SCALED_PRESSURE_CRC);
#else
    mace_scaled_pressure_t *packet = (mace_scaled_pressure_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->press_abs = press_abs;
    packet->press_diff = press_diff;
    packet->temperature = temperature;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_SCALED_PRESSURE, (const char *)packet, MACE_MSG_ID_SCALED_PRESSURE_MIN_LEN, MACE_MSG_ID_SCALED_PRESSURE_LEN, MACE_MSG_ID_SCALED_PRESSURE_CRC);
#endif
}
#endif

#endif

// MESSAGE SCALED_PRESSURE UNPACKING


/**
 * @brief Get field time_boot_ms from scaled_pressure message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mace_msg_scaled_pressure_get_time_boot_ms(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field press_abs from scaled_pressure message
 *
 * @return Absolute pressure (hectopascal)
 */
static inline float mace_msg_scaled_pressure_get_press_abs(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  4);
}

/**
 * @brief Get field press_diff from scaled_pressure message
 *
 * @return Differential pressure 1 (hectopascal)
 */
static inline float mace_msg_scaled_pressure_get_press_diff(const mace_message_t* msg)
{
    return _MACE_RETURN_float(msg,  8);
}

/**
 * @brief Get field temperature from scaled_pressure message
 *
 * @return Temperature measurement (0.01 degrees celsius)
 */
static inline int16_t mace_msg_scaled_pressure_get_temperature(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  12);
}

/**
 * @brief Decode a scaled_pressure message into a struct
 *
 * @param msg The message to decode
 * @param scaled_pressure C-struct to decode the message contents into
 */
static inline void mace_msg_scaled_pressure_decode(const mace_message_t* msg, mace_scaled_pressure_t* scaled_pressure)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    scaled_pressure->time_boot_ms = mace_msg_scaled_pressure_get_time_boot_ms(msg);
    scaled_pressure->press_abs = mace_msg_scaled_pressure_get_press_abs(msg);
    scaled_pressure->press_diff = mace_msg_scaled_pressure_get_press_diff(msg);
    scaled_pressure->temperature = mace_msg_scaled_pressure_get_temperature(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_SCALED_PRESSURE_LEN? msg->len : MACE_MSG_ID_SCALED_PRESSURE_LEN;
        memset(scaled_pressure, 0, MACE_MSG_ID_SCALED_PRESSURE_LEN);
    memcpy(scaled_pressure, _MACE_PAYLOAD(msg), len);
#endif
}
