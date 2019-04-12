#pragma once
// MESSAGE BATTERY_STATUS PACKING

#define MACE_MSG_ID_BATTERY_STATUS 3

MACEPACKED(
typedef struct __mace_battery_status_t {
 int16_t temperature; /*< Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.*/
 uint16_t voltage_battery; /*< Battery voltage, in millivolts (1 = 1 millivolt)*/
 int16_t current_battery; /*< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current*/
 uint8_t id; /*< Battery ID*/
 uint8_t battery_function; /*< Function of the battery*/
 uint8_t type; /*< Type (chemistry) of the battery*/
 int8_t battery_remaining; /*< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery*/
}) mace_battery_status_t;

#define MACE_MSG_ID_BATTERY_STATUS_LEN 10
#define MACE_MSG_ID_BATTERY_STATUS_MIN_LEN 10
#define MACE_MSG_ID_3_LEN 10
#define MACE_MSG_ID_3_MIN_LEN 10

#define MACE_MSG_ID_BATTERY_STATUS_CRC 227
#define MACE_MSG_ID_3_CRC 227



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_BATTERY_STATUS { \
    3, \
    "BATTERY_STATUS", \
    7, \
    {  { "temperature", NULL, MACE_TYPE_INT16_T, 0, 0, offsetof(mace_battery_status_t, temperature) }, \
         { "voltage_battery", NULL, MACE_TYPE_UINT16_T, 0, 2, offsetof(mace_battery_status_t, voltage_battery) }, \
         { "current_battery", NULL, MACE_TYPE_INT16_T, 0, 4, offsetof(mace_battery_status_t, current_battery) }, \
         { "id", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_battery_status_t, id) }, \
         { "battery_function", NULL, MACE_TYPE_UINT8_T, 0, 7, offsetof(mace_battery_status_t, battery_function) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 8, offsetof(mace_battery_status_t, type) }, \
         { "battery_remaining", NULL, MACE_TYPE_INT8_T, 0, 9, offsetof(mace_battery_status_t, battery_remaining) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_BATTERY_STATUS { \
    "BATTERY_STATUS", \
    7, \
    {  { "temperature", NULL, MACE_TYPE_INT16_T, 0, 0, offsetof(mace_battery_status_t, temperature) }, \
         { "voltage_battery", NULL, MACE_TYPE_UINT16_T, 0, 2, offsetof(mace_battery_status_t, voltage_battery) }, \
         { "current_battery", NULL, MACE_TYPE_INT16_T, 0, 4, offsetof(mace_battery_status_t, current_battery) }, \
         { "id", NULL, MACE_TYPE_UINT8_T, 0, 6, offsetof(mace_battery_status_t, id) }, \
         { "battery_function", NULL, MACE_TYPE_UINT8_T, 0, 7, offsetof(mace_battery_status_t, battery_function) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 8, offsetof(mace_battery_status_t, type) }, \
         { "battery_remaining", NULL, MACE_TYPE_INT8_T, 0, 9, offsetof(mace_battery_status_t, battery_remaining) }, \
         } \
}
#endif

/**
 * @brief Pack a battery_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id Battery ID
 * @param battery_function Function of the battery
 * @param type Type (chemistry) of the battery
 * @param temperature Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
 * @param voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_battery_status_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BATTERY_STATUS_LEN];
    _mace_put_int16_t(buf, 0, temperature);
    _mace_put_uint16_t(buf, 2, voltage_battery);
    _mace_put_int16_t(buf, 4, current_battery);
    _mace_put_uint8_t(buf, 6, id);
    _mace_put_uint8_t(buf, 7, battery_function);
    _mace_put_uint8_t(buf, 8, type);
    _mace_put_int8_t(buf, 9, battery_remaining);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_BATTERY_STATUS_LEN);
#else
    mace_battery_status_t packet;
    packet.temperature = temperature;
    packet.voltage_battery = voltage_battery;
    packet.current_battery = current_battery;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.battery_remaining = battery_remaining;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_BATTERY_STATUS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_BATTERY_STATUS;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_BATTERY_STATUS_MIN_LEN, MACE_MSG_ID_BATTERY_STATUS_LEN, MACE_MSG_ID_BATTERY_STATUS_CRC);
}

/**
 * @brief Pack a battery_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id Battery ID
 * @param battery_function Function of the battery
 * @param type Type (chemistry) of the battery
 * @param temperature Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
 * @param voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_battery_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint8_t id,uint8_t battery_function,uint8_t type,int16_t temperature,uint16_t voltage_battery,int16_t current_battery,int8_t battery_remaining)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BATTERY_STATUS_LEN];
    _mace_put_int16_t(buf, 0, temperature);
    _mace_put_uint16_t(buf, 2, voltage_battery);
    _mace_put_int16_t(buf, 4, current_battery);
    _mace_put_uint8_t(buf, 6, id);
    _mace_put_uint8_t(buf, 7, battery_function);
    _mace_put_uint8_t(buf, 8, type);
    _mace_put_int8_t(buf, 9, battery_remaining);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_BATTERY_STATUS_LEN);
#else
    mace_battery_status_t packet;
    packet.temperature = temperature;
    packet.voltage_battery = voltage_battery;
    packet.current_battery = current_battery;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.battery_remaining = battery_remaining;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_BATTERY_STATUS_LEN);
#endif

    msg->msgid = MACE_MSG_ID_BATTERY_STATUS;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_BATTERY_STATUS_MIN_LEN, MACE_MSG_ID_BATTERY_STATUS_LEN, MACE_MSG_ID_BATTERY_STATUS_CRC);
}

/**
 * @brief Encode a battery_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param battery_status C-struct to read the message contents from
 */
static inline uint16_t mace_msg_battery_status_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_battery_status_t* battery_status)
{
    return mace_msg_battery_status_pack(system_id, component_id, msg, battery_status->id, battery_status->battery_function, battery_status->type, battery_status->temperature, battery_status->voltage_battery, battery_status->current_battery, battery_status->battery_remaining);
}

/**
 * @brief Encode a battery_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param battery_status C-struct to read the message contents from
 */
static inline uint16_t mace_msg_battery_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_battery_status_t* battery_status)
{
    return mace_msg_battery_status_pack_chan(system_id, component_id, chan, msg, battery_status->id, battery_status->battery_function, battery_status->type, battery_status->temperature, battery_status->voltage_battery, battery_status->current_battery, battery_status->battery_remaining);
}

/**
 * @brief Send a battery_status message
 * @param chan MAVLink channel to send the message
 *
 * @param id Battery ID
 * @param battery_function Function of the battery
 * @param type Type (chemistry) of the battery
 * @param temperature Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
 * @param voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_battery_status_send(mace_channel_t chan, uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_BATTERY_STATUS_LEN];
    _mace_put_int16_t(buf, 0, temperature);
    _mace_put_uint16_t(buf, 2, voltage_battery);
    _mace_put_int16_t(buf, 4, current_battery);
    _mace_put_uint8_t(buf, 6, id);
    _mace_put_uint8_t(buf, 7, battery_function);
    _mace_put_uint8_t(buf, 8, type);
    _mace_put_int8_t(buf, 9, battery_remaining);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BATTERY_STATUS, buf, MACE_MSG_ID_BATTERY_STATUS_MIN_LEN, MACE_MSG_ID_BATTERY_STATUS_LEN, MACE_MSG_ID_BATTERY_STATUS_CRC);
#else
    mace_battery_status_t packet;
    packet.temperature = temperature;
    packet.voltage_battery = voltage_battery;
    packet.current_battery = current_battery;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.battery_remaining = battery_remaining;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BATTERY_STATUS, (const char *)&packet, MACE_MSG_ID_BATTERY_STATUS_MIN_LEN, MACE_MSG_ID_BATTERY_STATUS_LEN, MACE_MSG_ID_BATTERY_STATUS_CRC);
#endif
}

/**
 * @brief Send a battery_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_battery_status_send_struct(mace_channel_t chan, const mace_battery_status_t* battery_status)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_battery_status_send(chan, battery_status->id, battery_status->battery_function, battery_status->type, battery_status->temperature, battery_status->voltage_battery, battery_status->current_battery, battery_status->battery_remaining);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BATTERY_STATUS, (const char *)battery_status, MACE_MSG_ID_BATTERY_STATUS_MIN_LEN, MACE_MSG_ID_BATTERY_STATUS_LEN, MACE_MSG_ID_BATTERY_STATUS_CRC);
#endif
}

#if MACE_MSG_ID_BATTERY_STATUS_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_battery_status_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_int16_t(buf, 0, temperature);
    _mace_put_uint16_t(buf, 2, voltage_battery);
    _mace_put_int16_t(buf, 4, current_battery);
    _mace_put_uint8_t(buf, 6, id);
    _mace_put_uint8_t(buf, 7, battery_function);
    _mace_put_uint8_t(buf, 8, type);
    _mace_put_int8_t(buf, 9, battery_remaining);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BATTERY_STATUS, buf, MACE_MSG_ID_BATTERY_STATUS_MIN_LEN, MACE_MSG_ID_BATTERY_STATUS_LEN, MACE_MSG_ID_BATTERY_STATUS_CRC);
#else
    mace_battery_status_t *packet = (mace_battery_status_t *)msgbuf;
    packet->temperature = temperature;
    packet->voltage_battery = voltage_battery;
    packet->current_battery = current_battery;
    packet->id = id;
    packet->battery_function = battery_function;
    packet->type = type;
    packet->battery_remaining = battery_remaining;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_BATTERY_STATUS, (const char *)packet, MACE_MSG_ID_BATTERY_STATUS_MIN_LEN, MACE_MSG_ID_BATTERY_STATUS_LEN, MACE_MSG_ID_BATTERY_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE BATTERY_STATUS UNPACKING


/**
 * @brief Get field id from battery_status message
 *
 * @return Battery ID
 */
static inline uint8_t mace_msg_battery_status_get_id(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field battery_function from battery_status message
 *
 * @return Function of the battery
 */
static inline uint8_t mace_msg_battery_status_get_battery_function(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field type from battery_status message
 *
 * @return Type (chemistry) of the battery
 */
static inline uint8_t mace_msg_battery_status_get_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field temperature from battery_status message
 *
 * @return Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
 */
static inline int16_t mace_msg_battery_status_get_temperature(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field voltage_battery from battery_status message
 *
 * @return Battery voltage, in millivolts (1 = 1 millivolt)
 */
static inline uint16_t mace_msg_battery_status_get_voltage_battery(const mace_message_t* msg)
{
    return _MACE_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field current_battery from battery_status message
 *
 * @return Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 */
static inline int16_t mace_msg_battery_status_get_current_battery(const mace_message_t* msg)
{
    return _MACE_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field battery_remaining from battery_status message
 *
 * @return Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
 */
static inline int8_t mace_msg_battery_status_get_battery_remaining(const mace_message_t* msg)
{
    return _MACE_RETURN_int8_t(msg,  9);
}

/**
 * @brief Decode a battery_status message into a struct
 *
 * @param msg The message to decode
 * @param battery_status C-struct to decode the message contents into
 */
static inline void mace_msg_battery_status_decode(const mace_message_t* msg, mace_battery_status_t* battery_status)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    battery_status->temperature = mace_msg_battery_status_get_temperature(msg);
    battery_status->voltage_battery = mace_msg_battery_status_get_voltage_battery(msg);
    battery_status->current_battery = mace_msg_battery_status_get_current_battery(msg);
    battery_status->id = mace_msg_battery_status_get_id(msg);
    battery_status->battery_function = mace_msg_battery_status_get_battery_function(msg);
    battery_status->type = mace_msg_battery_status_get_type(msg);
    battery_status->battery_remaining = mace_msg_battery_status_get_battery_remaining(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_BATTERY_STATUS_LEN? msg->len : MACE_MSG_ID_BATTERY_STATUS_LEN;
        memset(battery_status, 0, MACE_MSG_ID_BATTERY_STATUS_LEN);
    memcpy(battery_status, _MACE_PAYLOAD(msg), len);
#endif
}
