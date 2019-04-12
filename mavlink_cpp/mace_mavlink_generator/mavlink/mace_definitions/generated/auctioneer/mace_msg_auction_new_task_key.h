#pragma once
// MESSAGE AUCTION_NEW_TASK_KEY PACKING

#define MACE_MSG_ID_AUCTION_NEW_TASK_KEY 10005

MACEPACKED(
typedef struct __mace_auction_new_task_key_t {
 uint64_t creatorID; /*< Task creator ID*/
 double taskGenTime; /*< Task generation time*/
 uint32_t taskID; /*< Creator local task ID*/
 uint8_t type; /*< Task type*/
}) mace_auction_new_task_key_t;

#define MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN 21
#define MACE_MSG_ID_AUCTION_NEW_TASK_KEY_MIN_LEN 21
#define MACE_MSG_ID_10005_LEN 21
#define MACE_MSG_ID_10005_MIN_LEN 21

#define MACE_MSG_ID_AUCTION_NEW_TASK_KEY_CRC 239
#define MACE_MSG_ID_10005_CRC 239



#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_AUCTION_NEW_TASK_KEY { \
    10005, \
    "AUCTION_NEW_TASK_KEY", \
    4, \
    {  { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_new_task_key_t, creatorID) }, \
         { "taskGenTime", NULL, MACE_TYPE_DOUBLE, 0, 8, offsetof(mace_auction_new_task_key_t, taskGenTime) }, \
         { "taskID", NULL, MACE_TYPE_UINT32_T, 0, 16, offsetof(mace_auction_new_task_key_t, taskID) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 20, offsetof(mace_auction_new_task_key_t, type) }, \
         } \
}
#else
#define MACE_MESSAGE_INFO_AUCTION_NEW_TASK_KEY { \
    "AUCTION_NEW_TASK_KEY", \
    4, \
    {  { "creatorID", NULL, MACE_TYPE_UINT64_T, 0, 0, offsetof(mace_auction_new_task_key_t, creatorID) }, \
         { "taskGenTime", NULL, MACE_TYPE_DOUBLE, 0, 8, offsetof(mace_auction_new_task_key_t, taskGenTime) }, \
         { "taskID", NULL, MACE_TYPE_UINT32_T, 0, 16, offsetof(mace_auction_new_task_key_t, taskID) }, \
         { "type", NULL, MACE_TYPE_UINT8_T, 0, 20, offsetof(mace_auction_new_task_key_t, type) }, \
         } \
}
#endif

/**
 * @brief Pack a auction_new_task_key message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param taskGenTime Task generation time
 * @param type Task type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_new_task_key_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                               uint64_t creatorID, uint32_t taskID, double taskGenTime, uint8_t type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN];
    _mace_put_uint64_t(buf, 0, creatorID);
    _mace_put_double(buf, 8, taskGenTime);
    _mace_put_uint32_t(buf, 16, taskID);
    _mace_put_uint8_t(buf, 20, type);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN);
#else
    mace_auction_new_task_key_t packet;
    packet.creatorID = creatorID;
    packet.taskGenTime = taskGenTime;
    packet.taskID = taskID;
    packet.type = type;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_NEW_TASK_KEY;
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_MIN_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_CRC);
}

/**
 * @brief Pack a auction_new_task_key message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param taskGenTime Task generation time
 * @param type Task type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_auction_new_task_key_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   uint64_t creatorID,uint32_t taskID,double taskGenTime,uint8_t type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN];
    _mace_put_uint64_t(buf, 0, creatorID);
    _mace_put_double(buf, 8, taskGenTime);
    _mace_put_uint32_t(buf, 16, taskID);
    _mace_put_uint8_t(buf, 20, type);

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN);
#else
    mace_auction_new_task_key_t packet;
    packet.creatorID = creatorID;
    packet.taskGenTime = taskGenTime;
    packet.taskID = taskID;
    packet.type = type;

        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN);
#endif

    msg->msgid = MACE_MSG_ID_AUCTION_NEW_TASK_KEY;
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_MIN_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_CRC);
}

/**
 * @brief Encode a auction_new_task_key struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auction_new_task_key C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_new_task_key_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_auction_new_task_key_t* auction_new_task_key)
{
    return mace_msg_auction_new_task_key_pack(system_id, component_id, msg, auction_new_task_key->creatorID, auction_new_task_key->taskID, auction_new_task_key->taskGenTime, auction_new_task_key->type);
}

/**
 * @brief Encode a auction_new_task_key struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param auction_new_task_key C-struct to read the message contents from
 */
static inline uint16_t mace_msg_auction_new_task_key_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_auction_new_task_key_t* auction_new_task_key)
{
    return mace_msg_auction_new_task_key_pack_chan(system_id, component_id, chan, msg, auction_new_task_key->creatorID, auction_new_task_key->taskID, auction_new_task_key->taskGenTime, auction_new_task_key->type);
}

/**
 * @brief Send a auction_new_task_key message
 * @param chan MAVLink channel to send the message
 *
 * @param creatorID Task creator ID
 * @param taskID Creator local task ID
 * @param taskGenTime Task generation time
 * @param type Task type
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_auction_new_task_key_send(mace_channel_t chan, uint64_t creatorID, uint32_t taskID, double taskGenTime, uint8_t type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN];
    _mace_put_uint64_t(buf, 0, creatorID);
    _mace_put_double(buf, 8, taskGenTime);
    _mace_put_uint32_t(buf, 16, taskID);
    _mace_put_uint8_t(buf, 20, type);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_NEW_TASK_KEY, buf, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_MIN_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_CRC);
#else
    mace_auction_new_task_key_t packet;
    packet.creatorID = creatorID;
    packet.taskGenTime = taskGenTime;
    packet.taskID = taskID;
    packet.type = type;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_NEW_TASK_KEY, (const char *)&packet, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_MIN_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_CRC);
#endif
}

/**
 * @brief Send a auction_new_task_key message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_auction_new_task_key_send_struct(mace_channel_t chan, const mace_auction_new_task_key_t* auction_new_task_key)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_auction_new_task_key_send(chan, auction_new_task_key->creatorID, auction_new_task_key->taskID, auction_new_task_key->taskGenTime, auction_new_task_key->type);
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_NEW_TASK_KEY, (const char *)auction_new_task_key, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_MIN_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_CRC);
#endif
}

#if MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_auction_new_task_key_send_buf(mace_message_t *msgbuf, mace_channel_t chan,  uint64_t creatorID, uint32_t taskID, double taskGenTime, uint8_t type)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mace_put_uint64_t(buf, 0, creatorID);
    _mace_put_double(buf, 8, taskGenTime);
    _mace_put_uint32_t(buf, 16, taskID);
    _mace_put_uint8_t(buf, 20, type);

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_NEW_TASK_KEY, buf, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_MIN_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_CRC);
#else
    mace_auction_new_task_key_t *packet = (mace_auction_new_task_key_t *)msgbuf;
    packet->creatorID = creatorID;
    packet->taskGenTime = taskGenTime;
    packet->taskID = taskID;
    packet->type = type;

    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_AUCTION_NEW_TASK_KEY, (const char *)packet, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_MIN_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_CRC);
#endif
}
#endif

#endif

// MESSAGE AUCTION_NEW_TASK_KEY UNPACKING


/**
 * @brief Get field creatorID from auction_new_task_key message
 *
 * @return Task creator ID
 */
static inline uint64_t mace_msg_auction_new_task_key_get_creatorID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field taskID from auction_new_task_key message
 *
 * @return Creator local task ID
 */
static inline uint32_t mace_msg_auction_new_task_key_get_taskID(const mace_message_t* msg)
{
    return _MACE_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field taskGenTime from auction_new_task_key message
 *
 * @return Task generation time
 */
static inline double mace_msg_auction_new_task_key_get_taskGenTime(const mace_message_t* msg)
{
    return _MACE_RETURN_double(msg,  8);
}

/**
 * @brief Get field type from auction_new_task_key message
 *
 * @return Task type
 */
static inline uint8_t mace_msg_auction_new_task_key_get_type(const mace_message_t* msg)
{
    return _MACE_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a auction_new_task_key message into a struct
 *
 * @param msg The message to decode
 * @param auction_new_task_key C-struct to decode the message contents into
 */
static inline void mace_msg_auction_new_task_key_decode(const mace_message_t* msg, mace_auction_new_task_key_t* auction_new_task_key)
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    auction_new_task_key->creatorID = mace_msg_auction_new_task_key_get_creatorID(msg);
    auction_new_task_key->taskGenTime = mace_msg_auction_new_task_key_get_taskGenTime(msg);
    auction_new_task_key->taskID = mace_msg_auction_new_task_key_get_taskID(msg);
    auction_new_task_key->type = mace_msg_auction_new_task_key_get_type(msg);
#else
        uint8_t len = msg->len < MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN? msg->len : MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN;
        memset(auction_new_task_key, 0, MACE_MSG_ID_AUCTION_NEW_TASK_KEY_LEN);
    memcpy(auction_new_task_key, _MACE_PAYLOAD(msg), len);
#endif
}
