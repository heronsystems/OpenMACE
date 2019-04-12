#pragma once

#include "string.h"
#include "mace_types.h"

/* 
   If you want MACELink on a system that is native big-endian,
   you need to define NATIVE_BIG_ENDIAN
*/
#ifdef NATIVE_BIG_ENDIAN
# define MACE_NEED_BYTE_SWAP (MACE_ENDIAN == MACE_LITTLE_ENDIAN)
#else
# define MACE_NEED_BYTE_SWAP (MACE_ENDIAN != MACE_LITTLE_ENDIAN)
#endif

#ifndef MACE_STACK_BUFFER
#define MACE_STACK_BUFFER 0
#endif

#ifndef MACE_AVOID_GCC_STACK_BUG
# define MACE_AVOID_GCC_STACK_BUG defined(__GNUC__)
#endif

#ifndef MACE_ASSERT
#define MACE_ASSERT(x)
#endif

#ifndef MACE_START_UART_SEND
#define MACE_START_UART_SEND(chan, length)
#endif

#ifndef MACE_END_UART_SEND
#define MACE_END_UART_SEND(chan, length)
#endif

/* option to provide alternative implementation of mace_helpers.h */
#ifdef MACE_SEPARATE_HELPERS

    #define MACE_HELPER

    /* decls in sync with those in mace_helpers.h */
    #ifndef MACE_GET_CHANNEL_STATUS
    MACE_HELPER mace_status_t* mace_get_channel_status(uint8_t chan);
    #endif
    MACE_HELPER void mace_reset_channel_status(uint8_t chan);
    MACE_HELPER uint16_t mace_finalize_message_chan(mace_message_t* msg, uint8_t system_id, uint8_t component_id,
                                                          uint8_t chan, uint8_t min_length, uint8_t length, uint8_t crc_extra);
    MACE_HELPER uint16_t mace_finalize_message(mace_message_t* msg, uint8_t system_id, uint8_t component_id,
                                                     uint8_t min_length, uint8_t length, uint8_t crc_extra);
    #ifdef MACE_USE_CONVENIENCE_FUNCTIONS
    MACE_HELPER void _mace_finalize_message_chan_send(mace_channel_t chan, uint32_t msgid, const char *packet,
                                                        uint8_t min_length, uint8_t length, uint8_t crc_extra);
    #endif
    MACE_HELPER uint16_t mace_msg_to_send_buffer(uint8_t *buffer, const mace_message_t *msg);
    MACE_HELPER void mace_start_checksum(mace_message_t* msg);
    MACE_HELPER void mace_update_checksum(mace_message_t* msg, uint8_t c);
    MACE_HELPER uint8_t mace_frame_char_buffer(mace_message_t* rxmsg, 
						     mace_status_t* status,
						     uint8_t c, 
						     mace_message_t* r_message, 
						     mace_status_t* r_mace_status);
    MACE_HELPER uint8_t mace_frame_char(uint8_t chan, uint8_t c, mace_message_t* r_message, mace_status_t* r_mace_status);
    MACE_HELPER uint8_t mace_parse_char(uint8_t chan, uint8_t c, mace_message_t* r_message, mace_status_t* r_mace_status);
    MACE_HELPER uint8_t put_bitfield_n_by_index(int32_t b, uint8_t bits, uint8_t packet_index, uint8_t bit_index,
                               uint8_t* r_bit_index, uint8_t* buffer);
    MACE_HELPER const mace_msg_entry_t *mace_get_msg_entry(uint32_t msgid);
    #ifdef MACE_USE_CONVENIENCE_FUNCTIONS
    MACE_HELPER void _mace_send_uart(mace_channel_t chan, const char *buf, uint16_t len);
    MACE_HELPER void _mace_resend_uart(mace_channel_t chan, const mace_message_t *msg);
    #endif

#else

    #define MACE_HELPER static inline
    #include "mace_helpers.h"

#endif // MACE_SEPARATE_HELPERS


/**
 * @brief Get the required buffer size for this message
 */
static inline uint16_t mace_msg_get_send_buffer_length(const mace_message_t* msg)
{
	if (msg->magic == MACE_STX_MACE1) {
		return msg->len + MACE_CORE_HEADER_MACE1_LEN+1 + 2;
	}
    	uint16_t signature_len = (msg->incompat_flags & MACE_IFLAG_SIGNED)?MACE_SIGNATURE_BLOCK_LEN:0;
	return msg->len + MACE_NUM_NON_PAYLOAD_BYTES + signature_len;
}

#if MACE_NEED_BYTE_SWAP
static inline void byte_swap_2(char *dst, const char *src)
{
	dst[0] = src[1];
	dst[1] = src[0];
}
static inline void byte_swap_4(char *dst, const char *src)
{
	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];
}
static inline void byte_swap_8(char *dst, const char *src)
{
	dst[0] = src[7];
	dst[1] = src[6];
	dst[2] = src[5];
	dst[3] = src[4];
	dst[4] = src[3];
	dst[5] = src[2];
	dst[6] = src[1];
	dst[7] = src[0];
}
#elif !MACE_ALIGNED_FIELDS
static inline void byte_copy_2(char *dst, const char *src)
{
	dst[0] = src[0];
	dst[1] = src[1];
}
static inline void byte_copy_4(char *dst, const char *src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
	dst[3] = src[3];
}
static inline void byte_copy_8(char *dst, const char *src)
{
	memcpy(dst, src, 8);
}
#endif

#define _mace_put_uint8_t(buf, wire_offset, b) buf[wire_offset] = (uint8_t)b
#define _mace_put_int8_t(buf, wire_offset, b)  buf[wire_offset] = (int8_t)b
#define _mace_put_char(buf, wire_offset, b)    buf[wire_offset] = b

#if MACE_NEED_BYTE_SWAP
#define _mace_put_uint16_t(buf, wire_offset, b) byte_swap_2(&buf[wire_offset], (const char *)&b)
#define _mace_put_int16_t(buf, wire_offset, b)  byte_swap_2(&buf[wire_offset], (const char *)&b)
#define _mace_put_uint32_t(buf, wire_offset, b) byte_swap_4(&buf[wire_offset], (const char *)&b)
#define _mace_put_int32_t(buf, wire_offset, b)  byte_swap_4(&buf[wire_offset], (const char *)&b)
#define _mace_put_uint64_t(buf, wire_offset, b) byte_swap_8(&buf[wire_offset], (const char *)&b)
#define _mace_put_int64_t(buf, wire_offset, b)  byte_swap_8(&buf[wire_offset], (const char *)&b)
#define _mace_put_float(buf, wire_offset, b)    byte_swap_4(&buf[wire_offset], (const char *)&b)
#define _mace_put_double(buf, wire_offset, b)   byte_swap_8(&buf[wire_offset], (const char *)&b)
#elif !MACE_ALIGNED_FIELDS
#define _mace_put_uint16_t(buf, wire_offset, b) byte_copy_2(&buf[wire_offset], (const char *)&b)
#define _mace_put_int16_t(buf, wire_offset, b)  byte_copy_2(&buf[wire_offset], (const char *)&b)
#define _mace_put_uint32_t(buf, wire_offset, b) byte_copy_4(&buf[wire_offset], (const char *)&b)
#define _mace_put_int32_t(buf, wire_offset, b)  byte_copy_4(&buf[wire_offset], (const char *)&b)
#define _mace_put_uint64_t(buf, wire_offset, b) byte_copy_8(&buf[wire_offset], (const char *)&b)
#define _mace_put_int64_t(buf, wire_offset, b)  byte_copy_8(&buf[wire_offset], (const char *)&b)
#define _mace_put_float(buf, wire_offset, b)    byte_copy_4(&buf[wire_offset], (const char *)&b)
#define _mace_put_double(buf, wire_offset, b)   byte_copy_8(&buf[wire_offset], (const char *)&b)
#else
#define _mace_put_uint16_t(buf, wire_offset, b) *(uint16_t *)&buf[wire_offset] = b
#define _mace_put_int16_t(buf, wire_offset, b)  *(int16_t *)&buf[wire_offset] = b
#define _mace_put_uint32_t(buf, wire_offset, b) *(uint32_t *)&buf[wire_offset] = b
#define _mace_put_int32_t(buf, wire_offset, b)  *(int32_t *)&buf[wire_offset] = b
#define _mace_put_uint64_t(buf, wire_offset, b) *(uint64_t *)&buf[wire_offset] = b
#define _mace_put_int64_t(buf, wire_offset, b)  *(int64_t *)&buf[wire_offset] = b
#define _mace_put_float(buf, wire_offset, b)    *(float *)&buf[wire_offset] = b
#define _mace_put_double(buf, wire_offset, b)   *(double *)&buf[wire_offset] = b
#endif

/*
  like memcpy(), but if src is NULL, do a memset to zero
*/
static inline void mace_array_memcpy(void *dest, const void *src, size_t n)
{
	if (src == NULL) {
		memset(dest, 0, n);
	} else {
		memcpy(dest, src, n);
	}
}

/*
 * Place a char array into a buffer
 */
static inline void _mace_put_char_array(char *buf, uint8_t wire_offset, const char *b, uint8_t array_length)
{
        mace_array_memcpy(&buf[wire_offset], b, array_length);

}

/*
 * Place a uint8_t array into a buffer
 */
static inline void _mace_put_uint8_t_array(char *buf, uint8_t wire_offset, const uint8_t *b, uint8_t array_length)
{
        mace_array_memcpy(&buf[wire_offset], b, array_length);

}

/*
 * Place a int8_t array into a buffer
 */
static inline void _mace_put_int8_t_array(char *buf, uint8_t wire_offset, const int8_t *b, uint8_t array_length)
{
        mace_array_memcpy(&buf[wire_offset], b, array_length);

}

#if MACE_NEED_BYTE_SWAP
#define _MACE_PUT_ARRAY(TYPE, V) \
static inline void _mace_put_ ## TYPE ##_array(char *buf, uint8_t wire_offset, const TYPE *b, uint8_t array_length) \
{ \
	if (b == NULL) { \
		memset(&buf[wire_offset], 0, array_length*sizeof(TYPE)); \
	} else { \
		uint16_t i; \
		for (i=0; i<array_length; i++) { \
			_mace_put_## TYPE (buf, wire_offset+(i*sizeof(TYPE)), b[i]); \
		} \
	} \
}
#else
#define _MACE_PUT_ARRAY(TYPE, V)					\
static inline void _mace_put_ ## TYPE ##_array(char *buf, uint8_t wire_offset, const TYPE *b, uint8_t array_length) \
{ \
        mace_array_memcpy(&buf[wire_offset], b, array_length*sizeof(TYPE)); \
}
#endif

_MACE_PUT_ARRAY(uint16_t, u16)
_MACE_PUT_ARRAY(uint32_t, u32)
_MACE_PUT_ARRAY(uint64_t, u64)
_MACE_PUT_ARRAY(int16_t,  i16)
_MACE_PUT_ARRAY(int32_t,  i32)
_MACE_PUT_ARRAY(int64_t,  i64)
_MACE_PUT_ARRAY(float,    f)
_MACE_PUT_ARRAY(double,   d)

#define _MACE_RETURN_char(msg, wire_offset)             (const char)_MACE_PAYLOAD(msg)[wire_offset]
#define _MACE_RETURN_int8_t(msg, wire_offset)   (const int8_t)_MACE_PAYLOAD(msg)[wire_offset]
#define _MACE_RETURN_uint8_t(msg, wire_offset) (const uint8_t)_MACE_PAYLOAD(msg)[wire_offset]

#if MACE_NEED_BYTE_SWAP
#define _MACE_MSG_RETURN_TYPE(TYPE, SIZE) \
static inline TYPE _MACE_RETURN_## TYPE(const mace_message_t *msg, uint8_t ofs) \
{ TYPE r; byte_swap_## SIZE((char*)&r, &_MACE_PAYLOAD(msg)[ofs]); return r; }

_MACE_MSG_RETURN_TYPE(uint16_t, 2)
_MACE_MSG_RETURN_TYPE(int16_t,  2)
_MACE_MSG_RETURN_TYPE(uint32_t, 4)
_MACE_MSG_RETURN_TYPE(int32_t,  4)
_MACE_MSG_RETURN_TYPE(uint64_t, 8)
_MACE_MSG_RETURN_TYPE(int64_t,  8)
_MACE_MSG_RETURN_TYPE(float,    4)
_MACE_MSG_RETURN_TYPE(double,   8)

#elif !MACE_ALIGNED_FIELDS
#define _MACE_MSG_RETURN_TYPE(TYPE, SIZE) \
static inline TYPE _MACE_RETURN_## TYPE(const mace_message_t *msg, uint8_t ofs) \
{ TYPE r; byte_copy_## SIZE((char*)&r, &_MACE_PAYLOAD(msg)[ofs]); return r; }

_MACE_MSG_RETURN_TYPE(uint16_t, 2)
_MACE_MSG_RETURN_TYPE(int16_t,  2)
_MACE_MSG_RETURN_TYPE(uint32_t, 4)
_MACE_MSG_RETURN_TYPE(int32_t,  4)
_MACE_MSG_RETURN_TYPE(uint64_t, 8)
_MACE_MSG_RETURN_TYPE(int64_t,  8)
_MACE_MSG_RETURN_TYPE(float,    4)
_MACE_MSG_RETURN_TYPE(double,   8)
#else // nicely aligned, no swap
#define _MACE_MSG_RETURN_TYPE(TYPE) \
static inline TYPE _MACE_RETURN_## TYPE(const mace_message_t *msg, uint8_t ofs) \
{ return *(const TYPE *)(&_MACE_PAYLOAD(msg)[ofs]);}

_MACE_MSG_RETURN_TYPE(uint16_t)
_MACE_MSG_RETURN_TYPE(int16_t)
_MACE_MSG_RETURN_TYPE(uint32_t)
_MACE_MSG_RETURN_TYPE(int32_t)
_MACE_MSG_RETURN_TYPE(uint64_t)
_MACE_MSG_RETURN_TYPE(int64_t)
_MACE_MSG_RETURN_TYPE(float)
_MACE_MSG_RETURN_TYPE(double)
#endif // MACE_NEED_BYTE_SWAP

static inline uint16_t _MACE_RETURN_char_array(const mace_message_t *msg, char *value, 
						     uint8_t array_length, uint8_t wire_offset)
{
	memcpy(value, &_MACE_PAYLOAD(msg)[wire_offset], array_length);
	return array_length;
}

static inline uint16_t _MACE_RETURN_uint8_t_array(const mace_message_t *msg, uint8_t *value, 
							uint8_t array_length, uint8_t wire_offset)
{
	memcpy(value, &_MACE_PAYLOAD(msg)[wire_offset], array_length);
	return array_length;
}

static inline uint16_t _MACE_RETURN_int8_t_array(const mace_message_t *msg, int8_t *value, 
						       uint8_t array_length, uint8_t wire_offset)
{
	memcpy(value, &_MACE_PAYLOAD(msg)[wire_offset], array_length);
	return array_length;
}

#if MACE_NEED_BYTE_SWAP
#define _MACE_RETURN_ARRAY(TYPE, V) \
static inline uint16_t _MACE_RETURN_## TYPE ##_array(const mace_message_t *msg, TYPE *value, \
							 uint8_t array_length, uint8_t wire_offset) \
{ \
	uint16_t i; \
	for (i=0; i<array_length; i++) { \
		value[i] = _MACE_RETURN_## TYPE (msg, wire_offset+(i*sizeof(value[0]))); \
	} \
	return array_length*sizeof(value[0]); \
}
#else
#define _MACE_RETURN_ARRAY(TYPE, V)					\
static inline uint16_t _MACE_RETURN_## TYPE ##_array(const mace_message_t *msg, TYPE *value, \
							 uint8_t array_length, uint8_t wire_offset) \
{ \
	memcpy(value, &_MACE_PAYLOAD(msg)[wire_offset], array_length*sizeof(TYPE)); \
	return array_length*sizeof(TYPE); \
}
#endif

_MACE_RETURN_ARRAY(uint16_t, u16)
_MACE_RETURN_ARRAY(uint32_t, u32)
_MACE_RETURN_ARRAY(uint64_t, u64)
_MACE_RETURN_ARRAY(int16_t,  i16)
_MACE_RETURN_ARRAY(int32_t,  i32)
_MACE_RETURN_ARRAY(int64_t,  i64)
_MACE_RETURN_ARRAY(float,    f)
_MACE_RETURN_ARRAY(double,   d)


