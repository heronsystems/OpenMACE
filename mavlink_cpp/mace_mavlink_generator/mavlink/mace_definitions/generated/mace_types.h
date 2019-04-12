#pragma once

// Visual Studio versions before 2010 don't have stdint.h, so we just error out.
#if (defined _MSC_VER) && (_MSC_VER < 1600)
#error "The C-MACELink implementation requires Visual Studio 2010 or greater"
#endif

#include <stdbool.h>
#include <stdint.h>

#ifdef MACE_USE_CXX_NAMESPACE
namespace mace {
#endif

// Macro to define packed structures
#ifdef __GNUC__
  #define MACEPACKED( __Declaration__ ) __Declaration__ __attribute__((packed))
#else
  #define MACEPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#ifndef MACE_MAX_PAYLOAD_LEN
// it is possible to override this, but be careful!
#define MACE_MAX_PAYLOAD_LEN 255 ///< Maximum payload length
#endif

#define MACE_CORE_HEADER_LEN 9 ///< Length of core header (of the comm. layer)
#define MACE_CORE_HEADER_MACE1_LEN 5 ///< Length of MACELink1 core header (of the comm. layer)
#define MACE_NUM_HEADER_BYTES (MACE_CORE_HEADER_LEN + 1) ///< Length of all header bytes, including core and stx
#define MACE_NUM_CHECKSUM_BYTES 2
#define MACE_NUM_NON_PAYLOAD_BYTES (MACE_NUM_HEADER_BYTES + MACE_NUM_CHECKSUM_BYTES)

#define MACE_SIGNATURE_BLOCK_LEN 13

#define MACE_MAX_PACKET_LEN (MACE_MAX_PAYLOAD_LEN + MACE_NUM_NON_PAYLOAD_BYTES + MACE_SIGNATURE_BLOCK_LEN) ///< Maximum packet length

/**
 * Old-style 4 byte param union
 *
 * This struct is the data format to be used when sending
 * parameters. The parameter should be copied to the native
 * type (without type conversion)
 * and re-instanted on the receiving side using the
 * native type as well.
 */
MACEPACKED(
typedef struct mace_param_union {
	union {
		float param_float;
		int32_t param_int32;
		uint32_t param_uint32;
		int16_t param_int16;
		uint16_t param_uint16;
		int8_t param_int8;
		uint8_t param_uint8;
		uint8_t bytes[4];
	};
	uint8_t type;
}) mace_param_union_t;


/**
 * New-style 8 byte param union
 * mace_param_union_double_t will be 8 bytes long, and treated as needing 8 byte alignment for the purposes of MACELink 1.0 field ordering.
 * The mace_param_union_double_t will be treated as a little-endian structure.
 *
 * If is_double is 1 then the type is a double, and the remaining 63 bits are the double, with the lowest bit of the mantissa zero.
 * The intention is that by replacing the is_double bit with 0 the type can be directly used as a double (as the is_double bit corresponds to the
 * lowest mantissa bit of a double). If is_double is 0 then mace_type gives the type in the union.
 * The mace_types.h header will also need to have shifts/masks to define the bit boundaries in the above,
 * as bitfield ordering isn’t consistent between platforms. The above is intended to be for gcc on x86,
 * which should be the same as gcc on little-endian arm. When using shifts/masks the value will be treated as a 64 bit unsigned number,
 * and the bits pulled out using the shifts/masks.
*/
MACEPACKED(
typedef struct mace_param_union_extended {
    union {
    struct {
        uint8_t is_double:1;
        uint8_t mace_type:7;
        union {
            char c;
            uint8_t uint8;
            int8_t int8;
            uint16_t uint16;
            int16_t int16;
            uint32_t uint32;
            int32_t int32;
            float f;
            uint8_t align[7];
        };
    };
    uint8_t data[8];
    };
}) mace_param_union_double_t;

/**
 * This structure is required to make the mace_send_xxx convenience functions
 * work, as it tells the library what the current system and component ID are.
 */
MACEPACKED(
typedef struct __mace_system {
    uint8_t sysid;   ///< Used by the MACELink message_xx_send() convenience function
    uint8_t compid;  ///< Used by the MACELink message_xx_send() convenience function
}) mace_system_t;

MACEPACKED(
typedef struct __mace_message {
	uint16_t checksum;      ///< sent at end of packet
	uint8_t magic;          ///< protocol magic marker
	uint8_t len;            ///< Length of payload
	uint8_t incompat_flags; ///< flags that must be understood
	uint8_t compat_flags;   ///< flags that can be ignored if not understood
	uint8_t seq;            ///< Sequence of packet
	uint8_t sysid;          ///< ID of message sender system/aircraft
	uint8_t compid;         ///< ID of the message sender component
	uint32_t msgid:24;      ///< ID of message in payload
	uint64_t payload64[(MACE_MAX_PAYLOAD_LEN+MACE_NUM_CHECKSUM_BYTES+7)/8];
	uint8_t ck[2];          ///< incoming checksum bytes
	uint8_t signature[MACE_SIGNATURE_BLOCK_LEN];
}) mace_message_t;

typedef enum {
	MACE_TYPE_CHAR     = 0,
	MACE_TYPE_UINT8_T  = 1,
	MACE_TYPE_INT8_T   = 2,
	MACE_TYPE_UINT16_T = 3,
	MACE_TYPE_INT16_T  = 4,
	MACE_TYPE_UINT32_T = 5,
	MACE_TYPE_INT32_T  = 6,
	MACE_TYPE_UINT64_T = 7,
	MACE_TYPE_INT64_T  = 8,
	MACE_TYPE_FLOAT    = 9,
	MACE_TYPE_DOUBLE   = 10
} mace_message_type_t;

#define MACE_MAX_FIELDS 64

typedef struct __mace_field_info {
	const char *name;                 // name of this field
        const char *print_format;         // printing format hint, or NULL
        mace_message_type_t type;      // type of this field
        unsigned int array_length;        // if non-zero, field is an array
        unsigned int wire_offset;         // offset of each field in the payload
        unsigned int structure_offset;    // offset in a C structure
} mace_field_info_t;

// note that in this structure the order of fields is the order
// in the XML file, not necessary the wire order
typedef struct __mace_message_info {
	uint32_t msgid;                                        // message ID
	const char *name;                                      // name of the message
	unsigned num_fields;                                   // how many fields in this message
	mace_field_info_t fields[MACE_MAX_FIELDS];       // field information
} mace_message_info_t;

#define _MACE_PAYLOAD(msg) ((const char *)(&((msg)->payload64[0])))
#define _MACE_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload64[0])))

// checksum is immediately after the payload bytes
#define mace_ck_a(msg) *((msg)->len + (uint8_t *)_MACE_PAYLOAD_NON_CONST(msg))
#define mace_ck_b(msg) *(((msg)->len+(uint16_t)1) + (uint8_t *)_MACE_PAYLOAD_NON_CONST(msg))

typedef enum {
    MACE_COMM_0,
    MACE_COMM_1,
    MACE_COMM_2,
    MACE_COMM_3
} mace_channel_t;

/*
 * applications can set MACE_COMM_NUM_BUFFERS to the maximum number
 * of buffers they will use. If more are used, then the result will be
 * a stack overrun
 */
#ifndef MACE_COMM_NUM_BUFFERS
#if (defined linux) | (defined __linux) | (defined  __MACH__) | (defined _WIN32)
# define MACE_COMM_NUM_BUFFERS 16
#else
# define MACE_COMM_NUM_BUFFERS 4
#endif
#endif

typedef enum {
    MACE_PARSE_STATE_UNINIT=0,
    MACE_PARSE_STATE_IDLE,
    MACE_PARSE_STATE_GOT_STX,
    MACE_PARSE_STATE_GOT_LENGTH,
    MACE_PARSE_STATE_GOT_INCOMPAT_FLAGS,
    MACE_PARSE_STATE_GOT_COMPAT_FLAGS,
    MACE_PARSE_STATE_GOT_SEQ,
    MACE_PARSE_STATE_GOT_SYSID,
    MACE_PARSE_STATE_GOT_COMPID,
    MACE_PARSE_STATE_GOT_MSGID1,
    MACE_PARSE_STATE_GOT_MSGID2,
    MACE_PARSE_STATE_GOT_MSGID3,
    MACE_PARSE_STATE_GOT_PAYLOAD,
    MACE_PARSE_STATE_GOT_CRC1,
    MACE_PARSE_STATE_GOT_BAD_CRC1,
    MACE_PARSE_STATE_SIGNATURE_WAIT
} mace_parse_state_t; ///< The state machine for the comm parser

typedef enum {
    MACE_FRAMING_INCOMPLETE=0,
    MACE_FRAMING_OK=1,
    MACE_FRAMING_BAD_CRC=2,
    MACE_FRAMING_BAD_SIGNATURE=3
} mace_framing_t;

#define MACE_STATUS_FLAG_IN_MACE1  1 // last incoming packet was MACELink1
#define MACE_STATUS_FLAG_OUT_MACE1 2 // generate MACELink1 by default
#define MACE_STATUS_FLAG_IN_SIGNED    4 // last incoming packet was signed and validated
#define MACE_STATUS_FLAG_IN_BADSIG    8 // last incoming packet had a bad signature

#define MACE_STX_MACE1 0xFE          // marker for old protocol

typedef struct __mace_status {
    uint8_t msg_received;               ///< Number of received messages
    uint8_t buffer_overrun;             ///< Number of buffer overruns
    uint8_t parse_error;                ///< Number of parse errors
    mace_parse_state_t parse_state;  ///< Parsing state machine
    uint8_t packet_idx;                 ///< Index in current packet
    uint8_t current_rx_seq;             ///< Sequence number of last packet received
    uint8_t current_tx_seq;             ///< Sequence number of last packet sent
    uint16_t packet_rx_success_count;   ///< Received packets
    uint16_t packet_rx_drop_count;      ///< Number of packet drops
    uint8_t flags;                      ///< MACE_STATUS_FLAG_*
    uint8_t signature_wait;             ///< number of signature bytes left to receive
    struct __mace_signing *signing;  ///< optional signing state
    struct __mace_signing_streams *signing_streams; ///< global record of stream timestamps
} mace_status_t;

/*
  a callback function to allow for accepting unsigned packets
 */
typedef bool (*mace_accept_unsigned_t)(const mace_status_t *status, uint32_t msgid);

/*
  flags controlling signing
 */
#define MACE_SIGNING_FLAG_SIGN_OUTGOING 1

/*
  state of MACELink signing for this channel
 */
typedef struct __mace_signing {
    uint8_t flags;                     ///< MACE_SIGNING_FLAG_*
    uint8_t link_id;
    uint64_t timestamp;
    uint8_t secret_key[32];
    mace_accept_unsigned_t accept_unsigned_callback;
} mace_signing_t;

/*
  timestamp state of each logical signing stream. This needs to be the same structure for all
  connections in order to be secure
 */
#ifndef MACE_MAX_SIGNING_STREAMS
#define MACE_MAX_SIGNING_STREAMS 16
#endif
typedef struct __mace_signing_streams {
    uint16_t num_signing_streams;
    struct __mace_signing_stream {
        uint8_t link_id;
        uint8_t sysid;
        uint8_t compid;
        uint8_t timestamp_bytes[6];
    } stream[MACE_MAX_SIGNING_STREAMS];
} mace_signing_streams_t;


#define MACE_BIG_ENDIAN 0
#define MACE_LITTLE_ENDIAN 1

#define MACE_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM    1
#define MACE_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT 2

/*
  entry in table of information about each message type
 */
typedef struct __mace_msg_entry {
	uint32_t msgid;
	uint8_t crc_extra;
	uint8_t msg_len;
        uint8_t flags;             // MACE_MSG_ENTRY_FLAG_*
	uint8_t target_system_ofs; // payload offset to target_system, or 0
	uint8_t target_component_ofs; // payload offset to target_component, or 0
} mace_msg_entry_t;

/*
  incompat_flags bits
 */
#define MACE_IFLAG_SIGNED  0x01
#define MACE_IFLAG_MASK    0x01 // mask of all understood bits

#ifdef MACE_USE_CXX_NAMESPACE
} // namespace mace
#endif
