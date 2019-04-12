#ifndef MACE_TYPES_H_
#define MACE_TYPES_H_

// Visual Studio versions before 2010 don't have stdint.h, so we just error out.
#if (defined _MSC_VER) && (_MSC_VER < 1600)
#error "The C-MACELink implementation requires Visual Studio 2010 or greater"
#endif

#include <stdint.h>

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

#define MACE_CORE_HEADER_LEN 5 ///< Length of core header (of the comm. layer): message length (1 byte) + message sequence (1 byte) + message system id (1 byte) + message component id (1 byte) + message type id (1 byte)
#define MACE_NUM_HEADER_BYTES (MACE_CORE_HEADER_LEN + 1) ///< Length of all header bytes, including core and checksum
#define MACE_NUM_CHECKSUM_BYTES 2
#define MACE_NUM_NON_PAYLOAD_BYTES (MACE_NUM_HEADER_BYTES + MACE_NUM_CHECKSUM_BYTES)

#define MACE_MAX_PACKET_LEN (MACE_MAX_PAYLOAD_LEN + MACE_NUM_NON_PAYLOAD_BYTES) ///< Maximum packet length

#define MACE_MSG_ID_EXTENDED_MESSAGE 255
#define MACE_EXTENDED_HEADER_LEN 14

#if (defined _MSC_VER) || ((defined __APPLE__) && (defined __MACH__)) || (defined __linux__)
  /* full fledged 32bit++ OS */
  #define MACE_MAX_EXTENDED_PACKET_LEN 65507
#else
  /* small microcontrollers */
  #define MACE_MAX_EXTENDED_PACKET_LEN 2048
#endif

#define MACE_MAX_EXTENDED_PAYLOAD_LEN (MACE_MAX_EXTENDED_PACKET_LEN - MACE_EXTENDED_HEADER_LEN - MACE_NUM_NON_PAYLOAD_BYTES)


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
typedef struct param_union {
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
typedef struct param_union_extended {
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
	uint16_t checksum; ///< sent at end of packet
	uint8_t magic;   ///< protocol magic marker
	uint8_t len;     ///< Length of payload
	uint8_t seq;     ///< Sequence of packet
	uint8_t sysid;   ///< ID of message sender system/aircraft
	uint8_t compid;  ///< ID of the message sender component
	uint8_t msgid;   ///< ID of message in payload
        uint64_t payload64[(MACE_MAX_PAYLOAD_LEN+MACE_NUM_CHECKSUM_BYTES+7)/8];
}) mace_message_t;

MACEPACKED(
typedef struct __mace_extended_message {
       mace_message_t base_msg;
       int32_t extended_payload_len;   ///< Length of extended payload if any
       uint8_t extended_payload[MACE_MAX_EXTENDED_PAYLOAD_LEN];
}) mace_extended_message_t;

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
    MACE_PARSE_STATE_GOT_SEQ,
    MACE_PARSE_STATE_GOT_LENGTH,
    MACE_PARSE_STATE_GOT_SYSID,
    MACE_PARSE_STATE_GOT_COMPID,
    MACE_PARSE_STATE_GOT_MSGID,
    MACE_PARSE_STATE_GOT_PAYLOAD,
    MACE_PARSE_STATE_GOT_CRC1,
    MACE_PARSE_STATE_GOT_BAD_CRC1
} mace_parse_state_t; ///< The state machine for the comm parser

typedef enum {
    MACE_FRAMING_INCOMPLETE=0,
    MACE_FRAMING_OK=1,
    MACE_FRAMING_BAD_CRC=2
} mace_framing_t;

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
} mace_status_t;

#define MACE_BIG_ENDIAN 0
#define MACE_LITTLE_ENDIAN 1

#endif /* MACE_TYPES_H_ */
