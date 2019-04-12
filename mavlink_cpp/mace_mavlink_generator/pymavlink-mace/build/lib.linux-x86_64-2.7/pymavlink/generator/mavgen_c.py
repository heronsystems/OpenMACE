#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a C implementation

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''
from __future__ import print_function

from builtins import range
from builtins import object

import os
from . import mavparse, mavtemplate

t = mavtemplate.MAVTemplate()

def generate_version_h(directory, xml):
    '''generate version.h'''
    f = open(os.path.join(directory, "version.h"), mode='w')
    t.write(f,'''
/** @file
 *  @brief MAVLink comm protocol built from ${basename}.xml
 *  @see http://mace.org
 */
#pragma once
 
#ifndef MACE_VERSION_H
#define MACE_VERSION_H

#define MACE_BUILD_DATE "${parse_time}"
#define MACE_WIRE_PROTOCOL_VERSION "${wire_protocol_version}"
#define MACE_MAX_DIALECT_PAYLOAD_SIZE ${largest_payload}
 
#endif // MACE_VERSION_H
''', xml)
    f.close()

def generate_mace_h(directory, xml):
    '''generate mace.h'''
    f = open(os.path.join(directory, "mace.h"), mode='w')
    t.write(f,'''
/** @file
 *  @brief MAVLink comm protocol built from ${basename}.xml
 *  @see http://mace.org
 */
#pragma once
#ifndef MACE_H
#define MACE_H

#define MACE_PRIMARY_XML_IDX ${xml_idx}

#ifndef MACE_STX
#define MACE_STX ${protocol_marker}
#endif

#ifndef MACE_ENDIAN
#define MACE_ENDIAN ${mace_endian}
#endif

#ifndef MACE_ALIGNED_FIELDS
#define MACE_ALIGNED_FIELDS ${aligned_fields_define}
#endif

#ifndef MACE_CRC_EXTRA
#define MACE_CRC_EXTRA ${crc_extra_define}
#endif

#ifndef MACE_COMMAND_24BIT
#define MACE_COMMAND_24BIT ${command_24bit_define}
#endif

#include "version.h"
#include "${basename}.h"

#endif // MACE_H
''', xml)
    f.close()

def generate_main_h(directory, xml):
    '''generate main header per XML file'''
    f = open(os.path.join(directory, xml.basename + ".h"), mode='w')
    t.write(f, '''
/** @file
 *  @brief MAVLink comm protocol generated from ${basename}.xml
 *  @see http://mace.org
 */
#pragma once
#ifndef MACE_${basename_upper}_H
#define MACE_${basename_upper}_H

#ifndef MACE_H
    #error Wrong include order: MACE_${basename_upper}.H MUST NOT BE DIRECTLY USED. Include mace.h from the same directory instead or set ALL AND EVERY defines from MACE.H manually accordingly, including the #define MACE_H call.
#endif

#undef MACE_THIS_XML_IDX
#define MACE_THIS_XML_IDX ${xml_idx}

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MACE_MESSAGE_LENGTHS
#define MACE_MESSAGE_LENGTHS {${message_lengths_array}}
#endif

#ifndef MACE_MESSAGE_CRCS
#define MACE_MESSAGE_CRCS {${message_crcs_array}}
#endif

#include "../mace_protocol.h"

#define MACE_ENABLED_${basename_upper}

// ENUM DEFINITIONS

${{enum:
/** @brief ${description} */
#ifndef HAVE_ENUM_${name}
#define HAVE_ENUM_${name}
typedef enum ${name}
{
${{entry:   ${name}=${value}, /* ${description} |${{param:${description}| }} */
}}
} ${name};
#endif
}}

// MACE VERSION

#ifndef MACE_VERSION
#define MACE_VERSION ${version}
#endif

#if (MACE_VERSION == 0)
#undef MACE_VERSION
#define MACE_VERSION ${version}
#endif

// MESSAGE DEFINITIONS
${{message:#include "./mace_msg_${name_lower}.h"
}}

// base include
${{include_list:#include "../${base}/${base}.h"
}}

#undef MACE_THIS_XML_IDX
#define MACE_THIS_XML_IDX ${xml_idx}

#if MACE_THIS_XML_IDX == MACE_PRIMARY_XML_IDX
# define MACE_MESSAGE_INFO {${message_info_array}}
# if MACE_COMMAND_24BIT
#  include "../mace_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MACE_${basename_upper}_H
''', xml)

    f.close()
             

def generate_message_h(directory, m):
    '''generate per-message header for a XML file'''
    f = open(os.path.join(directory, 'mace_msg_%s.h' % m.name_lower), mode='w')
    t.write(f, '''
#pragma once
// MESSAGE ${name} PACKING

#define MACE_MSG_ID_${name} ${id}

MACEPACKED(
typedef struct __mace_${name_lower}_t {
${{ordered_fields: ${type} ${name}${array_suffix}; /*< ${description}*/
}}
}) mace_${name_lower}_t;

#define MACE_MSG_ID_${name}_LEN ${wire_length}
#define MACE_MSG_ID_${name}_MIN_LEN ${wire_min_length}
#define MACE_MSG_ID_${id}_LEN ${wire_length}
#define MACE_MSG_ID_${id}_MIN_LEN ${wire_min_length}

#define MACE_MSG_ID_${name}_CRC ${crc_extra}
#define MACE_MSG_ID_${id}_CRC ${crc_extra}

${{array_fields:#define MACE_MSG_${msg_name}_FIELD_${name_upper}_LEN ${array_length}
}}

#if MACE_COMMAND_24BIT
#define MACE_MESSAGE_INFO_${name} { \\
    ${id}, \\
    "${name}", \\
    ${num_fields}, \\
    { ${{ordered_fields: { "${name}", ${c_print_format}, MACE_TYPE_${type_upper}, ${array_length}, ${wire_offset}, offsetof(mace_${name_lower}_t, ${name}) }, \\
        }} } \\
}
#else
#define MACE_MESSAGE_INFO_${name} { \\
    "${name}", \\
    ${num_fields}, \\
    { ${{ordered_fields: { "${name}", ${c_print_format}, MACE_TYPE_${type_upper}, ${array_length}, ${wire_offset}, offsetof(mace_${name_lower}_t, ${name}) }, \\
        }} } \\
}
#endif

/**
 * @brief Pack a ${name_lower} message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
${{arg_fields: * @param ${name} ${description}
}}
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_${name_lower}_pack(uint8_t system_id, uint8_t component_id, mace_message_t* msg,
                              ${{arg_fields: ${array_const}${type} ${array_prefix}${name},}})
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_${name}_LEN];
${{scalar_fields:    _mace_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:    _mace_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_${name}_LEN);
#else
    mace_${name_lower}_t packet;
${{scalar_fields:    packet.${name} = ${putname};
}}
${{array_fields:    mace_array_memcpy(packet.${name}, ${name}, sizeof(${type})*${array_length});
}}
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_${name}_LEN);
#endif

    msg->msgid = MACE_MSG_ID_${name};
    return mace_finalize_message(msg, system_id, component_id, MACE_MSG_ID_${name}_MIN_LEN, MACE_MSG_ID_${name}_LEN, MACE_MSG_ID_${name}_CRC);
}

/**
 * @brief Pack a ${name_lower} message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
${{arg_fields: * @param ${name} ${description}
}}
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mace_msg_${name_lower}_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mace_message_t* msg,
                                   ${{arg_fields:${array_const}${type} ${array_prefix}${name},}})
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_${name}_LEN];
${{scalar_fields:    _mace_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:    _mace_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), buf, MACE_MSG_ID_${name}_LEN);
#else
    mace_${name_lower}_t packet;
${{scalar_fields:    packet.${name} = ${putname};
}}
${{array_fields:    mace_array_memcpy(packet.${name}, ${name}, sizeof(${type})*${array_length});
}}
        memcpy(_MACE_PAYLOAD_NON_CONST(msg), &packet, MACE_MSG_ID_${name}_LEN);
#endif

    msg->msgid = MACE_MSG_ID_${name};
    return mace_finalize_message_chan(msg, system_id, component_id, chan, MACE_MSG_ID_${name}_MIN_LEN, MACE_MSG_ID_${name}_LEN, MACE_MSG_ID_${name}_CRC);
}

/**
 * @brief Encode a ${name_lower} struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ${name_lower} C-struct to read the message contents from
 */
static inline uint16_t mace_msg_${name_lower}_encode(uint8_t system_id, uint8_t component_id, mace_message_t* msg, const mace_${name_lower}_t* ${name_lower})
{
    return mace_msg_${name_lower}_pack(system_id, component_id, msg,${{arg_fields: ${name_lower}->${name},}});
}

/**
 * @brief Encode a ${name_lower} struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ${name_lower} C-struct to read the message contents from
 */
static inline uint16_t mace_msg_${name_lower}_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mace_message_t* msg, const mace_${name_lower}_t* ${name_lower})
{
    return mace_msg_${name_lower}_pack_chan(system_id, component_id, chan, msg,${{arg_fields: ${name_lower}->${name},}});
}

/**
 * @brief Send a ${name_lower} message
 * @param chan MAVLink channel to send the message
 *
${{arg_fields: * @param ${name} ${description}
}}
 */
#ifdef MACE_USE_CONVENIENCE_FUNCTIONS

static inline void mace_msg_${name_lower}_send(mace_channel_t chan,${{arg_fields: ${array_const}${type} ${array_prefix}${name},}})
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char buf[MACE_MSG_ID_${name}_LEN];
${{scalar_fields:    _mace_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:    _mace_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_${name}, buf, MACE_MSG_ID_${name}_MIN_LEN, MACE_MSG_ID_${name}_LEN, MACE_MSG_ID_${name}_CRC);
#else
    mace_${name_lower}_t packet;
${{scalar_fields:    packet.${name} = ${putname};
}}
${{array_fields:    mace_array_memcpy(packet.${name}, ${name}, sizeof(${type})*${array_length});
}}
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_${name}, (const char *)&packet, MACE_MSG_ID_${name}_MIN_LEN, MACE_MSG_ID_${name}_LEN, MACE_MSG_ID_${name}_CRC);
#endif
}

/**
 * @brief Send a ${name_lower} message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mace_msg_${name_lower}_send_struct(mace_channel_t chan, const mace_${name_lower}_t* ${name_lower})
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    mace_msg_${name_lower}_send(chan,${{arg_fields: ${name_lower}->${name},}});
#else
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_${name}, (const char *)${name_lower}, MACE_MSG_ID_${name}_MIN_LEN, MACE_MSG_ID_${name}_LEN, MACE_MSG_ID_${name}_CRC);
#endif
}

#if MACE_MSG_ID_${name}_LEN <= MACE_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mace_message_t which is the size of a full mace message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mace_msg_${name_lower}_send_buf(mace_message_t *msgbuf, mace_channel_t chan, ${{arg_fields: ${array_const}${type} ${array_prefix}${name},}})
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
${{scalar_fields:    _mace_put_${type}(buf, ${wire_offset}, ${putname});
}}
${{array_fields:    _mace_put_${type}_array(buf, ${wire_offset}, ${name}, ${array_length});
}}
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_${name}, buf, MACE_MSG_ID_${name}_MIN_LEN, MACE_MSG_ID_${name}_LEN, MACE_MSG_ID_${name}_CRC);
#else
    mace_${name_lower}_t *packet = (mace_${name_lower}_t *)msgbuf;
${{scalar_fields:    packet->${name} = ${putname};
}}
${{array_fields:    mace_array_memcpy(packet->${name}, ${name}, sizeof(${type})*${array_length});
}}
    _mace_finalize_message_chan_send(chan, MACE_MSG_ID_${name}, (const char *)packet, MACE_MSG_ID_${name}_MIN_LEN, MACE_MSG_ID_${name}_LEN, MACE_MSG_ID_${name}_CRC);
#endif
}
#endif

#endif

// MESSAGE ${name} UNPACKING

${{fields:
/**
 * @brief Get field ${name} from ${name_lower} message
 *
 * @return ${description}
 */
static inline ${return_type} mace_msg_${name_lower}_get_${name}(const mace_message_t* msg${get_arg})
{
    return _MACE_RETURN_${type}${array_tag}(msg, ${array_return_arg} ${wire_offset});
}
}}

/**
 * @brief Decode a ${name_lower} message into a struct
 *
 * @param msg The message to decode
 * @param ${name_lower} C-struct to decode the message contents into
 */
static inline void mace_msg_${name_lower}_decode(const mace_message_t* msg, mace_${name_lower}_t* ${name_lower})
{
#if MACE_NEED_BYTE_SWAP || !MACE_ALIGNED_FIELDS
${{ordered_fields:    ${decode_left}mace_msg_${name_lower}_get_${name}(msg${decode_right});
}}
#else
        uint8_t len = msg->len < MACE_MSG_ID_${name}_LEN? msg->len : MACE_MSG_ID_${name}_LEN;
        memset(${name_lower}, 0, MACE_MSG_ID_${name}_LEN);
    memcpy(${name_lower}, _MACE_PAYLOAD(msg), len);
#endif
}
''', m)
    f.close()


def generate_testsuite_h(directory, xml):
    '''generate testsuite.h per XML file'''
    f = open(os.path.join(directory, "testsuite.h"), mode='w')
    t.write(f, '''
/** @file
 *    @brief MAVLink comm protocol testsuite generated from ${basename}.xml
 *    @see http://qgroundcontrol.org/mace/
 */
#pragma once
#ifndef ${basename_upper}_TESTSUITE_H
#define ${basename_upper}_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MACE_TEST_ALL
#define MACE_TEST_ALL
${{include_list:static void mace_test_${base}(uint8_t, uint8_t, mace_message_t *last_msg);
}}
static void mace_test_${basename}(uint8_t, uint8_t, mace_message_t *last_msg);

static void mace_test_all(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
${{include_list:    mace_test_${base}(system_id, component_id, last_msg);
}}
    mace_test_${basename}(system_id, component_id, last_msg);
}
#endif

${{include_list:#include "../${base}/testsuite.h"
}}

${{message:
static void mace_test_${name_lower}(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_${name} >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_${name_lower}_t packet_in = {
        ${{ordered_fields:${c_test_value},}}
    };
    mace_${name_lower}_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        ${{scalar_fields:packet1.${name} = packet_in.${name};
        }}
        ${{array_fields:mace_array_memcpy(packet1.${name}, packet_in.${name}, sizeof(${type})*${array_length});
        }}
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_${name}_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_${name}_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_${name_lower}_encode(system_id, component_id, &msg, &packet1);
    mace_msg_${name_lower}_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_${name_lower}_pack(system_id, component_id, &msg ${{arg_fields:, packet1.${name} }});
    mace_msg_${name_lower}_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_${name_lower}_pack_chan(system_id, component_id, MACE_COMM_0, &msg ${{arg_fields:, packet1.${name} }});
    mace_msg_${name_lower}_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_${name_lower}_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_${name_lower}_send(MACE_COMM_1 ${{arg_fields:, packet1.${name} }});
    mace_msg_${name_lower}_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}
}}

static void mace_test_${basename}(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
${{message:    mace_test_${name_lower}(system_id, component_id, last_msg);
}}
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ${basename_upper}_TESTSUITE_H
''', xml)

    f.close()

def copy_fixed_headers(directory, xml):
    '''copy the fixed protocol headers to the target directory'''
    import shutil, filecmp
    hlist = {
        "0.9": [ 'mace_protocol.h', 'mace_helpers.h', 'mace_types.h', 'mace_checksum.h' ],
        "1.0": [ 'mace_protocol.h', 'mace_helpers.h', 'mace_types.h', 'mace_checksum.h', 'mace_conversions.h' ],
        "2.0": [ 'mace_protocol.h', 'mace_helpers.h', 'mace_types.h', 'mace_checksum.h', 'mace_conversions.h',
                 'mace_get_info.h', 'mace_sha256.h' ]
        }
    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'C/include_v%s' % xml.wire_protocol_version)
    print("Copying fixed headers for protocol %s to %s" % (xml.wire_protocol_version, directory))
    for h in hlist[xml.wire_protocol_version]:
        src = os.path.realpath(os.path.join(srcpath, h))
        dest = os.path.realpath(os.path.join(directory, h))
        if src == dest or (os.path.exists(dest) and filecmp.cmp(src, dest)):
            continue
        shutil.copy(src, dest)

class mav_include(object):
    def __init__(self, base):
        self.base = base

def generate_one(basename, xml):
    '''generate headers for one XML file'''

    directory = os.path.join(basename, xml.basename)

    print("Generating C implementation in directory %s" % directory)
    mavparse.mkdir_p(directory)

    if xml.little_endian:
        xml.mace_endian = "MACE_LITTLE_ENDIAN"
    else:
        xml.mace_endian = "MACE_BIG_ENDIAN"

    if xml.crc_extra:
        xml.crc_extra_define = "1"
    else:
        xml.crc_extra_define = "0"

    if xml.command_24bit:
        xml.command_24bit_define = "1"
    else:
        xml.command_24bit_define = "0"

    if xml.sort_fields:
        xml.aligned_fields_define = "1"
    else:
        xml.aligned_fields_define = "0"

    # work out the included headers
    xml.include_list = []
    for i in xml.include:
        base = i[:-4]
        xml.include_list.append(mav_include(base))

    # form message lengths array
    xml.message_lengths_array = ''
    if not xml.command_24bit:
        for msgid in range(256):
            mlen = xml.message_min_lengths.get(msgid, 0)
            xml.message_lengths_array += '%u, ' % mlen
        xml.message_lengths_array = xml.message_lengths_array[:-2]

    # and message CRCs array
    xml.message_crcs_array = ''
    if xml.command_24bit:
        # we sort with primary key msgid
        for msgid in sorted(xml.message_crcs.keys()):
            xml.message_crcs_array += '{%u, %u, %u, %u, %u, %u}, ' % (msgid,
                                                                      xml.message_crcs[msgid],
                                                                      xml.message_min_lengths[msgid],
                                                                      xml.message_flags[msgid],
                                                                      xml.message_target_system_ofs[msgid],
                                                                      xml.message_target_component_ofs[msgid])
    else:
        for msgid in range(256):
            crc = xml.message_crcs.get(msgid, 0)
            xml.message_crcs_array += '%u, ' % crc
    xml.message_crcs_array = xml.message_crcs_array[:-2]

    # form message info array
    xml.message_info_array = ''
    if xml.command_24bit:
        # we sort with primary key msgid
        for msgid in sorted(xml.message_names.keys()):
            name = xml.message_names[msgid]
            xml.message_info_array += 'MACE_MESSAGE_INFO_%s, ' % name
    else:
        for msgid in range(256):
            name = xml.message_names.get(msgid, None)
            if name is not None:
                xml.message_info_array += 'MACE_MESSAGE_INFO_%s, ' % name
            else:
                # Several C compilers don't accept {NULL} for
                # multi-dimensional arrays and structs
                # feed the compiler a "filled" empty message
                xml.message_info_array += '{"EMPTY",0,{{"","",MACE_TYPE_CHAR,0,0,0}}}, '
    xml.message_info_array = xml.message_info_array[:-2]

    # add some extra field attributes for convenience with arrays
    for m in xml.message:
        m.msg_name = m.name
        if xml.crc_extra:
            m.crc_extra_arg = ", %s" % m.crc_extra
        else:
            m.crc_extra_arg = ""
        for f in m.fields:
            if f.print_format is None:
                f.c_print_format = 'NULL'
            else:
                f.c_print_format = '"%s"' % f.print_format
            if f.array_length != 0:
                f.array_suffix = '[%u]' % f.array_length
                f.array_prefix = '*'
                f.array_tag = '_array'
                f.array_arg = ', %u' % f.array_length
                f.array_return_arg = '%s, %u, ' % (f.name, f.array_length)
                f.array_const = 'const '
                f.decode_left = ''
                f.decode_right = ', %s->%s' % (m.name_lower, f.name)
                f.return_type = 'uint16_t'
                f.get_arg = ', %s *%s' % (f.type, f.name)
                if f.type == 'char':
                    f.c_test_value = '"%s"' % f.test_value
                else:
                    test_strings = []
                    for v in f.test_value:
                        test_strings.append(str(v))
                    f.c_test_value = '{ %s }' % ', '.join(test_strings)
            else:
                f.array_suffix = ''
                f.array_prefix = ''
                f.array_tag = ''
                f.array_arg = ''
                f.array_return_arg = ''
                f.array_const = ''
                f.decode_left = "%s->%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.get_arg = ''
                f.return_type = f.type
                if f.type == 'char':
                    f.c_test_value = "'%s'" % f.test_value
                elif f.type == 'uint64_t':
                    f.c_test_value = "%sULL" % f.test_value                    
                elif f.type == 'int64_t':
                    f.c_test_value = "%sLL" % f.test_value                    
                else:
                    f.c_test_value = f.test_value

    # cope with uint8_t_mace_version
    for m in xml.message:
        m.arg_fields = []
        m.array_fields = []
        m.scalar_fields = []
        for f in m.ordered_fields:
            if f.array_length != 0:
                m.array_fields.append(f)
            else:
                m.scalar_fields.append(f)
        for f in m.fields:
            if not f.omit_arg:
                m.arg_fields.append(f)
                f.putname = f.name
            else:
                f.putname = f.const_value

    generate_mace_h(directory, xml)
    generate_version_h(directory, xml)
    generate_main_h(directory, xml)
    for m in xml.message:
        generate_message_h(directory, m)
    generate_testsuite_h(directory, xml)


def generate(basename, xml_list):
    '''generate complete MAVLink C implemenation'''

    for idx in range(len(xml_list)):
        xml = xml_list[idx]
        xml.xml_idx = idx
        generate_one(basename, xml)
    copy_fixed_headers(basename, xml_list[0])
