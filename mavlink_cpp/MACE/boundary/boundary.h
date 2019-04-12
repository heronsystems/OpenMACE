/** @file
 *  @brief MAVLink comm protocol generated from boundary.xml
 *  @see http://mace.org
 */
#pragma once
#ifndef MACE_BOUNDARY_H
#define MACE_BOUNDARY_H

#ifndef MACE_H
    #error Wrong include order: MACE_BOUNDARY.H MUST NOT BE DIRECTLY USED. Include mace.h from the same directory instead or set ALL AND EVERY defines from MACE.H manually accordingly, including the #define MACE_H call.
#endif

#undef MACE_THIS_XML_IDX
#define MACE_THIS_XML_IDX 3

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MACE_MESSAGE_LENGTHS
#define MACE_MESSAGE_LENGTHS {}
#endif

#ifndef MACE_MESSAGE_CRCS
#define MACE_MESSAGE_CRCS {{130, 52, 6, 0, 0, 0}, {131, 220, 4, 0, 0, 0}, {132, 171, 3, 0, 0, 0}, {133, 122, 5, 0, 0, 0}, {134, 138, 5, 0, 0, 0}, {135, 66, 18, 0, 0, 0}}
#endif

#include "../mace_protocol.h"

#define MACE_ENABLED_BOUNDARY

// ENUM DEFINITIONS


/** @brief Type of boundary items being requested/sent in boundary protocol. */
#ifndef HAVE_ENUM_BOUNDARY_TYPE
#define HAVE_ENUM_BOUNDARY_TYPE
typedef enum BOUNDARY_TYPE
{
   OPERATIONAL_FENCE=0, /* Items are operational boundary coniditions constraining the AO. Often this will be used to describe the general allowable flight area for all vehicles. | */
   RESOURCE_FENCE=1, /* Items are operational boundary conditions that may be a subset of the operational fence. Often used while tasking resources and determining the general area each agent should be concerned with. | */
   GENERIC_POLYGON=2, /* Items are described for a general polygon. | */
   BOUNDARY_TYPE_ENUM_END=3, /*  | */
} BOUNDARY_TYPE;
#endif

/** @brief Result in a mavlink boundary ack. */
#ifndef HAVE_ENUM_BOUNDARY_RESULT
#define HAVE_ENUM_BOUNDARY_RESULT
typedef enum BOUNDARY_RESULT
{
   BOUNDARY_ACCEPTED=0, /* Boundary accepted. | */
   BOUNDARY_ERROR=1, /* Generic error / not accepting boundary commands at all right now. | */
   BOUNDARY_UNSUPPORTED_FRAME=2, /* Coordinate frame is not supported. | */
   BOUNDARY_UNSUPPORTED=3, /* Boundary is not supported. | */
   BOUNDARY_NO_SPACE=4, /* Boundary item exceeds storage space. | */
   BOUNDARY_INVALID=5, /* One of the parameters has an invalid value. | */
   BOUNDARY_INVALID_SEQUENCE=6, /* Received boundary item out of sequence. | */
   BOUNDARY_DENIED=7, /* Not accepting any boundary commands from this communication partner. | */
   BOUNDARY_DOES_NOT_EXIST=15, /* The requested boundary with the associated key does not exist. | */
   BOUNDARY_RESULT_ENUM_END=16, /*  | */
} BOUNDARY_RESULT;
#endif

// MACE VERSION

#ifndef MACE_VERSION
#define MACE_VERSION 2
#endif

#if (MACE_VERSION == 0)
#undef MACE_VERSION
#define MACE_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mace_msg_new_boundary_object.h"
#include "./mace_msg_boundary_ack.h"
#include "./mace_msg_boundary_request_list.h"
#include "./mace_msg_boundary_count.h"
#include "./mace_msg_boundary_request_item.h"
#include "./mace_msg_boundary_item.h"

// base include
#include "../common/common.h"

#undef MACE_THIS_XML_IDX
#define MACE_THIS_XML_IDX 3

#if MACE_THIS_XML_IDX == MACE_PRIMARY_XML_IDX
# define MACE_MESSAGE_INFO {MACE_MESSAGE_INFO_NEW_BOUNDARY_OBJECT, MACE_MESSAGE_INFO_BOUNDARY_ACK, MACE_MESSAGE_INFO_BOUNDARY_REQUEST_LIST, MACE_MESSAGE_INFO_BOUNDARY_COUNT, MACE_MESSAGE_INFO_BOUNDARY_REQUEST_ITEM, MACE_MESSAGE_INFO_BOUNDARY_ITEM}
# if MACE_COMMAND_24BIT
#  include "../mace_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MACE_BOUNDARY_H
