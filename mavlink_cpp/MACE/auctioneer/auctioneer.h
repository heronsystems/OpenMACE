/** @file
 *  @brief MAVLink comm protocol generated from auctioneer.xml
 *  @see http://mace.org
 */
#pragma once
#ifndef MACE_AUCTIONEER_H
#define MACE_AUCTIONEER_H

#ifndef MACE_H
    #error Wrong include order: MACE_AUCTIONEER.H MUST NOT BE DIRECTLY USED. Include mace.h from the same directory instead or set ALL AND EVERY defines from MACE.H manually accordingly, including the #define MACE_H call.
#endif

#undef MACE_THIS_XML_IDX
#define MACE_THIS_XML_IDX 4

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MACE_MESSAGE_LENGTHS
#define MACE_MESSAGE_LENGTHS {}
#endif

#ifndef MACE_MESSAGE_CRCS
#define MACE_MESSAGE_CRCS {{10000, 72, 17, 0, 0, 0}, {10001, 74, 84, 0, 0, 0}, {10002, 175, 17, 0, 0, 0}, {10003, 174, 16, 0, 0, 0}, {10004, 150, 16, 0, 0, 0}, {10005, 239, 21, 0, 0, 0}, {10006, 65, 9, 0, 0, 0}, {10007, 22, 17, 0, 0, 0}, {10008, 200, 9, 0, 0, 0}, {10009, 91, 82, 0, 0, 0}, {10010, 201, 10, 0, 0, 0}, {10011, 35, 76, 0, 0, 0}, {10012, 97, 25, 0, 0, 0}, {10013, 229, 25, 0, 0, 0}, {10014, 231, 25, 0, 0, 0}, {10050, 128, 47, 0, 0, 0}, {10051, 226, 17, 0, 0, 0}, {10052, 214, 17, 0, 0, 0}, {10053, 180, 18, 0, 0, 0}, {10054, 133, 48, 0, 0, 0}, {10055, 54, 43, 0, 0, 0}, {10056, 95, 34, 0, 0, 0}}
#endif

#include "../mace_protocol.h"

#define MACE_ENABLED_AUCTIONEER

// ENUM DEFINITIONS



// MACE VERSION

#ifndef MACE_VERSION
#define MACE_VERSION 3
#endif

#if (MACE_VERSION == 0)
#undef MACE_VERSION
#define MACE_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mace_msg_auction_notify_bid_bundle.h"
#include "./mace_msg_auction_bid_bundle_next_part.h"
#include "./mace_msg_auction_bid_bundle_rcv_part.h"
#include "./mace_msg_auction_bid_bundle_done.h"
#include "./mace_msg_auction_ack_bid_bundle.h"
#include "./mace_msg_auction_new_task_key.h"
#include "./mace_msg_auction_task_key_ack.h"
#include "./mace_msg_auction_request_task_descriptor.h"
#include "./mace_msg_auction_request_task_descriptor_ack.h"
#include "./mace_msg_auction_task_status.h"
#include "./mace_msg_auction_task_status_ack.h"
#include "./mace_msg_auction_bid_descriptor.h"
#include "./mace_msg_auction_bid_descriptor_ack.h"
#include "./mace_msg_auction_scrub_bid.h"
#include "./mace_msg_auction_scrub_bid_ack.h"
#include "./mace_msg_auction_task_descriptor.h"
#include "./mace_msg_auction_task_descriptor_done.h"
#include "./mace_msg_auction_task_descriptor_ack.h"
#include "./mace_msg_auction_task_descriptor_req_part.h"
#include "./mace_msg_auction_task_descriptor_loiter_data.h"
#include "./mace_msg_auction_task_descriptor_part_survey_first.h"
#include "./mace_msg_auction_task_descriptor_part_survey.h"

// base include
#include "../common/common.h"

#undef MACE_THIS_XML_IDX
#define MACE_THIS_XML_IDX 4

#if MACE_THIS_XML_IDX == MACE_PRIMARY_XML_IDX
# define MACE_MESSAGE_INFO {MACE_MESSAGE_INFO_AUCTION_NOTIFY_BID_BUNDLE, MACE_MESSAGE_INFO_AUCTION_BID_BUNDLE_NEXT_PART, MACE_MESSAGE_INFO_AUCTION_BID_BUNDLE_RCV_PART, MACE_MESSAGE_INFO_AUCTION_BID_BUNDLE_DONE, MACE_MESSAGE_INFO_AUCTION_ACK_BID_BUNDLE, MACE_MESSAGE_INFO_AUCTION_NEW_TASK_KEY, MACE_MESSAGE_INFO_AUCTION_TASK_KEY_ACK, MACE_MESSAGE_INFO_AUCTION_REQUEST_TASK_DESCRIPTOR, MACE_MESSAGE_INFO_AUCTION_REQUEST_TASK_DESCRIPTOR_ACK, MACE_MESSAGE_INFO_AUCTION_TASK_STATUS, MACE_MESSAGE_INFO_AUCTION_TASK_STATUS_ACK, MACE_MESSAGE_INFO_AUCTION_BID_DESCRIPTOR, MACE_MESSAGE_INFO_AUCTION_BID_DESCRIPTOR_ACK, MACE_MESSAGE_INFO_AUCTION_SCRUB_BID, MACE_MESSAGE_INFO_AUCTION_SCRUB_BID_ACK, MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR, MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_DONE, MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_ACK, MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_REQ_PART, MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_LOITER_DATA, MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST, MACE_MESSAGE_INFO_AUCTION_TASK_DESCRIPTOR_PART_SURVEY}
# if MACE_COMMAND_24BIT
#  include "../mace_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MACE_AUCTIONEER_H
