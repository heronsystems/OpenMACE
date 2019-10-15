/** @file
 *	@brief MAVLink comm protocol generated from auctioneer.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace auctioneer {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (trought @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 22> MESSAGE_ENTRIES {{ {10000, 72, 17, 0, 0, 0}, {10001, 74, 84, 0, 0, 0}, {10002, 175, 17, 0, 0, 0}, {10003, 174, 16, 0, 0, 0}, {10004, 150, 16, 0, 0, 0}, {10005, 239, 21, 0, 0, 0}, {10006, 65, 9, 0, 0, 0}, {10007, 22, 17, 0, 0, 0}, {10008, 200, 9, 0, 0, 0}, {10009, 91, 82, 0, 0, 0}, {10010, 201, 10, 0, 0, 0}, {10011, 35, 76, 0, 0, 0}, {10012, 97, 25, 0, 0, 0}, {10013, 229, 25, 0, 0, 0}, {10014, 231, 25, 0, 0, 0}, {10050, 128, 47, 0, 0, 0}, {10051, 226, 17, 0, 0, 0}, {10052, 214, 17, 0, 0, 0}, {10053, 180, 18, 0, 0, 0}, {10054, 133, 48, 0, 0, 0}, {10055, 54, 43, 0, 0, 0}, {10056, 95, 34, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 3;


// ENUM DEFINITIONS




} // namespace auctioneer
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_auction_notify_bid_bundle.hpp"
#include "./mavlink_msg_auction_bid_bundle_next_part.hpp"
#include "./mavlink_msg_auction_bid_bundle_rcv_part.hpp"
#include "./mavlink_msg_auction_bid_bundle_done.hpp"
#include "./mavlink_msg_auction_ack_bid_bundle.hpp"
#include "./mavlink_msg_auction_new_task_key.hpp"
#include "./mavlink_msg_auction_task_key_ack.hpp"
#include "./mavlink_msg_auction_request_task_descriptor.hpp"
#include "./mavlink_msg_auction_request_task_descriptor_ack.hpp"
#include "./mavlink_msg_auction_task_status.hpp"
#include "./mavlink_msg_auction_task_status_ack.hpp"
#include "./mavlink_msg_auction_bid_descriptor.hpp"
#include "./mavlink_msg_auction_bid_descriptor_ack.hpp"
#include "./mavlink_msg_auction_scrub_bid.hpp"
#include "./mavlink_msg_auction_scrub_bid_ack.hpp"
#include "./mavlink_msg_auction_task_descriptor.hpp"
#include "./mavlink_msg_auction_task_descriptor_done.hpp"
#include "./mavlink_msg_auction_task_descriptor_ack.hpp"
#include "./mavlink_msg_auction_task_descriptor_req_part.hpp"
#include "./mavlink_msg_auction_task_descriptor_loiter_data.hpp"
#include "./mavlink_msg_auction_task_descriptor_part_survey_first.hpp"
#include "./mavlink_msg_auction_task_descriptor_part_survey.hpp"

// base include
#include "../common/common.hpp"
