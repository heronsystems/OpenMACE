/** @file
 *	@brief MAVLink comm testsuite protocol generated from auctioneer.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "auctioneer.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(auctioneer, AUCTION_NOTIFY_BID_BUNDLE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_NOTIFY_BID_BUNDLE packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.bundleGenTime = 179.0;
    packet_in.numBids = 53;

    mavlink::auctioneer::msg::AUCTION_NOTIFY_BID_BUNDLE packet1{};
    mavlink::auctioneer::msg::AUCTION_NOTIFY_BID_BUNDLE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.agentID, packet2.agentID);
    EXPECT_EQ(packet1.bundleGenTime, packet2.bundleGenTime);
    EXPECT_EQ(packet1.numBids, packet2.numBids);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_NOTIFY_BID_BUNDLE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_notify_bid_bundle_t packet_c {
         93372036854775807ULL, 179.0, 53
    };

    mavlink::auctioneer::msg::AUCTION_NOTIFY_BID_BUNDLE packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.bundleGenTime = 179.0;
    packet_in.numBids = 53;

    mavlink::auctioneer::msg::AUCTION_NOTIFY_BID_BUNDLE packet2{};

    mavlink_msg_auction_notify_bid_bundle_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.agentID, packet2.agentID);
    EXPECT_EQ(packet_in.bundleGenTime, packet2.bundleGenTime);
    EXPECT_EQ(packet_in.numBids, packet2.numBids);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_BID_BUNDLE_NEXT_PART)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_NEXT_PART packet_in{};
    packet_in.requestFrom = 93372036854775807ULL;
    packet_in.agentID = 93372036854776311ULL;
    packet_in.bundleGenTime = 235.0;
    packet_in.bidGenTime = 291.0;
    packet_in.utility = 347.0;
    packet_in.work = 403.0;
    packet_in.cost = 459.0;
    packet_in.reward = 515.0;
    packet_in.creatorID = 93372036854779839ULL;
    packet_in.taskID = 245;
    packet_in.taskGenTime = 627.0;
    packet_in.type = 56;
    packet_in.priority = 123;
    packet_in.seqNum = -66;

    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_NEXT_PART packet1{};
    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_NEXT_PART packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.requestFrom, packet2.requestFrom);
    EXPECT_EQ(packet1.agentID, packet2.agentID);
    EXPECT_EQ(packet1.bundleGenTime, packet2.bundleGenTime);
    EXPECT_EQ(packet1.bidGenTime, packet2.bidGenTime);
    EXPECT_EQ(packet1.utility, packet2.utility);
    EXPECT_EQ(packet1.work, packet2.work);
    EXPECT_EQ(packet1.cost, packet2.cost);
    EXPECT_EQ(packet1.reward, packet2.reward);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
    EXPECT_EQ(packet1.taskGenTime, packet2.taskGenTime);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.priority, packet2.priority);
    EXPECT_EQ(packet1.seqNum, packet2.seqNum);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_BID_BUNDLE_NEXT_PART)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_bid_bundle_next_part_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 235.0, 291.0, 347.0, 403.0, 459.0, 515.0, 93372036854779839ULL, 627.0, 245, 56, 123, -66
    };

    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_NEXT_PART packet_in{};
    packet_in.requestFrom = 93372036854775807ULL;
    packet_in.agentID = 93372036854776311ULL;
    packet_in.bundleGenTime = 235.0;
    packet_in.bidGenTime = 291.0;
    packet_in.utility = 347.0;
    packet_in.work = 403.0;
    packet_in.cost = 459.0;
    packet_in.reward = 515.0;
    packet_in.creatorID = 93372036854779839ULL;
    packet_in.taskID = 245;
    packet_in.taskGenTime = 627.0;
    packet_in.type = 56;
    packet_in.priority = 123;
    packet_in.seqNum = -66;

    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_NEXT_PART packet2{};

    mavlink_msg_auction_bid_bundle_next_part_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.requestFrom, packet2.requestFrom);
    EXPECT_EQ(packet_in.agentID, packet2.agentID);
    EXPECT_EQ(packet_in.bundleGenTime, packet2.bundleGenTime);
    EXPECT_EQ(packet_in.bidGenTime, packet2.bidGenTime);
    EXPECT_EQ(packet_in.utility, packet2.utility);
    EXPECT_EQ(packet_in.work, packet2.work);
    EXPECT_EQ(packet_in.cost, packet2.cost);
    EXPECT_EQ(packet_in.reward, packet2.reward);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);
    EXPECT_EQ(packet_in.taskGenTime, packet2.taskGenTime);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.priority, packet2.priority);
    EXPECT_EQ(packet_in.seqNum, packet2.seqNum);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_BID_BUNDLE_RCV_PART)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_RCV_PART packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.bundleGenTime = 179.0;
    packet_in.seqNum = 53;

    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_RCV_PART packet1{};
    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_RCV_PART packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.agentID, packet2.agentID);
    EXPECT_EQ(packet1.bundleGenTime, packet2.bundleGenTime);
    EXPECT_EQ(packet1.seqNum, packet2.seqNum);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_BID_BUNDLE_RCV_PART)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_bid_bundle_rcv_part_t packet_c {
         93372036854775807ULL, 179.0, 53
    };

    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_RCV_PART packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.bundleGenTime = 179.0;
    packet_in.seqNum = 53;

    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_RCV_PART packet2{};

    mavlink_msg_auction_bid_bundle_rcv_part_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.agentID, packet2.agentID);
    EXPECT_EQ(packet_in.bundleGenTime, packet2.bundleGenTime);
    EXPECT_EQ(packet_in.seqNum, packet2.seqNum);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_BID_BUNDLE_DONE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_DONE packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.bundleGenTime = 179.0;

    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_DONE packet1{};
    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_DONE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.agentID, packet2.agentID);
    EXPECT_EQ(packet1.bundleGenTime, packet2.bundleGenTime);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_BID_BUNDLE_DONE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_bid_bundle_done_t packet_c {
         93372036854775807ULL, 179.0
    };

    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_DONE packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.bundleGenTime = 179.0;

    mavlink::auctioneer::msg::AUCTION_BID_BUNDLE_DONE packet2{};

    mavlink_msg_auction_bid_bundle_done_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.agentID, packet2.agentID);
    EXPECT_EQ(packet_in.bundleGenTime, packet2.bundleGenTime);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_ACK_BID_BUNDLE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_ACK_BID_BUNDLE packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.bundleGenTime = 179.0;

    mavlink::auctioneer::msg::AUCTION_ACK_BID_BUNDLE packet1{};
    mavlink::auctioneer::msg::AUCTION_ACK_BID_BUNDLE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.agentID, packet2.agentID);
    EXPECT_EQ(packet1.bundleGenTime, packet2.bundleGenTime);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_ACK_BID_BUNDLE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_ack_bid_bundle_t packet_c {
         93372036854775807ULL, 179.0
    };

    mavlink::auctioneer::msg::AUCTION_ACK_BID_BUNDLE packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.bundleGenTime = 179.0;

    mavlink::auctioneer::msg::AUCTION_ACK_BID_BUNDLE packet2{};

    mavlink_msg_auction_ack_bid_bundle_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.agentID, packet2.agentID);
    EXPECT_EQ(packet_in.bundleGenTime, packet2.bundleGenTime);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_NEW_TASK_KEY)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_NEW_TASK_KEY packet_in{};
    packet_in.creatorID = 93372036854775807ULL;
    packet_in.taskID = 963498296;
    packet_in.taskGenTime = 179.0;
    packet_in.type = 65;

    mavlink::auctioneer::msg::AUCTION_NEW_TASK_KEY packet1{};
    mavlink::auctioneer::msg::AUCTION_NEW_TASK_KEY packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
    EXPECT_EQ(packet1.taskGenTime, packet2.taskGenTime);
    EXPECT_EQ(packet1.type, packet2.type);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_NEW_TASK_KEY)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_new_task_key_t packet_c {
         93372036854775807ULL, 179.0, 963498296, 65
    };

    mavlink::auctioneer::msg::AUCTION_NEW_TASK_KEY packet_in{};
    packet_in.creatorID = 93372036854775807ULL;
    packet_in.taskID = 963498296;
    packet_in.taskGenTime = 179.0;
    packet_in.type = 65;

    mavlink::auctioneer::msg::AUCTION_NEW_TASK_KEY packet2{};

    mavlink_msg_auction_new_task_key_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);
    EXPECT_EQ(packet_in.taskGenTime, packet2.taskGenTime);
    EXPECT_EQ(packet_in.type, packet2.type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_TASK_KEY_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_TASK_KEY_ACK packet_in{};
    packet_in.creatorID = 93372036854775807ULL;
    packet_in.taskID = 29;

    mavlink::auctioneer::msg::AUCTION_TASK_KEY_ACK packet1{};
    mavlink::auctioneer::msg::AUCTION_TASK_KEY_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_TASK_KEY_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_task_key_ack_t packet_c {
         93372036854775807ULL, 29
    };

    mavlink::auctioneer::msg::AUCTION_TASK_KEY_ACK packet_in{};
    packet_in.creatorID = 93372036854775807ULL;
    packet_in.taskID = 29;

    mavlink::auctioneer::msg::AUCTION_TASK_KEY_ACK packet2{};

    mavlink_msg_auction_task_key_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_REQUEST_TASK_DESCRIPTOR)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_REQUEST_TASK_DESCRIPTOR packet_in{};
    packet_in.requestFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 53;

    mavlink::auctioneer::msg::AUCTION_REQUEST_TASK_DESCRIPTOR packet1{};
    mavlink::auctioneer::msg::AUCTION_REQUEST_TASK_DESCRIPTOR packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.requestFrom, packet2.requestFrom);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_REQUEST_TASK_DESCRIPTOR)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_request_task_descriptor_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 53
    };

    mavlink::auctioneer::msg::AUCTION_REQUEST_TASK_DESCRIPTOR packet_in{};
    packet_in.requestFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 53;

    mavlink::auctioneer::msg::AUCTION_REQUEST_TASK_DESCRIPTOR packet2{};

    mavlink_msg_auction_request_task_descriptor_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.requestFrom, packet2.requestFrom);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_REQUEST_TASK_DESCRIPTOR_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_REQUEST_TASK_DESCRIPTOR_ACK packet_in{};
    packet_in.creatorID = 93372036854775807ULL;
    packet_in.taskID = 29;

    mavlink::auctioneer::msg::AUCTION_REQUEST_TASK_DESCRIPTOR_ACK packet1{};
    mavlink::auctioneer::msg::AUCTION_REQUEST_TASK_DESCRIPTOR_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_REQUEST_TASK_DESCRIPTOR_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_request_task_descriptor_ack_t packet_c {
         93372036854775807ULL, 29
    };

    mavlink::auctioneer::msg::AUCTION_REQUEST_TASK_DESCRIPTOR_ACK packet_in{};
    packet_in.creatorID = 93372036854775807ULL;
    packet_in.taskID = 29;

    mavlink::auctioneer::msg::AUCTION_REQUEST_TASK_DESCRIPTOR_ACK packet2{};

    mavlink_msg_auction_request_task_descriptor_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_TASK_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_TASK_STATUS packet_in{};
    packet_in.creatorID = 93372036854775807ULL;
    packet_in.taskID = 53;
    packet_in.type = 120;
    packet_in.agentID = 93372036854776311ULL;
    packet_in.data = {{ 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250 }};

    mavlink::auctioneer::msg::AUCTION_TASK_STATUS packet1{};
    mavlink::auctioneer::msg::AUCTION_TASK_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.agentID, packet2.agentID);
    EXPECT_EQ(packet1.data, packet2.data);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_TASK_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_task_status_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 53, 120, { 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250 }
    };

    mavlink::auctioneer::msg::AUCTION_TASK_STATUS packet_in{};
    packet_in.creatorID = 93372036854775807ULL;
    packet_in.taskID = 53;
    packet_in.type = 120;
    packet_in.agentID = 93372036854776311ULL;
    packet_in.data = {{ 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250 }};

    mavlink::auctioneer::msg::AUCTION_TASK_STATUS packet2{};

    mavlink_msg_auction_task_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.agentID, packet2.agentID);
    EXPECT_EQ(packet_in.data, packet2.data);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_TASK_STATUS_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_TASK_STATUS_ACK packet_in{};
    packet_in.creatorID = 93372036854775807ULL;
    packet_in.taskID = 29;
    packet_in.type = 96;

    mavlink::auctioneer::msg::AUCTION_TASK_STATUS_ACK packet1{};
    mavlink::auctioneer::msg::AUCTION_TASK_STATUS_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
    EXPECT_EQ(packet1.type, packet2.type);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_TASK_STATUS_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_task_status_ack_t packet_c {
         93372036854775807ULL, 29, 96
    };

    mavlink::auctioneer::msg::AUCTION_TASK_STATUS_ACK packet_in{};
    packet_in.creatorID = 93372036854775807ULL;
    packet_in.taskID = 29;
    packet_in.type = 96;

    mavlink::auctioneer::msg::AUCTION_TASK_STATUS_ACK packet2{};

    mavlink_msg_auction_task_status_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);
    EXPECT_EQ(packet_in.type, packet2.type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_BID_DESCRIPTOR)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_BID_DESCRIPTOR packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.bidGenTime = 179.0;
    packet_in.utility = 235.0;
    packet_in.work = 291.0;
    packet_in.cost = 347.0;
    packet_in.reward = 403.0;
    packet_in.creatorID = 93372036854778831ULL;
    packet_in.taskID = 221;
    packet_in.taskGenTime = 515.0;
    packet_in.type = 32;
    packet_in.priority = 99;
    packet_in.valid = 166;
    packet_in.rebroadcastTime = 571.0;

    mavlink::auctioneer::msg::AUCTION_BID_DESCRIPTOR packet1{};
    mavlink::auctioneer::msg::AUCTION_BID_DESCRIPTOR packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.agentID, packet2.agentID);
    EXPECT_EQ(packet1.bidGenTime, packet2.bidGenTime);
    EXPECT_EQ(packet1.utility, packet2.utility);
    EXPECT_EQ(packet1.work, packet2.work);
    EXPECT_EQ(packet1.cost, packet2.cost);
    EXPECT_EQ(packet1.reward, packet2.reward);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
    EXPECT_EQ(packet1.taskGenTime, packet2.taskGenTime);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.priority, packet2.priority);
    EXPECT_EQ(packet1.valid, packet2.valid);
    EXPECT_EQ(packet1.rebroadcastTime, packet2.rebroadcastTime);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_BID_DESCRIPTOR)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_bid_descriptor_t packet_c {
         93372036854775807ULL, 179.0, 235.0, 291.0, 347.0, 403.0, 93372036854778831ULL, 515.0, 571.0, 221, 32, 99, 166
    };

    mavlink::auctioneer::msg::AUCTION_BID_DESCRIPTOR packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.bidGenTime = 179.0;
    packet_in.utility = 235.0;
    packet_in.work = 291.0;
    packet_in.cost = 347.0;
    packet_in.reward = 403.0;
    packet_in.creatorID = 93372036854778831ULL;
    packet_in.taskID = 221;
    packet_in.taskGenTime = 515.0;
    packet_in.type = 32;
    packet_in.priority = 99;
    packet_in.valid = 166;
    packet_in.rebroadcastTime = 571.0;

    mavlink::auctioneer::msg::AUCTION_BID_DESCRIPTOR packet2{};

    mavlink_msg_auction_bid_descriptor_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.agentID, packet2.agentID);
    EXPECT_EQ(packet_in.bidGenTime, packet2.bidGenTime);
    EXPECT_EQ(packet_in.utility, packet2.utility);
    EXPECT_EQ(packet_in.work, packet2.work);
    EXPECT_EQ(packet_in.cost, packet2.cost);
    EXPECT_EQ(packet_in.reward, packet2.reward);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);
    EXPECT_EQ(packet_in.taskGenTime, packet2.taskGenTime);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.priority, packet2.priority);
    EXPECT_EQ(packet_in.valid, packet2.valid);
    EXPECT_EQ(packet_in.rebroadcastTime, packet2.rebroadcastTime);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_BID_DESCRIPTOR_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_BID_DESCRIPTOR_ACK packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.bidGenTime = 179.0;
    packet_in.creatorID = 93372036854776815ULL;
    packet_in.taskID = 77;

    mavlink::auctioneer::msg::AUCTION_BID_DESCRIPTOR_ACK packet1{};
    mavlink::auctioneer::msg::AUCTION_BID_DESCRIPTOR_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.agentID, packet2.agentID);
    EXPECT_EQ(packet1.bidGenTime, packet2.bidGenTime);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_BID_DESCRIPTOR_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_bid_descriptor_ack_t packet_c {
         93372036854775807ULL, 179.0, 93372036854776815ULL, 77
    };

    mavlink::auctioneer::msg::AUCTION_BID_DESCRIPTOR_ACK packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.bidGenTime = 179.0;
    packet_in.creatorID = 93372036854776815ULL;
    packet_in.taskID = 77;

    mavlink::auctioneer::msg::AUCTION_BID_DESCRIPTOR_ACK packet2{};

    mavlink_msg_auction_bid_descriptor_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.agentID, packet2.agentID);
    EXPECT_EQ(packet_in.bidGenTime, packet2.bidGenTime);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_SCRUB_BID)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_SCRUB_BID packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.timestamp = 179.0;
    packet_in.creatorID = 93372036854776815ULL;
    packet_in.taskID = 77;

    mavlink::auctioneer::msg::AUCTION_SCRUB_BID packet1{};
    mavlink::auctioneer::msg::AUCTION_SCRUB_BID packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.agentID, packet2.agentID);
    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_SCRUB_BID)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_scrub_bid_t packet_c {
         93372036854775807ULL, 179.0, 93372036854776815ULL, 77
    };

    mavlink::auctioneer::msg::AUCTION_SCRUB_BID packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.timestamp = 179.0;
    packet_in.creatorID = 93372036854776815ULL;
    packet_in.taskID = 77;

    mavlink::auctioneer::msg::AUCTION_SCRUB_BID packet2{};

    mavlink_msg_auction_scrub_bid_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.agentID, packet2.agentID);
    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_SCRUB_BID_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_SCRUB_BID_ACK packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.timestamp = 179.0;
    packet_in.creatorID = 93372036854776815ULL;
    packet_in.taskID = 77;

    mavlink::auctioneer::msg::AUCTION_SCRUB_BID_ACK packet1{};
    mavlink::auctioneer::msg::AUCTION_SCRUB_BID_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.agentID, packet2.agentID);
    EXPECT_EQ(packet1.timestamp, packet2.timestamp);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_SCRUB_BID_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_scrub_bid_ack_t packet_c {
         93372036854775807ULL, 179.0, 93372036854776815ULL, 77
    };

    mavlink::auctioneer::msg::AUCTION_SCRUB_BID_ACK packet_in{};
    packet_in.agentID = 93372036854775807ULL;
    packet_in.timestamp = 179.0;
    packet_in.creatorID = 93372036854776815ULL;
    packet_in.taskID = 77;

    mavlink::auctioneer::msg::AUCTION_SCRUB_BID_ACK packet2{};

    mavlink_msg_auction_scrub_bid_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.agentID, packet2.agentID);
    EXPECT_EQ(packet_in.timestamp, packet2.timestamp);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_TASK_DESCRIPTOR)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 137;
    packet_in.taskGenTime = 235.0;
    packet_in.type = 204;
    packet_in.penalty = 963499544;
    packet_in.reqStart = 291.0;
    packet_in.reqEnd = 347.0;
    packet_in.numParts = 15;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR packet1{};
    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
    EXPECT_EQ(packet1.taskGenTime, packet2.taskGenTime);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.penalty, packet2.penalty);
    EXPECT_EQ(packet1.reqStart, packet2.reqStart);
    EXPECT_EQ(packet1.reqEnd, packet2.reqEnd);
    EXPECT_EQ(packet1.numParts, packet2.numParts);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_TASK_DESCRIPTOR)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_task_descriptor_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 235.0, 291.0, 347.0, 963499544, 137, 204, 15
    };

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 137;
    packet_in.taskGenTime = 235.0;
    packet_in.type = 204;
    packet_in.penalty = 963499544;
    packet_in.reqStart = 291.0;
    packet_in.reqEnd = 347.0;
    packet_in.numParts = 15;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR packet2{};

    mavlink_msg_auction_task_descriptor_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);
    EXPECT_EQ(packet_in.taskGenTime, packet2.taskGenTime);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.penalty, packet2.penalty);
    EXPECT_EQ(packet_in.reqStart, packet2.reqStart);
    EXPECT_EQ(packet_in.reqEnd, packet2.reqEnd);
    EXPECT_EQ(packet_in.numParts, packet2.numParts);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_TASK_DESCRIPTOR_DONE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_DONE packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 53;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_DONE packet1{};
    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_DONE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_TASK_DESCRIPTOR_DONE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_task_descriptor_done_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 53
    };

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_DONE packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 53;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_DONE packet2{};

    mavlink_msg_auction_task_descriptor_done_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_TASK_DESCRIPTOR_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_ACK packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 53;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_ACK packet1{};
    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_TASK_DESCRIPTOR_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_task_descriptor_ack_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 53
    };

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_ACK packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 53;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_ACK packet2{};

    mavlink_msg_auction_task_descriptor_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_TASK_DESCRIPTOR_REQ_PART)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_REQ_PART packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 53;
    packet_in.seqNum = 120;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_REQ_PART packet1{};
    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_REQ_PART packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
    EXPECT_EQ(packet1.seqNum, packet2.seqNum);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_TASK_DESCRIPTOR_REQ_PART)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_task_descriptor_req_part_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 53, 120
    };

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_REQ_PART packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 53;
    packet_in.seqNum = 120;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_REQ_PART packet2{};

    mavlink_msg_auction_task_descriptor_req_part_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);
    EXPECT_EQ(packet_in.seqNum, packet2.seqNum);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_TASK_DESCRIPTOR_LOITER_DATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_LOITER_DATA packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 137;
    packet_in.seqNum = -52;
    packet_in.xPos = 235.0;
    packet_in.yPos = 291.0;
    packet_in.zPos = 347.0;
    packet_in.duration = 963499544;
    packet_in.coordinateType = 15;
    packet_in.coordinateFrame = 82;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_LOITER_DATA packet1{};
    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_LOITER_DATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
    EXPECT_EQ(packet1.seqNum, packet2.seqNum);
    EXPECT_EQ(packet1.xPos, packet2.xPos);
    EXPECT_EQ(packet1.yPos, packet2.yPos);
    EXPECT_EQ(packet1.zPos, packet2.zPos);
    EXPECT_EQ(packet1.duration, packet2.duration);
    EXPECT_EQ(packet1.coordinateType, packet2.coordinateType);
    EXPECT_EQ(packet1.coordinateFrame, packet2.coordinateFrame);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_TASK_DESCRIPTOR_LOITER_DATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_task_descriptor_loiter_data_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 235.0, 291.0, 347.0, 963499544, 137, -52, 15, 82
    };

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_LOITER_DATA packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 137;
    packet_in.seqNum = -52;
    packet_in.xPos = 235.0;
    packet_in.yPos = 291.0;
    packet_in.zPos = 347.0;
    packet_in.duration = 963499544;
    packet_in.coordinateType = 15;
    packet_in.coordinateFrame = 82;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_LOITER_DATA packet2{};

    mavlink_msg_auction_task_descriptor_loiter_data_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);
    EXPECT_EQ(packet_in.seqNum, packet2.seqNum);
    EXPECT_EQ(packet_in.xPos, packet2.xPos);
    EXPECT_EQ(packet_in.yPos, packet2.yPos);
    EXPECT_EQ(packet_in.zPos, packet2.zPos);
    EXPECT_EQ(packet_in.duration, packet2.duration);
    EXPECT_EQ(packet_in.coordinateType, packet2.coordinateType);
    EXPECT_EQ(packet_in.coordinateFrame, packet2.coordinateFrame);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 125;
    packet_in.seqNum = -64;
    packet_in.sensorResolution = 235.0;
    packet_in.overlapHorizontal = 291.0;
    packet_in.overlapVertical = 347.0;
    packet_in.coordinateType = 3;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST packet1{};
    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
    EXPECT_EQ(packet1.seqNum, packet2.seqNum);
    EXPECT_EQ(packet1.sensorResolution, packet2.sensorResolution);
    EXPECT_EQ(packet1.overlapHorizontal, packet2.overlapHorizontal);
    EXPECT_EQ(packet1.overlapVertical, packet2.overlapVertical);
    EXPECT_EQ(packet1.coordinateType, packet2.coordinateType);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_task_descriptor_part_survey_first_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 235.0, 291.0, 347.0, 125, -64, 3
    };

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 125;
    packet_in.seqNum = -64;
    packet_in.sensorResolution = 235.0;
    packet_in.overlapHorizontal = 291.0;
    packet_in.overlapVertical = 347.0;
    packet_in.coordinateType = 3;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST packet2{};

    mavlink_msg_auction_task_descriptor_part_survey_first_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);
    EXPECT_EQ(packet_in.seqNum, packet2.seqNum);
    EXPECT_EQ(packet_in.sensorResolution, packet2.sensorResolution);
    EXPECT_EQ(packet_in.overlapHorizontal, packet2.overlapHorizontal);
    EXPECT_EQ(packet_in.overlapVertical, packet2.overlapVertical);
    EXPECT_EQ(packet_in.coordinateType, packet2.coordinateType);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(auctioneer, AUCTION_TASK_DESCRIPTOR_PART_SURVEY)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_PART_SURVEY packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 101;
    packet_in.seqNum = -88;
    packet_in.xPos = 235.0;
    packet_in.yPos = 291.0;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_PART_SURVEY packet1{};
    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_PART_SURVEY packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet1.creatorID, packet2.creatorID);
    EXPECT_EQ(packet1.taskID, packet2.taskID);
    EXPECT_EQ(packet1.seqNum, packet2.seqNum);
    EXPECT_EQ(packet1.xPos, packet2.xPos);
    EXPECT_EQ(packet1.yPos, packet2.yPos);
}

#ifdef TEST_INTEROP
TEST(auctioneer_interop, AUCTION_TASK_DESCRIPTOR_PART_SURVEY)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auction_task_descriptor_part_survey_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 235.0, 291.0, 101, -88
    };

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_PART_SURVEY packet_in{};
    packet_in.sentFrom = 93372036854775807ULL;
    packet_in.creatorID = 93372036854776311ULL;
    packet_in.taskID = 101;
    packet_in.seqNum = -88;
    packet_in.xPos = 235.0;
    packet_in.yPos = 291.0;

    mavlink::auctioneer::msg::AUCTION_TASK_DESCRIPTOR_PART_SURVEY packet2{};

    mavlink_msg_auction_task_descriptor_part_survey_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.sentFrom, packet2.sentFrom);
    EXPECT_EQ(packet_in.creatorID, packet2.creatorID);
    EXPECT_EQ(packet_in.taskID, packet2.taskID);
    EXPECT_EQ(packet_in.seqNum, packet2.seqNum);
    EXPECT_EQ(packet_in.xPos, packet2.xPos);
    EXPECT_EQ(packet_in.yPos, packet2.yPos);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
