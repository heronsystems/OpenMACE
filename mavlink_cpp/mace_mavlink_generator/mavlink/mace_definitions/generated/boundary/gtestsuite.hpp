/** @file
 *	@brief MAVLink comm testsuite protocol generated from boundary.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "boundary.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(boundary, NEW_BOUNDARY_OBJECT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::boundary::msg::NEW_BOUNDARY_OBJECT packet_in{};
    packet_in.boundary_host_sysid = 5;
    packet_in.boundary_host_compid = 72;
    packet_in.boundary_type = 139;
    packet_in.boundary_identifier = 206;
    packet_in.vehicle_aplicable = 17;
    packet_in.num_vehicles = 84;

    mavlink::boundary::msg::NEW_BOUNDARY_OBJECT packet1{};
    mavlink::boundary::msg::NEW_BOUNDARY_OBJECT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.boundary_host_sysid, packet2.boundary_host_sysid);
    EXPECT_EQ(packet1.boundary_host_compid, packet2.boundary_host_compid);
    EXPECT_EQ(packet1.boundary_type, packet2.boundary_type);
    EXPECT_EQ(packet1.boundary_identifier, packet2.boundary_identifier);
    EXPECT_EQ(packet1.vehicle_aplicable, packet2.vehicle_aplicable);
    EXPECT_EQ(packet1.num_vehicles, packet2.num_vehicles);
}

#ifdef TEST_INTEROP
TEST(boundary_interop, NEW_BOUNDARY_OBJECT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_new_boundary_object_t packet_c {
         5, 72, 139, 206, 17, 84
    };

    mavlink::boundary::msg::NEW_BOUNDARY_OBJECT packet_in{};
    packet_in.boundary_host_sysid = 5;
    packet_in.boundary_host_compid = 72;
    packet_in.boundary_type = 139;
    packet_in.boundary_identifier = 206;
    packet_in.vehicle_aplicable = 17;
    packet_in.num_vehicles = 84;

    mavlink::boundary::msg::NEW_BOUNDARY_OBJECT packet2{};

    mavlink_msg_new_boundary_object_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.boundary_host_sysid, packet2.boundary_host_sysid);
    EXPECT_EQ(packet_in.boundary_host_compid, packet2.boundary_host_compid);
    EXPECT_EQ(packet_in.boundary_type, packet2.boundary_type);
    EXPECT_EQ(packet_in.boundary_identifier, packet2.boundary_identifier);
    EXPECT_EQ(packet_in.vehicle_aplicable, packet2.vehicle_aplicable);
    EXPECT_EQ(packet_in.num_vehicles, packet2.num_vehicles);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(boundary, BOUNDARY_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::boundary::msg::BOUNDARY_ACK packet_in{};
    packet_in.boundary_host_sysid = 5;
    packet_in.boundary_host_compid = 72;
    packet_in.boundary_identifier = 139;
    packet_in.boundary_result = 206;

    mavlink::boundary::msg::BOUNDARY_ACK packet1{};
    mavlink::boundary::msg::BOUNDARY_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.boundary_host_sysid, packet2.boundary_host_sysid);
    EXPECT_EQ(packet1.boundary_host_compid, packet2.boundary_host_compid);
    EXPECT_EQ(packet1.boundary_identifier, packet2.boundary_identifier);
    EXPECT_EQ(packet1.boundary_result, packet2.boundary_result);
}

#ifdef TEST_INTEROP
TEST(boundary_interop, BOUNDARY_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_boundary_ack_t packet_c {
         5, 72, 139, 206
    };

    mavlink::boundary::msg::BOUNDARY_ACK packet_in{};
    packet_in.boundary_host_sysid = 5;
    packet_in.boundary_host_compid = 72;
    packet_in.boundary_identifier = 139;
    packet_in.boundary_result = 206;

    mavlink::boundary::msg::BOUNDARY_ACK packet2{};

    mavlink_msg_boundary_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.boundary_host_sysid, packet2.boundary_host_sysid);
    EXPECT_EQ(packet_in.boundary_host_compid, packet2.boundary_host_compid);
    EXPECT_EQ(packet_in.boundary_identifier, packet2.boundary_identifier);
    EXPECT_EQ(packet_in.boundary_result, packet2.boundary_result);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(boundary, BOUNDARY_REQUEST_LIST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::boundary::msg::BOUNDARY_REQUEST_LIST packet_in{};
    packet_in.boundary_host_sysid = 5;
    packet_in.boundary_host_compid = 72;
    packet_in.boundary_identifier = 139;

    mavlink::boundary::msg::BOUNDARY_REQUEST_LIST packet1{};
    mavlink::boundary::msg::BOUNDARY_REQUEST_LIST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.boundary_host_sysid, packet2.boundary_host_sysid);
    EXPECT_EQ(packet1.boundary_host_compid, packet2.boundary_host_compid);
    EXPECT_EQ(packet1.boundary_identifier, packet2.boundary_identifier);
}

#ifdef TEST_INTEROP
TEST(boundary_interop, BOUNDARY_REQUEST_LIST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_boundary_request_list_t packet_c {
         5, 72, 139
    };

    mavlink::boundary::msg::BOUNDARY_REQUEST_LIST packet_in{};
    packet_in.boundary_host_sysid = 5;
    packet_in.boundary_host_compid = 72;
    packet_in.boundary_identifier = 139;

    mavlink::boundary::msg::BOUNDARY_REQUEST_LIST packet2{};

    mavlink_msg_boundary_request_list_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.boundary_host_sysid, packet2.boundary_host_sysid);
    EXPECT_EQ(packet_in.boundary_host_compid, packet2.boundary_host_compid);
    EXPECT_EQ(packet_in.boundary_identifier, packet2.boundary_identifier);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(boundary, BOUNDARY_COUNT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::boundary::msg::BOUNDARY_COUNT packet_in{};
    packet_in.boundary_host_sysid = 139;
    packet_in.boundary_host_compid = 206;
    packet_in.boundary_identifier = 17;
    packet_in.count = 17235;

    mavlink::boundary::msg::BOUNDARY_COUNT packet1{};
    mavlink::boundary::msg::BOUNDARY_COUNT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.boundary_host_sysid, packet2.boundary_host_sysid);
    EXPECT_EQ(packet1.boundary_host_compid, packet2.boundary_host_compid);
    EXPECT_EQ(packet1.boundary_identifier, packet2.boundary_identifier);
    EXPECT_EQ(packet1.count, packet2.count);
}

#ifdef TEST_INTEROP
TEST(boundary_interop, BOUNDARY_COUNT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_boundary_count_t packet_c {
         17235, 139, 206, 17
    };

    mavlink::boundary::msg::BOUNDARY_COUNT packet_in{};
    packet_in.boundary_host_sysid = 139;
    packet_in.boundary_host_compid = 206;
    packet_in.boundary_identifier = 17;
    packet_in.count = 17235;

    mavlink::boundary::msg::BOUNDARY_COUNT packet2{};

    mavlink_msg_boundary_count_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.boundary_host_sysid, packet2.boundary_host_sysid);
    EXPECT_EQ(packet_in.boundary_host_compid, packet2.boundary_host_compid);
    EXPECT_EQ(packet_in.boundary_identifier, packet2.boundary_identifier);
    EXPECT_EQ(packet_in.count, packet2.count);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(boundary, BOUNDARY_REQUEST_ITEM)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::boundary::msg::BOUNDARY_REQUEST_ITEM packet_in{};
    packet_in.boundary_host_sysid = 139;
    packet_in.boundary_host_compid = 206;
    packet_in.boundary_identifier = 17;
    packet_in.seq = 17235;

    mavlink::boundary::msg::BOUNDARY_REQUEST_ITEM packet1{};
    mavlink::boundary::msg::BOUNDARY_REQUEST_ITEM packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.boundary_host_sysid, packet2.boundary_host_sysid);
    EXPECT_EQ(packet1.boundary_host_compid, packet2.boundary_host_compid);
    EXPECT_EQ(packet1.boundary_identifier, packet2.boundary_identifier);
    EXPECT_EQ(packet1.seq, packet2.seq);
}

#ifdef TEST_INTEROP
TEST(boundary_interop, BOUNDARY_REQUEST_ITEM)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_boundary_request_item_t packet_c {
         17235, 139, 206, 17
    };

    mavlink::boundary::msg::BOUNDARY_REQUEST_ITEM packet_in{};
    packet_in.boundary_host_sysid = 139;
    packet_in.boundary_host_compid = 206;
    packet_in.boundary_identifier = 17;
    packet_in.seq = 17235;

    mavlink::boundary::msg::BOUNDARY_REQUEST_ITEM packet2{};

    mavlink_msg_boundary_request_item_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.boundary_host_sysid, packet2.boundary_host_sysid);
    EXPECT_EQ(packet_in.boundary_host_compid, packet2.boundary_host_compid);
    EXPECT_EQ(packet_in.boundary_identifier, packet2.boundary_identifier);
    EXPECT_EQ(packet_in.seq, packet2.seq);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(boundary, BOUNDARY_ITEM)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::boundary::msg::BOUNDARY_ITEM packet_in{};
    packet_in.boundary_host_sysid = 175;
    packet_in.boundary_host_compid = 242;
    packet_in.boundary_identifier = 53;
    packet_in.frame = 120;
    packet_in.x = 17.0;
    packet_in.y = 45.0;
    packet_in.z = 73.0;
    packet_in.seq = 17859;

    mavlink::boundary::msg::BOUNDARY_ITEM packet1{};
    mavlink::boundary::msg::BOUNDARY_ITEM packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.boundary_host_sysid, packet2.boundary_host_sysid);
    EXPECT_EQ(packet1.boundary_host_compid, packet2.boundary_host_compid);
    EXPECT_EQ(packet1.boundary_identifier, packet2.boundary_identifier);
    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.seq, packet2.seq);
}

#ifdef TEST_INTEROP
TEST(boundary_interop, BOUNDARY_ITEM)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_boundary_item_t packet_c {
         17.0, 45.0, 73.0, 17859, 175, 242, 53, 120
    };

    mavlink::boundary::msg::BOUNDARY_ITEM packet_in{};
    packet_in.boundary_host_sysid = 175;
    packet_in.boundary_host_compid = 242;
    packet_in.boundary_identifier = 53;
    packet_in.frame = 120;
    packet_in.x = 17.0;
    packet_in.y = 45.0;
    packet_in.z = 73.0;
    packet_in.seq = 17859;

    mavlink::boundary::msg::BOUNDARY_ITEM packet2{};

    mavlink_msg_boundary_item_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.boundary_host_sysid, packet2.boundary_host_sysid);
    EXPECT_EQ(packet_in.boundary_host_compid, packet2.boundary_host_compid);
    EXPECT_EQ(packet_in.boundary_identifier, packet2.boundary_identifier);
    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.seq, packet2.seq);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
