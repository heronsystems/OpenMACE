/** @file
 *	@brief MAVLink comm testsuite protocol generated from mace_common.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "mace_common.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(mace_common, VEHICLE_SYNC)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mace_common::msg::VEHICLE_SYNC packet_in{};
    packet_in.target_system = 5;

    mavlink::mace_common::msg::VEHICLE_SYNC packet1{};
    mavlink::mace_common::msg::VEHICLE_SYNC packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
}

#ifdef TEST_INTEROP
TEST(mace_common_interop, VEHICLE_SYNC)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vehicle_sync_t packet_c {
         5
    };

    mavlink::mace_common::msg::VEHICLE_SYNC packet_in{};
    packet_in.target_system = 5;

    mavlink::mace_common::msg::VEHICLE_SYNC packet2{};

    mavlink_msg_vehicle_sync_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mace_common, ROI_AG)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mace_common::msg::ROI_AG packet_in{};
    packet_in.target_system = 53;
    packet_in.target_component = 120;
    packet_in.point_discovery = 187;
    packet_in.stress_threshold = 254;
    packet_in.stress_value = 17.0;
    packet_in.frame = 65;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;

    mavlink::mace_common::msg::ROI_AG packet1{};
    mavlink::mace_common::msg::ROI_AG packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.point_discovery, packet2.point_discovery);
    EXPECT_EQ(packet1.stress_threshold, packet2.stress_threshold);
    EXPECT_EQ(packet1.stress_value, packet2.stress_value);
    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
}

#ifdef TEST_INTEROP
TEST(mace_common_interop, ROI_AG)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_roi_ag_t packet_c {
         17.0, 45.0, 73.0, 101.0, 53, 120, 187, 254, 65
    };

    mavlink::mace_common::msg::ROI_AG packet_in{};
    packet_in.target_system = 53;
    packet_in.target_component = 120;
    packet_in.point_discovery = 187;
    packet_in.stress_threshold = 254;
    packet_in.stress_value = 17.0;
    packet_in.frame = 65;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;

    mavlink::mace_common::msg::ROI_AG packet2{};

    mavlink_msg_roi_ag_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.point_discovery, packet2.point_discovery);
    EXPECT_EQ(packet_in.stress_threshold, packet2.stress_threshold);
    EXPECT_EQ(packet_in.stress_value, packet2.stress_value);
    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
