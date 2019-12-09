/** @file
 *	@brief MAVLink comm testsuite protocol generated from mission.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "mission.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(mission, NEW_ONBOARD_MISSION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::NEW_ONBOARD_MISSION packet_in{};
    packet_in.mission_system = 5;
    packet_in.mission_creator = 72;
    packet_in.mission_id = 139;
    packet_in.mission_type = 206;
    packet_in.mission_state = 17;

    mavlink::mission::msg::NEW_ONBOARD_MISSION packet1{};
    mavlink::mission::msg::NEW_ONBOARD_MISSION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.mission_system, packet2.mission_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.mission_state, packet2.mission_state);
}

#ifdef TEST_INTEROP
TEST(mission_interop, NEW_ONBOARD_MISSION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_new_onboard_mission_t packet_c {
         5, 72, 139, 206, 17
    };

    mavlink::mission::msg::NEW_ONBOARD_MISSION packet_in{};
    packet_in.mission_system = 5;
    packet_in.mission_creator = 72;
    packet_in.mission_id = 139;
    packet_in.mission_type = 206;
    packet_in.mission_state = 17;

    mavlink::mission::msg::NEW_ONBOARD_MISSION packet2{};

    mavlink_msg_new_onboard_mission_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.mission_system, packet2.mission_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.mission_state, packet2.mission_state);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, NEW_PROPOSED_MISSION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::NEW_PROPOSED_MISSION packet_in{};
    packet_in.target_system = 139;
    packet_in.mission_creator = 206;
    packet_in.mission_id = 17;
    packet_in.mission_type = 84;
    packet_in.mission_state = 151;
    packet_in.count = 17235;

    mavlink::mission::msg::NEW_PROPOSED_MISSION packet1{};
    mavlink::mission::msg::NEW_PROPOSED_MISSION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.mission_state, packet2.mission_state);
    EXPECT_EQ(packet1.count, packet2.count);
}

#ifdef TEST_INTEROP
TEST(mission_interop, NEW_PROPOSED_MISSION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_new_proposed_mission_t packet_c {
         17235, 139, 206, 17, 84, 151
    };

    mavlink::mission::msg::NEW_PROPOSED_MISSION packet_in{};
    packet_in.target_system = 139;
    packet_in.mission_creator = 206;
    packet_in.mission_id = 17;
    packet_in.mission_type = 84;
    packet_in.mission_state = 151;
    packet_in.count = 17235;

    mavlink::mission::msg::NEW_PROPOSED_MISSION packet2{};

    mavlink_msg_new_proposed_mission_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.mission_state, packet2.mission_state);
    EXPECT_EQ(packet_in.count, packet2.count);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_ACK packet_in{};
    packet_in.mission_system = 5;
    packet_in.mission_creator = 72;
    packet_in.mission_id = 139;
    packet_in.mission_type = 206;
    packet_in.prev_mission_state = 17;
    packet_in.mission_result = 84;
    packet_in.cur_mission_state = 151;

    mavlink::mission::msg::MISSION_ACK packet1{};
    mavlink::mission::msg::MISSION_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.mission_system, packet2.mission_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.prev_mission_state, packet2.prev_mission_state);
    EXPECT_EQ(packet1.mission_result, packet2.mission_result);
    EXPECT_EQ(packet1.cur_mission_state, packet2.cur_mission_state);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_ack_t packet_c {
         5, 72, 139, 206, 17, 84, 151
    };

    mavlink::mission::msg::MISSION_ACK packet_in{};
    packet_in.mission_system = 5;
    packet_in.mission_creator = 72;
    packet_in.mission_id = 139;
    packet_in.mission_type = 206;
    packet_in.prev_mission_state = 17;
    packet_in.mission_result = 84;
    packet_in.cur_mission_state = 151;

    mavlink::mission::msg::MISSION_ACK packet2{};

    mavlink_msg_mission_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.mission_system, packet2.mission_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.prev_mission_state, packet2.prev_mission_state);
    EXPECT_EQ(packet_in.mission_result, packet2.mission_result);
    EXPECT_EQ(packet_in.cur_mission_state, packet2.cur_mission_state);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_REQUEST_LIST_GENERIC)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_REQUEST_LIST_GENERIC packet_in{};
    packet_in.mission_system = 5;
    packet_in.mission_type = 72;
    packet_in.mission_state = 139;

    mavlink::mission::msg::MISSION_REQUEST_LIST_GENERIC packet1{};
    mavlink::mission::msg::MISSION_REQUEST_LIST_GENERIC packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.mission_system, packet2.mission_system);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.mission_state, packet2.mission_state);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_REQUEST_LIST_GENERIC)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_request_list_generic_t packet_c {
         5, 72, 139
    };

    mavlink::mission::msg::MISSION_REQUEST_LIST_GENERIC packet_in{};
    packet_in.mission_system = 5;
    packet_in.mission_type = 72;
    packet_in.mission_state = 139;

    mavlink::mission::msg::MISSION_REQUEST_LIST_GENERIC packet2{};

    mavlink_msg_mission_request_list_generic_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.mission_system, packet2.mission_system);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.mission_state, packet2.mission_state);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_REQUEST_LIST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_REQUEST_LIST packet_in{};
    packet_in.mission_system = 5;
    packet_in.mission_creator = 72;
    packet_in.mission_id = 139;
    packet_in.mission_type = 206;
    packet_in.mission_state = 17;

    mavlink::mission::msg::MISSION_REQUEST_LIST packet1{};
    mavlink::mission::msg::MISSION_REQUEST_LIST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.mission_system, packet2.mission_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.mission_state, packet2.mission_state);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_REQUEST_LIST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_request_list_t packet_c {
         5, 72, 139, 206, 17
    };

    mavlink::mission::msg::MISSION_REQUEST_LIST packet_in{};
    packet_in.mission_system = 5;
    packet_in.mission_creator = 72;
    packet_in.mission_id = 139;
    packet_in.mission_type = 206;
    packet_in.mission_state = 17;

    mavlink::mission::msg::MISSION_REQUEST_LIST packet2{};

    mavlink_msg_mission_request_list_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.mission_system, packet2.mission_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.mission_state, packet2.mission_state);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_COUNT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_COUNT packet_in{};
    packet_in.target_system = 139;
    packet_in.mission_system = 206;
    packet_in.mission_creator = 17;
    packet_in.mission_id = 84;
    packet_in.mission_type = 151;
    packet_in.mission_state = 218;
    packet_in.count = 17235;

    mavlink::mission::msg::MISSION_COUNT packet1{};
    mavlink::mission::msg::MISSION_COUNT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.mission_system, packet2.mission_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.mission_state, packet2.mission_state);
    EXPECT_EQ(packet1.count, packet2.count);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_COUNT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_count_t packet_c {
         17235, 139, 206, 17, 84, 151, 218
    };

    mavlink::mission::msg::MISSION_COUNT packet_in{};
    packet_in.target_system = 139;
    packet_in.mission_system = 206;
    packet_in.mission_creator = 17;
    packet_in.mission_id = 84;
    packet_in.mission_type = 151;
    packet_in.mission_state = 218;
    packet_in.count = 17235;

    mavlink::mission::msg::MISSION_COUNT packet2{};

    mavlink_msg_mission_count_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.mission_system, packet2.mission_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.mission_state, packet2.mission_state);
    EXPECT_EQ(packet_in.count, packet2.count);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_REQUEST_ITEM)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_REQUEST_ITEM packet_in{};
    packet_in.target_system = 139;
    packet_in.mission_system = 206;
    packet_in.mission_creator = 17;
    packet_in.mission_id = 84;
    packet_in.mission_type = 151;
    packet_in.mission_state = 218;
    packet_in.seq = 17235;

    mavlink::mission::msg::MISSION_REQUEST_ITEM packet1{};
    mavlink::mission::msg::MISSION_REQUEST_ITEM packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.mission_system, packet2.mission_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.mission_state, packet2.mission_state);
    EXPECT_EQ(packet1.seq, packet2.seq);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_REQUEST_ITEM)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_request_item_t packet_c {
         17235, 139, 206, 17, 84, 151, 218
    };

    mavlink::mission::msg::MISSION_REQUEST_ITEM packet_in{};
    packet_in.target_system = 139;
    packet_in.mission_system = 206;
    packet_in.mission_creator = 17;
    packet_in.mission_id = 84;
    packet_in.mission_type = 151;
    packet_in.mission_state = 218;
    packet_in.seq = 17235;

    mavlink::mission::msg::MISSION_REQUEST_ITEM packet2{};

    mavlink_msg_mission_request_item_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.mission_system, packet2.mission_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.mission_state, packet2.mission_state);
    EXPECT_EQ(packet_in.seq, packet2.seq);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_ITEM)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_ITEM packet_in{};
    packet_in.target_system = 101;
    packet_in.mission_system = 168;
    packet_in.mission_creator = 235;
    packet_in.mission_id = 46;
    packet_in.mission_type = 113;
    packet_in.mission_state = 180;
    packet_in.seq = 18691;
    packet_in.frame = 247;
    packet_in.command = 18795;
    packet_in.current = 58;
    packet_in.autocontinue = 125;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.x = 129.0;
    packet_in.y = 157.0;
    packet_in.z = 185.0;

    mavlink::mission::msg::MISSION_ITEM packet1{};
    mavlink::mission::msg::MISSION_ITEM packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.mission_system, packet2.mission_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.mission_state, packet2.mission_state);
    EXPECT_EQ(packet1.seq, packet2.seq);
    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.command, packet2.command);
    EXPECT_EQ(packet1.current, packet2.current);
    EXPECT_EQ(packet1.autocontinue, packet2.autocontinue);
    EXPECT_EQ(packet1.param1, packet2.param1);
    EXPECT_EQ(packet1.param2, packet2.param2);
    EXPECT_EQ(packet1.param3, packet2.param3);
    EXPECT_EQ(packet1.param4, packet2.param4);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_ITEM)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_item_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 18691, 18795, 101, 168, 235, 46, 113, 180, 247, 58, 125
    };

    mavlink::mission::msg::MISSION_ITEM packet_in{};
    packet_in.target_system = 101;
    packet_in.mission_system = 168;
    packet_in.mission_creator = 235;
    packet_in.mission_id = 46;
    packet_in.mission_type = 113;
    packet_in.mission_state = 180;
    packet_in.seq = 18691;
    packet_in.frame = 247;
    packet_in.command = 18795;
    packet_in.current = 58;
    packet_in.autocontinue = 125;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.x = 129.0;
    packet_in.y = 157.0;
    packet_in.z = 185.0;

    mavlink::mission::msg::MISSION_ITEM packet2{};

    mavlink_msg_mission_item_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.mission_system, packet2.mission_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.mission_state, packet2.mission_state);
    EXPECT_EQ(packet_in.seq, packet2.seq);
    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.command, packet2.command);
    EXPECT_EQ(packet_in.current, packet2.current);
    EXPECT_EQ(packet_in.autocontinue, packet2.autocontinue);
    EXPECT_EQ(packet_in.param1, packet2.param1);
    EXPECT_EQ(packet_in.param2, packet2.param2);
    EXPECT_EQ(packet_in.param3, packet2.param3);
    EXPECT_EQ(packet_in.param4, packet2.param4);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_REQUEST_PARTIAL_LIST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_REQUEST_PARTIAL_LIST packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.start_index = 17235;
    packet_in.end_index = 17339;
    packet_in.mission_type = 151;

    mavlink::mission::msg::MISSION_REQUEST_PARTIAL_LIST packet1{};
    mavlink::mission::msg::MISSION_REQUEST_PARTIAL_LIST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.start_index, packet2.start_index);
    EXPECT_EQ(packet1.end_index, packet2.end_index);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_REQUEST_PARTIAL_LIST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_request_partial_list_t packet_c {
         17235, 17339, 17, 84, 151
    };

    mavlink::mission::msg::MISSION_REQUEST_PARTIAL_LIST packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.start_index = 17235;
    packet_in.end_index = 17339;
    packet_in.mission_type = 151;

    mavlink::mission::msg::MISSION_REQUEST_PARTIAL_LIST packet2{};

    mavlink_msg_mission_request_partial_list_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.start_index, packet2.start_index);
    EXPECT_EQ(packet_in.end_index, packet2.end_index);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_WRITE_PARTIAL_LIST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_WRITE_PARTIAL_LIST packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.start_index = 17235;
    packet_in.end_index = 17339;
    packet_in.mission_type = 151;

    mavlink::mission::msg::MISSION_WRITE_PARTIAL_LIST packet1{};
    mavlink::mission::msg::MISSION_WRITE_PARTIAL_LIST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.start_index, packet2.start_index);
    EXPECT_EQ(packet1.end_index, packet2.end_index);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_WRITE_PARTIAL_LIST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_write_partial_list_t packet_c {
         17235, 17339, 17, 84, 151
    };

    mavlink::mission::msg::MISSION_WRITE_PARTIAL_LIST packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.start_index = 17235;
    packet_in.end_index = 17339;
    packet_in.mission_type = 151;

    mavlink::mission::msg::MISSION_WRITE_PARTIAL_LIST packet2{};

    mavlink_msg_mission_write_partial_list_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.start_index, packet2.start_index);
    EXPECT_EQ(packet_in.end_index, packet2.end_index);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, STARTING_CURRENT_MISSION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::STARTING_CURRENT_MISSION packet_in{};
    packet_in.mission_system = 5;
    packet_in.mission_creator = 72;
    packet_in.mission_id = 139;
    packet_in.mission_type = 206;

    mavlink::mission::msg::STARTING_CURRENT_MISSION packet1{};
    mavlink::mission::msg::STARTING_CURRENT_MISSION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.mission_system, packet2.mission_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
}

#ifdef TEST_INTEROP
TEST(mission_interop, STARTING_CURRENT_MISSION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_starting_current_mission_t packet_c {
         5, 72, 139, 206
    };

    mavlink::mission::msg::STARTING_CURRENT_MISSION packet_in{};
    packet_in.mission_system = 5;
    packet_in.mission_creator = 72;
    packet_in.mission_id = 139;
    packet_in.mission_type = 206;

    mavlink::mission::msg::STARTING_CURRENT_MISSION packet2{};

    mavlink_msg_starting_current_mission_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.mission_system, packet2.mission_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_SET_CURRENT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_SET_CURRENT packet_in{};
    packet_in.mission_system = 139;
    packet_in.mission_creator = 206;
    packet_in.mission_id = 17;
    packet_in.seq = 17235;

    mavlink::mission::msg::MISSION_SET_CURRENT packet1{};
    mavlink::mission::msg::MISSION_SET_CURRENT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.mission_system, packet2.mission_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.seq, packet2.seq);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_SET_CURRENT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_set_current_t packet_c {
         17235, 139, 206, 17
    };

    mavlink::mission::msg::MISSION_SET_CURRENT packet_in{};
    packet_in.mission_system = 139;
    packet_in.mission_creator = 206;
    packet_in.mission_id = 17;
    packet_in.seq = 17235;

    mavlink::mission::msg::MISSION_SET_CURRENT packet2{};

    mavlink_msg_mission_set_current_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.mission_system, packet2.mission_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.seq, packet2.seq);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_ITEM_CURRENT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_ITEM_CURRENT packet_in{};
    packet_in.mission_system = 139;
    packet_in.mission_creator = 206;
    packet_in.mission_id = 17;
    packet_in.mission_type = 84;
    packet_in.mission_state = 151;
    packet_in.seq = 17235;

    mavlink::mission::msg::MISSION_ITEM_CURRENT packet1{};
    mavlink::mission::msg::MISSION_ITEM_CURRENT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.mission_system, packet2.mission_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.mission_state, packet2.mission_state);
    EXPECT_EQ(packet1.seq, packet2.seq);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_ITEM_CURRENT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_item_current_t packet_c {
         17235, 139, 206, 17, 84, 151
    };

    mavlink::mission::msg::MISSION_ITEM_CURRENT packet_in{};
    packet_in.mission_system = 139;
    packet_in.mission_creator = 206;
    packet_in.mission_id = 17;
    packet_in.mission_type = 84;
    packet_in.mission_state = 151;
    packet_in.seq = 17235;

    mavlink::mission::msg::MISSION_ITEM_CURRENT packet2{};

    mavlink_msg_mission_item_current_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.mission_system, packet2.mission_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.mission_state, packet2.mission_state);
    EXPECT_EQ(packet_in.seq, packet2.seq);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_ITEM_REACHED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_ITEM_REACHED packet_in{};
    packet_in.mission_system = 139;
    packet_in.mission_creator = 206;
    packet_in.mission_id = 17;
    packet_in.mission_type = 84;
    packet_in.mission_state = 151;
    packet_in.seq = 17235;

    mavlink::mission::msg::MISSION_ITEM_REACHED packet1{};
    mavlink::mission::msg::MISSION_ITEM_REACHED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.mission_system, packet2.mission_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.mission_state, packet2.mission_state);
    EXPECT_EQ(packet1.seq, packet2.seq);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_ITEM_REACHED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_item_reached_t packet_c {
         17235, 139, 206, 17, 84, 151
    };

    mavlink::mission::msg::MISSION_ITEM_REACHED packet_in{};
    packet_in.mission_system = 139;
    packet_in.mission_creator = 206;
    packet_in.mission_id = 17;
    packet_in.mission_type = 84;
    packet_in.mission_state = 151;
    packet_in.seq = 17235;

    mavlink::mission::msg::MISSION_ITEM_REACHED packet2{};

    mavlink_msg_mission_item_reached_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.mission_system, packet2.mission_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.mission_state, packet2.mission_state);
    EXPECT_EQ(packet_in.seq, packet2.seq);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_CLEAR)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_CLEAR packet_in{};
    packet_in.target_system = 5;
    packet_in.mission_creator = 72;
    packet_in.mission_id = 139;
    packet_in.mission_type = 206;

    mavlink::mission::msg::MISSION_CLEAR packet1{};
    mavlink::mission::msg::MISSION_CLEAR packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_CLEAR)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_clear_t packet_c {
         5, 72, 139, 206
    };

    mavlink::mission::msg::MISSION_CLEAR packet_in{};
    packet_in.target_system = 5;
    packet_in.mission_creator = 72;
    packet_in.mission_id = 139;
    packet_in.mission_type = 206;

    mavlink::mission::msg::MISSION_CLEAR packet2{};

    mavlink_msg_mission_clear_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_EXE_STATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_EXE_STATE packet_in{};
    packet_in.mission_system = 5;
    packet_in.mission_creator = 72;
    packet_in.mission_id = 139;
    packet_in.mission_type = 206;
    packet_in.mission_state = 17;

    mavlink::mission::msg::MISSION_EXE_STATE packet1{};
    mavlink::mission::msg::MISSION_EXE_STATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.mission_system, packet2.mission_system);
    EXPECT_EQ(packet1.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet1.mission_id, packet2.mission_id);
    EXPECT_EQ(packet1.mission_type, packet2.mission_type);
    EXPECT_EQ(packet1.mission_state, packet2.mission_state);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_EXE_STATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_exe_state_t packet_c {
         5, 72, 139, 206, 17
    };

    mavlink::mission::msg::MISSION_EXE_STATE packet_in{};
    packet_in.mission_system = 5;
    packet_in.mission_creator = 72;
    packet_in.mission_id = 139;
    packet_in.mission_type = 206;
    packet_in.mission_state = 17;

    mavlink::mission::msg::MISSION_EXE_STATE packet2{};

    mavlink_msg_mission_exe_state_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.mission_system, packet2.mission_system);
    EXPECT_EQ(packet_in.mission_creator, packet2.mission_creator);
    EXPECT_EQ(packet_in.mission_id, packet2.mission_id);
    EXPECT_EQ(packet_in.mission_type, packet2.mission_type);
    EXPECT_EQ(packet_in.mission_state, packet2.mission_state);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, MISSION_REQUEST_HOME)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::MISSION_REQUEST_HOME packet_in{};
    packet_in.target_system = 5;

    mavlink::mission::msg::MISSION_REQUEST_HOME packet1{};
    mavlink::mission::msg::MISSION_REQUEST_HOME packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
}

#ifdef TEST_INTEROP
TEST(mission_interop, MISSION_REQUEST_HOME)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_mission_request_home_t packet_c {
         5
    };

    mavlink::mission::msg::MISSION_REQUEST_HOME packet_in{};
    packet_in.target_system = 5;

    mavlink::mission::msg::MISSION_REQUEST_HOME packet2{};

    mavlink_msg_mission_request_home_encode(1, 1, &msg, &packet_c);

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

TEST(mission, HOME_POSITION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::HOME_POSITION packet_in{};
    packet_in.validity = 161;
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;
    packet_in.x = 101.0;
    packet_in.y = 129.0;
    packet_in.z = 157.0;
    packet_in.q = {{ 185.0, 186.0, 187.0, 188.0 }};
    packet_in.approach_x = 297.0;
    packet_in.approach_y = 325.0;
    packet_in.approach_z = 353.0;

    mavlink::mission::msg::HOME_POSITION packet1{};
    mavlink::mission::msg::HOME_POSITION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.validity, packet2.validity);
    EXPECT_EQ(packet1.latitude, packet2.latitude);
    EXPECT_EQ(packet1.longitude, packet2.longitude);
    EXPECT_EQ(packet1.altitude, packet2.altitude);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.approach_x, packet2.approach_x);
    EXPECT_EQ(packet1.approach_y, packet2.approach_y);
    EXPECT_EQ(packet1.approach_z, packet2.approach_z);
}

#ifdef TEST_INTEROP
TEST(mission_interop, HOME_POSITION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_home_position_t packet_c {
         963497464, 963497672, 963497880, 101.0, 129.0, 157.0, { 185.0, 186.0, 187.0, 188.0 }, 297.0, 325.0, 353.0, 161
    };

    mavlink::mission::msg::HOME_POSITION packet_in{};
    packet_in.validity = 161;
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;
    packet_in.x = 101.0;
    packet_in.y = 129.0;
    packet_in.z = 157.0;
    packet_in.q = {{ 185.0, 186.0, 187.0, 188.0 }};
    packet_in.approach_x = 297.0;
    packet_in.approach_y = 325.0;
    packet_in.approach_z = 353.0;

    mavlink::mission::msg::HOME_POSITION packet2{};

    mavlink_msg_home_position_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.validity, packet2.validity);
    EXPECT_EQ(packet_in.latitude, packet2.latitude);
    EXPECT_EQ(packet_in.longitude, packet2.longitude);
    EXPECT_EQ(packet_in.altitude, packet2.altitude);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.approach_x, packet2.approach_x);
    EXPECT_EQ(packet_in.approach_y, packet2.approach_y);
    EXPECT_EQ(packet_in.approach_z, packet2.approach_z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, SET_HOME_POSITION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::SET_HOME_POSITION packet_in{};
    packet_in.target_system = 161;
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;
    packet_in.x = 101.0;
    packet_in.y = 129.0;
    packet_in.z = 157.0;
    packet_in.q = {{ 185.0, 186.0, 187.0, 188.0 }};
    packet_in.approach_x = 297.0;
    packet_in.approach_y = 325.0;
    packet_in.approach_z = 353.0;

    mavlink::mission::msg::SET_HOME_POSITION packet1{};
    mavlink::mission::msg::SET_HOME_POSITION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.latitude, packet2.latitude);
    EXPECT_EQ(packet1.longitude, packet2.longitude);
    EXPECT_EQ(packet1.altitude, packet2.altitude);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.approach_x, packet2.approach_x);
    EXPECT_EQ(packet1.approach_y, packet2.approach_y);
    EXPECT_EQ(packet1.approach_z, packet2.approach_z);
}

#ifdef TEST_INTEROP
TEST(mission_interop, SET_HOME_POSITION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_set_home_position_t packet_c {
         963497464, 963497672, 963497880, 101.0, 129.0, 157.0, { 185.0, 186.0, 187.0, 188.0 }, 297.0, 325.0, 353.0, 161
    };

    mavlink::mission::msg::SET_HOME_POSITION packet_in{};
    packet_in.target_system = 161;
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;
    packet_in.x = 101.0;
    packet_in.y = 129.0;
    packet_in.z = 157.0;
    packet_in.q = {{ 185.0, 186.0, 187.0, 188.0 }};
    packet_in.approach_x = 297.0;
    packet_in.approach_y = 325.0;
    packet_in.approach_z = 353.0;

    mavlink::mission::msg::SET_HOME_POSITION packet2{};

    mavlink_msg_set_home_position_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.latitude, packet2.latitude);
    EXPECT_EQ(packet_in.longitude, packet2.longitude);
    EXPECT_EQ(packet_in.altitude, packet2.altitude);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.approach_x, packet2.approach_x);
    EXPECT_EQ(packet_in.approach_y, packet2.approach_y);
    EXPECT_EQ(packet_in.approach_z, packet2.approach_z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, HOME_POSITION_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::HOME_POSITION_ACK packet_in{};
    packet_in.target_system = 5;
    packet_in.ack = 72;

    mavlink::mission::msg::HOME_POSITION_ACK packet1{};
    mavlink::mission::msg::HOME_POSITION_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.ack, packet2.ack);
}

#ifdef TEST_INTEROP
TEST(mission_interop, HOME_POSITION_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_home_position_ack_t packet_c {
         5, 72
    };

    mavlink::mission::msg::HOME_POSITION_ACK packet_in{};
    packet_in.target_system = 5;
    packet_in.ack = 72;

    mavlink::mission::msg::HOME_POSITION_ACK packet2{};

    mavlink_msg_home_position_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.ack, packet2.ack);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(mission, GUIDED_TARGET_STATS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::mission::msg::GUIDED_TARGET_STATS packet_in{};
    packet_in.x = 17.0;
    packet_in.y = 45.0;
    packet_in.z = 73.0;
    packet_in.distance = 101.0;
    packet_in.coordinate_frame = 53;
    packet_in.state = 120;

    mavlink::mission::msg::GUIDED_TARGET_STATS packet1{};
    mavlink::mission::msg::GUIDED_TARGET_STATS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.distance, packet2.distance);
    EXPECT_EQ(packet1.coordinate_frame, packet2.coordinate_frame);
    EXPECT_EQ(packet1.state, packet2.state);
}

#ifdef TEST_INTEROP
TEST(mission_interop, GUIDED_TARGET_STATS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_guided_target_stats_t packet_c {
         17.0, 45.0, 73.0, 101.0, 53, 120
    };

    mavlink::mission::msg::GUIDED_TARGET_STATS packet_in{};
    packet_in.x = 17.0;
    packet_in.y = 45.0;
    packet_in.z = 73.0;
    packet_in.distance = 101.0;
    packet_in.coordinate_frame = 53;
    packet_in.state = 120;

    mavlink::mission::msg::GUIDED_TARGET_STATS packet2{};

    mavlink_msg_guided_target_stats_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.distance, packet2.distance);
    EXPECT_EQ(packet_in.coordinate_frame, packet2.coordinate_frame);
    EXPECT_EQ(packet_in.state, packet2.state);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
