/** @file
 *	@brief MAVLink comm testsuite protocol generated from common.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "common.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(common, HEARTBEAT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::HEARTBEAT packet_in{};
    packet_in.protocol = 5;
    packet_in.type = 72;
    packet_in.autopilot = 139;
    packet_in.mission_state = 206;
    packet_in.mace_companion = 17;
    packet_in.mavlink_version = 3;
    packet_in.mavlinkID = 151;

    mavlink::common::msg::HEARTBEAT packet1{};
    mavlink::common::msg::HEARTBEAT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.protocol, packet2.protocol);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.autopilot, packet2.autopilot);
    EXPECT_EQ(packet1.mission_state, packet2.mission_state);
    EXPECT_EQ(packet1.mace_companion, packet2.mace_companion);
    EXPECT_EQ(packet1.mavlink_version, packet2.mavlink_version);
    EXPECT_EQ(packet1.mavlinkID, packet2.mavlinkID);
}

#ifdef TEST_INTEROP
TEST(common_interop, HEARTBEAT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_heartbeat_t packet_c {
         5, 72, 139, 206, 17, 3, 151
    };

    mavlink::common::msg::HEARTBEAT packet_in{};
    packet_in.protocol = 5;
    packet_in.type = 72;
    packet_in.autopilot = 139;
    packet_in.mission_state = 206;
    packet_in.mace_companion = 17;
    packet_in.mavlink_version = 3;
    packet_in.mavlinkID = 151;

    mavlink::common::msg::HEARTBEAT packet2{};

    mavlink_msg_heartbeat_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.protocol, packet2.protocol);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.autopilot, packet2.autopilot);
    EXPECT_EQ(packet_in.mission_state, packet2.mission_state);
    EXPECT_EQ(packet_in.mace_companion, packet2.mace_companion);
    EXPECT_EQ(packet_in.mavlink_version, packet2.mavlink_version);
    EXPECT_EQ(packet_in.mavlinkID, packet2.mavlinkID);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, VEHICLE_MODE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::VEHICLE_MODE packet_in{};
    packet_in.vehicle_mode = to_char_array("ABCDEFGHI");

    mavlink::common::msg::VEHICLE_MODE packet1{};
    mavlink::common::msg::VEHICLE_MODE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.vehicle_mode, packet2.vehicle_mode);
}

#ifdef TEST_INTEROP
TEST(common_interop, VEHICLE_MODE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vehicle_mode_t packet_c {
         "ABCDEFGHI"
    };

    mavlink::common::msg::VEHICLE_MODE packet_in{};
    packet_in.vehicle_mode = to_char_array("ABCDEFGHI");

    mavlink::common::msg::VEHICLE_MODE packet2{};

    mavlink_msg_vehicle_mode_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.vehicle_mode, packet2.vehicle_mode);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, VEHICLE_ARMED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::VEHICLE_ARMED packet_in{};
    packet_in.vehicle_armed = 5;

    mavlink::common::msg::VEHICLE_ARMED packet1{};
    mavlink::common::msg::VEHICLE_ARMED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.vehicle_armed, packet2.vehicle_armed);
}

#ifdef TEST_INTEROP
TEST(common_interop, VEHICLE_ARMED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vehicle_armed_t packet_c {
         5
    };

    mavlink::common::msg::VEHICLE_ARMED packet_in{};
    packet_in.vehicle_armed = 5;

    mavlink::common::msg::VEHICLE_ARMED packet2{};

    mavlink_msg_vehicle_armed_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.vehicle_armed, packet2.vehicle_armed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, BATTERY_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::BATTERY_STATUS packet_in{};
    packet_in.id = 151;
    packet_in.battery_function = 218;
    packet_in.type = 29;
    packet_in.temperature = 17235;
    packet_in.voltage_battery = 17339;
    packet_in.current_battery = 17443;
    packet_in.battery_remaining = 96;

    mavlink::common::msg::BATTERY_STATUS packet1{};
    mavlink::common::msg::BATTERY_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.battery_function, packet2.battery_function);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.voltage_battery, packet2.voltage_battery);
    EXPECT_EQ(packet1.current_battery, packet2.current_battery);
    EXPECT_EQ(packet1.battery_remaining, packet2.battery_remaining);
}

#ifdef TEST_INTEROP
TEST(common_interop, BATTERY_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_battery_status_t packet_c {
         17235, 17339, 17443, 151, 218, 29, 96
    };

    mavlink::common::msg::BATTERY_STATUS packet_in{};
    packet_in.id = 151;
    packet_in.battery_function = 218;
    packet_in.type = 29;
    packet_in.temperature = 17235;
    packet_in.voltage_battery = 17339;
    packet_in.current_battery = 17443;
    packet_in.battery_remaining = 96;

    mavlink::common::msg::BATTERY_STATUS packet2{};

    mavlink_msg_battery_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.battery_function, packet2.battery_function);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.voltage_battery, packet2.voltage_battery);
    EXPECT_EQ(packet_in.current_battery, packet2.current_battery);
    EXPECT_EQ(packet_in.battery_remaining, packet2.battery_remaining);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SYSTEM_TIME)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SYSTEM_TIME packet_in{};
    packet_in.time_unix_usec = 93372036854775807ULL;
    packet_in.time_boot_ms = 963497880;

    mavlink::common::msg::SYSTEM_TIME packet1{};
    mavlink::common::msg::SYSTEM_TIME packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_unix_usec, packet2.time_unix_usec);
    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
}

#ifdef TEST_INTEROP
TEST(common_interop, SYSTEM_TIME)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_system_time_t packet_c {
         93372036854775807ULL, 963497880
    };

    mavlink::common::msg::SYSTEM_TIME packet_in{};
    packet_in.time_unix_usec = 93372036854775807ULL;
    packet_in.time_boot_ms = 963497880;

    mavlink::common::msg::SYSTEM_TIME packet2{};

    mavlink_msg_system_time_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_unix_usec, packet2.time_unix_usec);
    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PING)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PING packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.seq = 963497880;
    packet_in.target_system = 41;
    packet_in.target_component = 108;

    mavlink::common::msg::PING packet1{};
    mavlink::common::msg::PING packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.seq, packet2.seq);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(common_interop, PING)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_ping_t packet_c {
         93372036854775807ULL, 963497880, 41, 108
    };

    mavlink::common::msg::PING packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.seq = 963497880;
    packet_in.target_system = 41;
    packet_in.target_component = 108;

    mavlink::common::msg::PING packet2{};

    mavlink_msg_ping_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.seq, packet2.seq);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CHANGE_OPERATOR_CONTROL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL packet_in{};
    packet_in.target_system = 5;
    packet_in.control_request = 72;
    packet_in.version = 139;
    packet_in.passkey = to_char_array("DEFGHIJKLMNOPQRSTUVWXYZA");

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL packet1{};
    mavlink::common::msg::CHANGE_OPERATOR_CONTROL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.control_request, packet2.control_request);
    EXPECT_EQ(packet1.version, packet2.version);
    EXPECT_EQ(packet1.passkey, packet2.passkey);
}

#ifdef TEST_INTEROP
TEST(common_interop, CHANGE_OPERATOR_CONTROL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_change_operator_control_t packet_c {
         5, 72, 139, "DEFGHIJKLMNOPQRSTUVWXYZA"
    };

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL packet_in{};
    packet_in.target_system = 5;
    packet_in.control_request = 72;
    packet_in.version = 139;
    packet_in.passkey = to_char_array("DEFGHIJKLMNOPQRSTUVWXYZA");

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL packet2{};

    mavlink_msg_change_operator_control_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.control_request, packet2.control_request);
    EXPECT_EQ(packet_in.version, packet2.version);
    EXPECT_EQ(packet_in.passkey, packet2.passkey);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CHANGE_OPERATOR_CONTROL_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL_ACK packet_in{};
    packet_in.gcs_system_id = 5;
    packet_in.control_request = 72;
    packet_in.ack = 139;

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL_ACK packet1{};
    mavlink::common::msg::CHANGE_OPERATOR_CONTROL_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.gcs_system_id, packet2.gcs_system_id);
    EXPECT_EQ(packet1.control_request, packet2.control_request);
    EXPECT_EQ(packet1.ack, packet2.ack);
}

#ifdef TEST_INTEROP
TEST(common_interop, CHANGE_OPERATOR_CONTROL_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_change_operator_control_ack_t packet_c {
         5, 72, 139
    };

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL_ACK packet_in{};
    packet_in.gcs_system_id = 5;
    packet_in.control_request = 72;
    packet_in.ack = 139;

    mavlink::common::msg::CHANGE_OPERATOR_CONTROL_ACK packet2{};

    mavlink_msg_change_operator_control_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.gcs_system_id, packet2.gcs_system_id);
    EXPECT_EQ(packet_in.control_request, packet2.control_request);
    EXPECT_EQ(packet_in.ack, packet2.ack);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, AUTH_KEY)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::AUTH_KEY packet_in{};
    packet_in.key = to_char_array("ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE");

    mavlink::common::msg::AUTH_KEY packet1{};
    mavlink::common::msg::AUTH_KEY packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.key, packet2.key);
}

#ifdef TEST_INTEROP
TEST(common_interop, AUTH_KEY)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_auth_key_t packet_c {
         "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE"
    };

    mavlink::common::msg::AUTH_KEY packet_in{};
    packet_in.key = to_char_array("ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE");

    mavlink::common::msg::AUTH_KEY packet2{};

    mavlink_msg_auth_key_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.key, packet2.key);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_REQUEST_READ)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_REQUEST_READ packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.param_id = to_char_array("EFGHIJKLMNOPQRS");
    packet_in.param_index = 17235;

    mavlink::common::msg::PARAM_REQUEST_READ packet1{};
    mavlink::common::msg::PARAM_REQUEST_READ packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_index, packet2.param_index);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_REQUEST_READ)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_request_read_t packet_c {
         17235, 139, 206, "EFGHIJKLMNOPQRS"
    };

    mavlink::common::msg::PARAM_REQUEST_READ packet_in{};
    packet_in.target_system = 139;
    packet_in.target_component = 206;
    packet_in.param_id = to_char_array("EFGHIJKLMNOPQRS");
    packet_in.param_index = 17235;

    mavlink::common::msg::PARAM_REQUEST_READ packet2{};

    mavlink_msg_param_request_read_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_index, packet2.param_index);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_REQUEST_LIST)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_REQUEST_LIST packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;

    mavlink::common::msg::PARAM_REQUEST_LIST packet1{};
    mavlink::common::msg::PARAM_REQUEST_LIST packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_REQUEST_LIST)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_request_list_t packet_c {
         5, 72
    };

    mavlink::common::msg::PARAM_REQUEST_LIST packet_in{};
    packet_in.target_system = 5;
    packet_in.target_component = 72;

    mavlink::common::msg::PARAM_REQUEST_LIST packet2{};

    mavlink_msg_param_request_list_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_VALUE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_VALUE packet_in{};
    packet_in.param_id = to_char_array("IJKLMNOPQRSTUVW");
    packet_in.param_value = 17.0;
    packet_in.param_type = 77;
    packet_in.param_count = 17443;
    packet_in.param_index = 17547;

    mavlink::common::msg::PARAM_VALUE packet1{};
    mavlink::common::msg::PARAM_VALUE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_value, packet2.param_value);
    EXPECT_EQ(packet1.param_type, packet2.param_type);
    EXPECT_EQ(packet1.param_count, packet2.param_count);
    EXPECT_EQ(packet1.param_index, packet2.param_index);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_VALUE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_value_t packet_c {
         17.0, 17443, 17547, "IJKLMNOPQRSTUVW", 77
    };

    mavlink::common::msg::PARAM_VALUE packet_in{};
    packet_in.param_id = to_char_array("IJKLMNOPQRSTUVW");
    packet_in.param_value = 17.0;
    packet_in.param_type = 77;
    packet_in.param_count = 17443;
    packet_in.param_index = 17547;

    mavlink::common::msg::PARAM_VALUE packet2{};

    mavlink_msg_param_value_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_value, packet2.param_value);
    EXPECT_EQ(packet_in.param_type, packet2.param_type);
    EXPECT_EQ(packet_in.param_count, packet2.param_count);
    EXPECT_EQ(packet_in.param_index, packet2.param_index);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, PARAM_SET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::PARAM_SET packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.param_id = to_char_array("GHIJKLMNOPQRSTU");
    packet_in.param_value = 17.0;
    packet_in.param_type = 199;

    mavlink::common::msg::PARAM_SET packet1{};
    mavlink::common::msg::PARAM_SET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.param_id, packet2.param_id);
    EXPECT_EQ(packet1.param_value, packet2.param_value);
    EXPECT_EQ(packet1.param_type, packet2.param_type);
}

#ifdef TEST_INTEROP
TEST(common_interop, PARAM_SET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_param_set_t packet_c {
         17.0, 17, 84, "GHIJKLMNOPQRSTU", 199
    };

    mavlink::common::msg::PARAM_SET packet_in{};
    packet_in.target_system = 17;
    packet_in.target_component = 84;
    packet_in.param_id = to_char_array("GHIJKLMNOPQRSTU");
    packet_in.param_value = 17.0;
    packet_in.param_type = 199;

    mavlink::common::msg::PARAM_SET packet2{};

    mavlink_msg_param_set_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.param_id, packet2.param_id);
    EXPECT_EQ(packet_in.param_value, packet2.param_value);
    EXPECT_EQ(packet_in.param_type, packet2.param_type);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GPS_RAW_INT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GPS_RAW_INT packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.fix_type = 89;
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 963498296;
    packet_in.eph = 18275;
    packet_in.epv = 18379;
    packet_in.vel = 18483;
    packet_in.cog = 18587;
    packet_in.satellites_visible = 156;

    mavlink::common::msg::GPS_RAW_INT packet1{};
    mavlink::common::msg::GPS_RAW_INT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.fix_type, packet2.fix_type);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.eph, packet2.eph);
    EXPECT_EQ(packet1.epv, packet2.epv);
    EXPECT_EQ(packet1.vel, packet2.vel);
    EXPECT_EQ(packet1.cog, packet2.cog);
    EXPECT_EQ(packet1.satellites_visible, packet2.satellites_visible);
}

#ifdef TEST_INTEROP
TEST(common_interop, GPS_RAW_INT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps_raw_int_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, 18275, 18379, 18483, 18587, 89, 156
    };

    mavlink::common::msg::GPS_RAW_INT packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.fix_type = 89;
    packet_in.lat = 963497880;
    packet_in.lon = 963498088;
    packet_in.alt = 963498296;
    packet_in.eph = 18275;
    packet_in.epv = 18379;
    packet_in.vel = 18483;
    packet_in.cog = 18587;
    packet_in.satellites_visible = 156;

    mavlink::common::msg::GPS_RAW_INT packet2{};

    mavlink_msg_gps_raw_int_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.fix_type, packet2.fix_type);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.eph, packet2.eph);
    EXPECT_EQ(packet_in.epv, packet2.epv);
    EXPECT_EQ(packet_in.vel, packet2.vel);
    EXPECT_EQ(packet_in.cog, packet2.cog);
    EXPECT_EQ(packet_in.satellites_visible, packet2.satellites_visible);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GPS_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GPS_STATUS packet_in{};
    packet_in.satellites_visible = 5;
    packet_in.satellite_prn = {{ 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91 }};
    packet_in.satellite_used = {{ 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151 }};
    packet_in.satellite_elevation = {{ 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 }};
    packet_in.satellite_azimuth = {{ 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }};
    packet_in.satellite_snr = {{ 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75 }};

    mavlink::common::msg::GPS_STATUS packet1{};
    mavlink::common::msg::GPS_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet1.satellite_prn, packet2.satellite_prn);
    EXPECT_EQ(packet1.satellite_used, packet2.satellite_used);
    EXPECT_EQ(packet1.satellite_elevation, packet2.satellite_elevation);
    EXPECT_EQ(packet1.satellite_azimuth, packet2.satellite_azimuth);
    EXPECT_EQ(packet1.satellite_snr, packet2.satellite_snr);
}

#ifdef TEST_INTEROP
TEST(common_interop, GPS_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps_status_t packet_c {
         5, { 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91 }, { 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151 }, { 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 }, { 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }, { 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75 }
    };

    mavlink::common::msg::GPS_STATUS packet_in{};
    packet_in.satellites_visible = 5;
    packet_in.satellite_prn = {{ 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91 }};
    packet_in.satellite_used = {{ 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151 }};
    packet_in.satellite_elevation = {{ 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 }};
    packet_in.satellite_azimuth = {{ 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }};
    packet_in.satellite_snr = {{ 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75 }};

    mavlink::common::msg::GPS_STATUS packet2{};

    mavlink_msg_gps_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet_in.satellite_prn, packet2.satellite_prn);
    EXPECT_EQ(packet_in.satellite_used, packet2.satellite_used);
    EXPECT_EQ(packet_in.satellite_elevation, packet2.satellite_elevation);
    EXPECT_EQ(packet_in.satellite_azimuth, packet2.satellite_azimuth);
    EXPECT_EQ(packet_in.satellite_snr, packet2.satellite_snr);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SCALED_PRESSURE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SCALED_PRESSURE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.press_abs = 45.0;
    packet_in.press_diff = 73.0;
    packet_in.temperature = 17859;

    mavlink::common::msg::SCALED_PRESSURE packet1{};
    mavlink::common::msg::SCALED_PRESSURE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.press_abs, packet2.press_abs);
    EXPECT_EQ(packet1.press_diff, packet2.press_diff);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
}

#ifdef TEST_INTEROP
TEST(common_interop, SCALED_PRESSURE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_scaled_pressure_t packet_c {
         963497464, 45.0, 73.0, 17859
    };

    mavlink::common::msg::SCALED_PRESSURE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.press_abs = 45.0;
    packet_in.press_diff = 73.0;
    packet_in.temperature = 17859;

    mavlink::common::msg::SCALED_PRESSURE packet2{};

    mavlink_msg_scaled_pressure_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.press_abs, packet2.press_abs);
    EXPECT_EQ(packet_in.press_diff, packet2.press_diff);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ATTITUDE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ATTITUDE packet_in{};
    packet_in.roll = 17.0;
    packet_in.pitch = 45.0;
    packet_in.yaw = 73.0;

    mavlink::common::msg::ATTITUDE packet1{};
    mavlink::common::msg::ATTITUDE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
}

#ifdef TEST_INTEROP
TEST(common_interop, ATTITUDE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_t packet_c {
         17.0, 45.0, 73.0
    };

    mavlink::common::msg::ATTITUDE packet_in{};
    packet_in.roll = 17.0;
    packet_in.pitch = 45.0;
    packet_in.yaw = 73.0;

    mavlink::common::msg::ATTITUDE packet2{};

    mavlink_msg_attitude_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ATTITUDE_RATES)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ATTITUDE_RATES packet_in{};
    packet_in.rollspeed = 17.0;
    packet_in.pitchspeed = 45.0;
    packet_in.yawspeed = 73.0;

    mavlink::common::msg::ATTITUDE_RATES packet1{};
    mavlink::common::msg::ATTITUDE_RATES packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
}

#ifdef TEST_INTEROP
TEST(common_interop, ATTITUDE_RATES)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_rates_t packet_c {
         17.0, 45.0, 73.0
    };

    mavlink::common::msg::ATTITUDE_RATES packet_in{};
    packet_in.rollspeed = 17.0;
    packet_in.pitchspeed = 45.0;
    packet_in.yawspeed = 73.0;

    mavlink::common::msg::ATTITUDE_RATES packet2{};

    mavlink_msg_attitude_rates_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ATTITUDE_STATE_FULL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ATTITUDE_STATE_FULL packet_in{};
    packet_in.roll = 17.0;
    packet_in.pitch = 45.0;
    packet_in.yaw = 73.0;
    packet_in.rollspeed = 101.0;
    packet_in.pitchspeed = 129.0;
    packet_in.yawspeed = 157.0;

    mavlink::common::msg::ATTITUDE_STATE_FULL packet1{};
    mavlink::common::msg::ATTITUDE_STATE_FULL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
}

#ifdef TEST_INTEROP
TEST(common_interop, ATTITUDE_STATE_FULL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_state_full_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0
    };

    mavlink::common::msg::ATTITUDE_STATE_FULL packet_in{};
    packet_in.roll = 17.0;
    packet_in.pitch = 45.0;
    packet_in.yaw = 73.0;
    packet_in.rollspeed = 101.0;
    packet_in.pitchspeed = 129.0;
    packet_in.yawspeed = 157.0;

    mavlink::common::msg::ATTITUDE_STATE_FULL packet2{};

    mavlink_msg_attitude_state_full_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ATTITUDE_QUATERNION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ATTITUDE_QUATERNION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.q1 = 45.0;
    packet_in.q2 = 73.0;
    packet_in.q3 = 101.0;
    packet_in.q4 = 129.0;
    packet_in.rollspeed = 157.0;
    packet_in.pitchspeed = 185.0;
    packet_in.yawspeed = 213.0;

    mavlink::common::msg::ATTITUDE_QUATERNION packet1{};
    mavlink::common::msg::ATTITUDE_QUATERNION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.q1, packet2.q1);
    EXPECT_EQ(packet1.q2, packet2.q2);
    EXPECT_EQ(packet1.q3, packet2.q3);
    EXPECT_EQ(packet1.q4, packet2.q4);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
}

#ifdef TEST_INTEROP
TEST(common_interop, ATTITUDE_QUATERNION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_quaternion_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0
    };

    mavlink::common::msg::ATTITUDE_QUATERNION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.q1 = 45.0;
    packet_in.q2 = 73.0;
    packet_in.q3 = 101.0;
    packet_in.q4 = 129.0;
    packet_in.rollspeed = 157.0;
    packet_in.pitchspeed = 185.0;
    packet_in.yawspeed = 213.0;

    mavlink::common::msg::ATTITUDE_QUATERNION packet2{};

    mavlink_msg_attitude_quaternion_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.q1, packet2.q1);
    EXPECT_EQ(packet_in.q2, packet2.q2);
    EXPECT_EQ(packet_in.q3, packet2.q3);
    EXPECT_EQ(packet_in.q4, packet2.q4);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOCAL_POSITION_NED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOCAL_POSITION_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;

    mavlink::common::msg::LOCAL_POSITION_NED packet1{};
    mavlink::common::msg::LOCAL_POSITION_NED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOCAL_POSITION_NED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_local_position_ned_t packet_c {
         963497464, 45.0, 73.0, 101.0
    };

    mavlink::common::msg::LOCAL_POSITION_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;

    mavlink::common::msg::LOCAL_POSITION_NED packet2{};

    mavlink_msg_local_position_ned_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOCAL_VELOCITY_NED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOCAL_VELOCITY_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.vx = 45.0;
    packet_in.vy = 73.0;
    packet_in.vz = 101.0;

    mavlink::common::msg::LOCAL_VELOCITY_NED packet1{};
    mavlink::common::msg::LOCAL_VELOCITY_NED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOCAL_VELOCITY_NED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_local_velocity_ned_t packet_c {
         963497464, 45.0, 73.0, 101.0
    };

    mavlink::common::msg::LOCAL_VELOCITY_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.vx = 45.0;
    packet_in.vy = 73.0;
    packet_in.vz = 101.0;

    mavlink::common::msg::LOCAL_VELOCITY_NED packet2{};

    mavlink_msg_local_velocity_ned_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, LOCAL_STATE_FULL_NED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::LOCAL_STATE_FULL_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;

    mavlink::common::msg::LOCAL_STATE_FULL_NED packet1{};
    mavlink::common::msg::LOCAL_STATE_FULL_NED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
}

#ifdef TEST_INTEROP
TEST(common_interop, LOCAL_STATE_FULL_NED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_local_state_full_ned_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0
    };

    mavlink::common::msg::LOCAL_STATE_FULL_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;

    mavlink::common::msg::LOCAL_STATE_FULL_NED packet2{};

    mavlink_msg_local_state_full_ned_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GLOBAL_POSITION_INT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GLOBAL_POSITION_INT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 963498088;
    packet_in.relative_alt = 963498296;
    packet_in.hdg = 18275;

    mavlink::common::msg::GLOBAL_POSITION_INT packet1{};
    mavlink::common::msg::GLOBAL_POSITION_INT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet1.hdg, packet2.hdg);
}

#ifdef TEST_INTEROP
TEST(common_interop, GLOBAL_POSITION_INT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_global_position_int_t packet_c {
         963497464, 963497672, 963497880, 963498088, 963498296, 18275
    };

    mavlink::common::msg::GLOBAL_POSITION_INT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 963498088;
    packet_in.relative_alt = 963498296;
    packet_in.hdg = 18275;

    mavlink::common::msg::GLOBAL_POSITION_INT packet2{};

    mavlink_msg_global_position_int_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet_in.hdg, packet2.hdg);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GLOBAL_VELOCITY_INT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GLOBAL_VELOCITY_INT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.vx = 17443;
    packet_in.vy = 17547;
    packet_in.vz = 17651;

    mavlink::common::msg::GLOBAL_VELOCITY_INT packet1{};
    mavlink::common::msg::GLOBAL_VELOCITY_INT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
}

#ifdef TEST_INTEROP
TEST(common_interop, GLOBAL_VELOCITY_INT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_global_velocity_int_t packet_c {
         963497464, 17443, 17547, 17651
    };

    mavlink::common::msg::GLOBAL_VELOCITY_INT packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.vx = 17443;
    packet_in.vy = 17547;
    packet_in.vz = 17651;

    mavlink::common::msg::GLOBAL_VELOCITY_INT packet2{};

    mavlink_msg_global_velocity_int_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GLOBAL_POSITION_STATE_FULL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GLOBAL_POSITION_STATE_FULL packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 963498088;
    packet_in.vx = 18275;
    packet_in.vy = 18379;
    packet_in.vz = 18483;
    packet_in.relative_alt = 963498296;
    packet_in.hdg = 18587;

    mavlink::common::msg::GLOBAL_POSITION_STATE_FULL packet1{};
    mavlink::common::msg::GLOBAL_POSITION_STATE_FULL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
    EXPECT_EQ(packet1.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet1.hdg, packet2.hdg);
}

#ifdef TEST_INTEROP
TEST(common_interop, GLOBAL_POSITION_STATE_FULL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_global_position_state_full_t packet_c {
         963497464, 963497672, 963497880, 963498088, 963498296, 18275, 18379, 18483, 18587
    };

    mavlink::common::msg::GLOBAL_POSITION_STATE_FULL packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 963498088;
    packet_in.vx = 18275;
    packet_in.vy = 18379;
    packet_in.vz = 18483;
    packet_in.relative_alt = 963498296;
    packet_in.hdg = 18587;

    mavlink::common::msg::GLOBAL_POSITION_STATE_FULL packet2{};

    mavlink_msg_global_position_state_full_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);
    EXPECT_EQ(packet_in.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet_in.hdg, packet2.hdg);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SET_GPS_GLOBAL_ORIGIN)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN packet_in{};
    packet_in.target_system = 41;
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;

    mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN packet1{};
    mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.latitude, packet2.latitude);
    EXPECT_EQ(packet1.longitude, packet2.longitude);
    EXPECT_EQ(packet1.altitude, packet2.altitude);
}

#ifdef TEST_INTEROP
TEST(common_interop, SET_GPS_GLOBAL_ORIGIN)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_set_gps_global_origin_t packet_c {
         963497464, 963497672, 963497880, 41
    };

    mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN packet_in{};
    packet_in.target_system = 41;
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;

    mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN packet2{};

    mavlink_msg_set_gps_global_origin_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.latitude, packet2.latitude);
    EXPECT_EQ(packet_in.longitude, packet2.longitude);
    EXPECT_EQ(packet_in.altitude, packet2.altitude);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, GPS_GLOBAL_ORIGIN)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::GPS_GLOBAL_ORIGIN packet_in{};
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;

    mavlink::common::msg::GPS_GLOBAL_ORIGIN packet1{};
    mavlink::common::msg::GPS_GLOBAL_ORIGIN packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.latitude, packet2.latitude);
    EXPECT_EQ(packet1.longitude, packet2.longitude);
    EXPECT_EQ(packet1.altitude, packet2.altitude);
}

#ifdef TEST_INTEROP
TEST(common_interop, GPS_GLOBAL_ORIGIN)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps_global_origin_t packet_c {
         963497464, 963497672, 963497880
    };

    mavlink::common::msg::GPS_GLOBAL_ORIGIN packet_in{};
    packet_in.latitude = 963497464;
    packet_in.longitude = 963497672;
    packet_in.altitude = 963497880;

    mavlink::common::msg::GPS_GLOBAL_ORIGIN packet2{};

    mavlink_msg_gps_global_origin_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.latitude, packet2.latitude);
    EXPECT_EQ(packet_in.longitude, packet2.longitude);
    EXPECT_EQ(packet_in.altitude, packet2.altitude);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, VFR_HUD)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::VFR_HUD packet_in{};
    packet_in.airspeed = 17.0;
    packet_in.groundspeed = 45.0;
    packet_in.heading = 18067;
    packet_in.throttle = 18171;
    packet_in.alt = 73.0;
    packet_in.climb = 101.0;

    mavlink::common::msg::VFR_HUD packet1{};
    mavlink::common::msg::VFR_HUD packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.airspeed, packet2.airspeed);
    EXPECT_EQ(packet1.groundspeed, packet2.groundspeed);
    EXPECT_EQ(packet1.heading, packet2.heading);
    EXPECT_EQ(packet1.throttle, packet2.throttle);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.climb, packet2.climb);
}

#ifdef TEST_INTEROP
TEST(common_interop, VFR_HUD)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_vfr_hud_t packet_c {
         17.0, 45.0, 73.0, 101.0, 18067, 18171
    };

    mavlink::common::msg::VFR_HUD packet_in{};
    packet_in.airspeed = 17.0;
    packet_in.groundspeed = 45.0;
    packet_in.heading = 18067;
    packet_in.throttle = 18171;
    packet_in.alt = 73.0;
    packet_in.climb = 101.0;

    mavlink::common::msg::VFR_HUD packet2{};

    mavlink_msg_vfr_hud_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.airspeed, packet2.airspeed);
    EXPECT_EQ(packet_in.groundspeed, packet2.groundspeed);
    EXPECT_EQ(packet_in.heading, packet2.heading);
    EXPECT_EQ(packet_in.throttle, packet2.throttle);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.climb, packet2.climb);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COMMAND_INT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COMMAND_INT packet_in{};
    packet_in.target_system = 223;
    packet_in.target_component = 34;
    packet_in.frame = 101;
    packet_in.command = 18691;
    packet_in.current = 168;
    packet_in.autocontinue = 235;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.x = 963498296;
    packet_in.y = 963498504;
    packet_in.z = 185.0;

    mavlink::common::msg::COMMAND_INT packet1{};
    mavlink::common::msg::COMMAND_INT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
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
TEST(common_interop, COMMAND_INT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_command_int_t packet_c {
         17.0, 45.0, 73.0, 101.0, 963498296, 963498504, 185.0, 18691, 223, 34, 101, 168, 235
    };

    mavlink::common::msg::COMMAND_INT packet_in{};
    packet_in.target_system = 223;
    packet_in.target_component = 34;
    packet_in.frame = 101;
    packet_in.command = 18691;
    packet_in.current = 168;
    packet_in.autocontinue = 235;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.x = 963498296;
    packet_in.y = 963498504;
    packet_in.z = 185.0;

    mavlink::common::msg::COMMAND_INT packet2{};

    mavlink_msg_command_int_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
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

TEST(common, COMMAND_LONG)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COMMAND_LONG packet_in{};
    packet_in.target_system = 223;
    packet_in.target_component = 34;
    packet_in.command = 18691;
    packet_in.confirmation = 101;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.param5 = 129.0;
    packet_in.param6 = 157.0;
    packet_in.param7 = 185.0;

    mavlink::common::msg::COMMAND_LONG packet1{};
    mavlink::common::msg::COMMAND_LONG packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.command, packet2.command);
    EXPECT_EQ(packet1.confirmation, packet2.confirmation);
    EXPECT_EQ(packet1.param1, packet2.param1);
    EXPECT_EQ(packet1.param2, packet2.param2);
    EXPECT_EQ(packet1.param3, packet2.param3);
    EXPECT_EQ(packet1.param4, packet2.param4);
    EXPECT_EQ(packet1.param5, packet2.param5);
    EXPECT_EQ(packet1.param6, packet2.param6);
    EXPECT_EQ(packet1.param7, packet2.param7);
}

#ifdef TEST_INTEROP
TEST(common_interop, COMMAND_LONG)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_command_long_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 18691, 223, 34, 101
    };

    mavlink::common::msg::COMMAND_LONG packet_in{};
    packet_in.target_system = 223;
    packet_in.target_component = 34;
    packet_in.command = 18691;
    packet_in.confirmation = 101;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.param5 = 129.0;
    packet_in.param6 = 157.0;
    packet_in.param7 = 185.0;

    mavlink::common::msg::COMMAND_LONG packet2{};

    mavlink_msg_command_long_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.command, packet2.command);
    EXPECT_EQ(packet_in.confirmation, packet2.confirmation);
    EXPECT_EQ(packet_in.param1, packet2.param1);
    EXPECT_EQ(packet_in.param2, packet2.param2);
    EXPECT_EQ(packet_in.param3, packet2.param3);
    EXPECT_EQ(packet_in.param4, packet2.param4);
    EXPECT_EQ(packet_in.param5, packet2.param5);
    EXPECT_EQ(packet_in.param6, packet2.param6);
    EXPECT_EQ(packet_in.param7, packet2.param7);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COMMAND_SHORT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COMMAND_SHORT packet_in{};
    packet_in.command = 17443;
    packet_in.target_system = 151;
    packet_in.target_component = 218;
    packet_in.confirmation = 29;
    packet_in.param = 17.0;

    mavlink::common::msg::COMMAND_SHORT packet1{};
    mavlink::common::msg::COMMAND_SHORT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.command, packet2.command);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.confirmation, packet2.confirmation);
    EXPECT_EQ(packet1.param, packet2.param);
}

#ifdef TEST_INTEROP
TEST(common_interop, COMMAND_SHORT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_command_short_t packet_c {
         17.0, 17443, 151, 218, 29
    };

    mavlink::common::msg::COMMAND_SHORT packet_in{};
    packet_in.command = 17443;
    packet_in.target_system = 151;
    packet_in.target_component = 218;
    packet_in.confirmation = 29;
    packet_in.param = 17.0;

    mavlink::common::msg::COMMAND_SHORT packet2{};

    mavlink_msg_command_short_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.command, packet2.command);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.confirmation, packet2.confirmation);
    EXPECT_EQ(packet_in.param, packet2.param);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COMMAND_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COMMAND_ACK packet_in{};
    packet_in.command = 17235;
    packet_in.result = 139;

    mavlink::common::msg::COMMAND_ACK packet1{};
    mavlink::common::msg::COMMAND_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.command, packet2.command);
    EXPECT_EQ(packet1.result, packet2.result);
}

#ifdef TEST_INTEROP
TEST(common_interop, COMMAND_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_command_ack_t packet_c {
         17235, 139
    };

    mavlink::common::msg::COMMAND_ACK packet_in{};
    packet_in.command = 17235;
    packet_in.result = 139;

    mavlink::common::msg::COMMAND_ACK packet2{};

    mavlink_msg_command_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.command, packet2.command);
    EXPECT_EQ(packet_in.result, packet2.result);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COMMAND_SYSTEM_MODE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COMMAND_SYSTEM_MODE packet_in{};
    packet_in.target_system = 5;
    packet_in.mode = to_char_array("BCDEFGHIJKLMNOPQRST");

    mavlink::common::msg::COMMAND_SYSTEM_MODE packet1{};
    mavlink::common::msg::COMMAND_SYSTEM_MODE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.mode, packet2.mode);
}

#ifdef TEST_INTEROP
TEST(common_interop, COMMAND_SYSTEM_MODE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_command_system_mode_t packet_c {
         5, "BCDEFGHIJKLMNOPQRST"
    };

    mavlink::common::msg::COMMAND_SYSTEM_MODE packet_in{};
    packet_in.target_system = 5;
    packet_in.mode = to_char_array("BCDEFGHIJKLMNOPQRST");

    mavlink::common::msg::COMMAND_SYSTEM_MODE packet2{};

    mavlink_msg_command_system_mode_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.mode, packet2.mode);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, SYSTEM_MODE_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::SYSTEM_MODE_ACK packet_in{};
    packet_in.result = 5;

    mavlink::common::msg::SYSTEM_MODE_ACK packet1{};
    mavlink::common::msg::SYSTEM_MODE_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.result, packet2.result);
}

#ifdef TEST_INTEROP
TEST(common_interop, SYSTEM_MODE_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_system_mode_ack_t packet_c {
         5
    };

    mavlink::common::msg::SYSTEM_MODE_ACK packet_in{};
    packet_in.result = 5;

    mavlink::common::msg::SYSTEM_MODE_ACK packet2{};

    mavlink_msg_system_mode_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.result, packet2.result);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, EXECUTE_SPATIAL_ACTION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::EXECUTE_SPATIAL_ACTION packet_in{};
    packet_in.target_system = 101;
    packet_in.target_component = 168;
    packet_in.action = 18691;
    packet_in.frame = 235;
    packet_in.dimension = 46;
    packet_in.mask = 18795;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.param5 = 129.0;
    packet_in.param6 = 157.0;
    packet_in.param7 = 185.0;

    mavlink::common::msg::EXECUTE_SPATIAL_ACTION packet1{};
    mavlink::common::msg::EXECUTE_SPATIAL_ACTION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.action, packet2.action);
    EXPECT_EQ(packet1.frame, packet2.frame);
    EXPECT_EQ(packet1.dimension, packet2.dimension);
    EXPECT_EQ(packet1.mask, packet2.mask);
    EXPECT_EQ(packet1.param1, packet2.param1);
    EXPECT_EQ(packet1.param2, packet2.param2);
    EXPECT_EQ(packet1.param3, packet2.param3);
    EXPECT_EQ(packet1.param4, packet2.param4);
    EXPECT_EQ(packet1.param5, packet2.param5);
    EXPECT_EQ(packet1.param6, packet2.param6);
    EXPECT_EQ(packet1.param7, packet2.param7);
}

#ifdef TEST_INTEROP
TEST(common_interop, EXECUTE_SPATIAL_ACTION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_execute_spatial_action_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 18691, 18795, 101, 168, 235, 46
    };

    mavlink::common::msg::EXECUTE_SPATIAL_ACTION packet_in{};
    packet_in.target_system = 101;
    packet_in.target_component = 168;
    packet_in.action = 18691;
    packet_in.frame = 235;
    packet_in.dimension = 46;
    packet_in.mask = 18795;
    packet_in.param1 = 17.0;
    packet_in.param2 = 45.0;
    packet_in.param3 = 73.0;
    packet_in.param4 = 101.0;
    packet_in.param5 = 129.0;
    packet_in.param6 = 157.0;
    packet_in.param7 = 185.0;

    mavlink::common::msg::EXECUTE_SPATIAL_ACTION packet2{};

    mavlink_msg_execute_spatial_action_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.action, packet2.action);
    EXPECT_EQ(packet_in.frame, packet2.frame);
    EXPECT_EQ(packet_in.dimension, packet2.dimension);
    EXPECT_EQ(packet_in.mask, packet2.mask);
    EXPECT_EQ(packet_in.param1, packet2.param1);
    EXPECT_EQ(packet_in.param2, packet2.param2);
    EXPECT_EQ(packet_in.param3, packet2.param3);
    EXPECT_EQ(packet_in.param4, packet2.param4);
    EXPECT_EQ(packet_in.param5, packet2.param5);
    EXPECT_EQ(packet_in.param6, packet2.param6);
    EXPECT_EQ(packet_in.param7, packet2.param7);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, EXECUTE_SPATIAL_ACTION_ACK)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::EXECUTE_SPATIAL_ACTION_ACK packet_in{};
    packet_in.result = 5;

    mavlink::common::msg::EXECUTE_SPATIAL_ACTION_ACK packet1{};
    mavlink::common::msg::EXECUTE_SPATIAL_ACTION_ACK packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.result, packet2.result);
}

#ifdef TEST_INTEROP
TEST(common_interop, EXECUTE_SPATIAL_ACTION_ACK)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_execute_spatial_action_ack_t packet_c {
         5
    };

    mavlink::common::msg::EXECUTE_SPATIAL_ACTION_ACK packet_in{};
    packet_in.result = 5;

    mavlink::common::msg::EXECUTE_SPATIAL_ACTION_ACK packet2{};

    mavlink_msg_execute_spatial_action_ack_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.result, packet2.result);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, RADIO_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::RADIO_STATUS packet_in{};
    packet_in.rssi = 17;
    packet_in.remrssi = 84;
    packet_in.txbuf = 151;
    packet_in.noise = 218;
    packet_in.remnoise = 29;
    packet_in.rxerrors = 17235;
    packet_in.fixed = 17339;

    mavlink::common::msg::RADIO_STATUS packet1{};
    mavlink::common::msg::RADIO_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.rssi, packet2.rssi);
    EXPECT_EQ(packet1.remrssi, packet2.remrssi);
    EXPECT_EQ(packet1.txbuf, packet2.txbuf);
    EXPECT_EQ(packet1.noise, packet2.noise);
    EXPECT_EQ(packet1.remnoise, packet2.remnoise);
    EXPECT_EQ(packet1.rxerrors, packet2.rxerrors);
    EXPECT_EQ(packet1.fixed, packet2.fixed);
}

#ifdef TEST_INTEROP
TEST(common_interop, RADIO_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_radio_status_t packet_c {
         17235, 17339, 17, 84, 151, 218, 29
    };

    mavlink::common::msg::RADIO_STATUS packet_in{};
    packet_in.rssi = 17;
    packet_in.remrssi = 84;
    packet_in.txbuf = 151;
    packet_in.noise = 218;
    packet_in.remnoise = 29;
    packet_in.rxerrors = 17235;
    packet_in.fixed = 17339;

    mavlink::common::msg::RADIO_STATUS packet2{};

    mavlink_msg_radio_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.rssi, packet2.rssi);
    EXPECT_EQ(packet_in.remrssi, packet2.remrssi);
    EXPECT_EQ(packet_in.txbuf, packet2.txbuf);
    EXPECT_EQ(packet_in.noise, packet2.noise);
    EXPECT_EQ(packet_in.remnoise, packet2.remnoise);
    EXPECT_EQ(packet_in.rxerrors, packet2.rxerrors);
    EXPECT_EQ(packet_in.fixed, packet2.fixed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, TIMESYNC)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::TIMESYNC packet_in{};
    packet_in.tc1 = 93372036854775807LL;
    packet_in.ts1 = 170LL;

    mavlink::common::msg::TIMESYNC packet1{};
    mavlink::common::msg::TIMESYNC packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.tc1, packet2.tc1);
    EXPECT_EQ(packet1.ts1, packet2.ts1);
}

#ifdef TEST_INTEROP
TEST(common_interop, TIMESYNC)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_timesync_t packet_c {
         93372036854775807LL, 170LL
    };

    mavlink::common::msg::TIMESYNC packet_in{};
    packet_in.tc1 = 93372036854775807LL;
    packet_in.ts1 = 170LL;

    mavlink::common::msg::TIMESYNC packet2{};

    mavlink_msg_timesync_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.tc1, packet2.tc1);
    EXPECT_EQ(packet_in.ts1, packet2.ts1);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, POWER_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::POWER_STATUS packet_in{};
    packet_in.Vcc = 17235;
    packet_in.Vservo = 17339;
    packet_in.flags = 17443;

    mavlink::common::msg::POWER_STATUS packet1{};
    mavlink::common::msg::POWER_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.Vcc, packet2.Vcc);
    EXPECT_EQ(packet1.Vservo, packet2.Vservo);
    EXPECT_EQ(packet1.flags, packet2.flags);
}

#ifdef TEST_INTEROP
TEST(common_interop, POWER_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_power_status_t packet_c {
         17235, 17339, 17443
    };

    mavlink::common::msg::POWER_STATUS packet_in{};
    packet_in.Vcc = 17235;
    packet_in.Vservo = 17339;
    packet_in.flags = 17443;

    mavlink::common::msg::POWER_STATUS packet2{};

    mavlink_msg_power_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.Vcc, packet2.Vcc);
    EXPECT_EQ(packet_in.Vservo, packet2.Vservo);
    EXPECT_EQ(packet_in.flags, packet2.flags);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, DISTANCE_SENSOR)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::DISTANCE_SENSOR packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.min_distance = 17443;
    packet_in.max_distance = 17547;
    packet_in.current_distance = 17651;
    packet_in.type = 163;
    packet_in.id = 230;
    packet_in.orientation = 41;
    packet_in.covariance = 108;

    mavlink::common::msg::DISTANCE_SENSOR packet1{};
    mavlink::common::msg::DISTANCE_SENSOR packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.min_distance, packet2.min_distance);
    EXPECT_EQ(packet1.max_distance, packet2.max_distance);
    EXPECT_EQ(packet1.current_distance, packet2.current_distance);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.orientation, packet2.orientation);
    EXPECT_EQ(packet1.covariance, packet2.covariance);
}

#ifdef TEST_INTEROP
TEST(common_interop, DISTANCE_SENSOR)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_distance_sensor_t packet_c {
         963497464, 17443, 17547, 17651, 163, 230, 41, 108
    };

    mavlink::common::msg::DISTANCE_SENSOR packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.min_distance = 17443;
    packet_in.max_distance = 17547;
    packet_in.current_distance = 17651;
    packet_in.type = 163;
    packet_in.id = 230;
    packet_in.orientation = 41;
    packet_in.covariance = 108;

    mavlink::common::msg::DISTANCE_SENSOR packet2{};

    mavlink_msg_distance_sensor_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.min_distance, packet2.min_distance);
    EXPECT_EQ(packet_in.max_distance, packet2.max_distance);
    EXPECT_EQ(packet_in.current_distance, packet2.current_distance);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.orientation, packet2.orientation);
    EXPECT_EQ(packet_in.covariance, packet2.covariance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ALTITUDE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ALTITUDE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.altitude_monotonic = 73.0;
    packet_in.altitude_amsl = 101.0;
    packet_in.altitude_local = 129.0;
    packet_in.altitude_relative = 157.0;
    packet_in.altitude_terrain = 185.0;
    packet_in.bottom_clearance = 213.0;

    mavlink::common::msg::ALTITUDE packet1{};
    mavlink::common::msg::ALTITUDE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.altitude_monotonic, packet2.altitude_monotonic);
    EXPECT_EQ(packet1.altitude_amsl, packet2.altitude_amsl);
    EXPECT_EQ(packet1.altitude_local, packet2.altitude_local);
    EXPECT_EQ(packet1.altitude_relative, packet2.altitude_relative);
    EXPECT_EQ(packet1.altitude_terrain, packet2.altitude_terrain);
    EXPECT_EQ(packet1.bottom_clearance, packet2.bottom_clearance);
}

#ifdef TEST_INTEROP
TEST(common_interop, ALTITUDE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_altitude_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0
    };

    mavlink::common::msg::ALTITUDE packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.altitude_monotonic = 73.0;
    packet_in.altitude_amsl = 101.0;
    packet_in.altitude_local = 129.0;
    packet_in.altitude_relative = 157.0;
    packet_in.altitude_terrain = 185.0;
    packet_in.bottom_clearance = 213.0;

    mavlink::common::msg::ALTITUDE packet2{};

    mavlink_msg_altitude_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.altitude_monotonic, packet2.altitude_monotonic);
    EXPECT_EQ(packet_in.altitude_amsl, packet2.altitude_amsl);
    EXPECT_EQ(packet_in.altitude_local, packet2.altitude_local);
    EXPECT_EQ(packet_in.altitude_relative, packet2.altitude_relative);
    EXPECT_EQ(packet_in.altitude_terrain, packet2.altitude_terrain);
    EXPECT_EQ(packet_in.bottom_clearance, packet2.bottom_clearance);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, MESSAGE_INTERVAL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::MESSAGE_INTERVAL packet_in{};
    packet_in.message_id = 17443;
    packet_in.interval_us = 963497464;

    mavlink::common::msg::MESSAGE_INTERVAL packet1{};
    mavlink::common::msg::MESSAGE_INTERVAL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.message_id, packet2.message_id);
    EXPECT_EQ(packet1.interval_us, packet2.interval_us);
}

#ifdef TEST_INTEROP
TEST(common_interop, MESSAGE_INTERVAL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_message_interval_t packet_c {
         963497464, 17443
    };

    mavlink::common::msg::MESSAGE_INTERVAL packet_in{};
    packet_in.message_id = 17443;
    packet_in.interval_us = 963497464;

    mavlink::common::msg::MESSAGE_INTERVAL packet2{};

    mavlink_msg_message_interval_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.message_id, packet2.message_id);
    EXPECT_EQ(packet_in.interval_us, packet2.interval_us);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, EXTENDED_SYS_STATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::EXTENDED_SYS_STATE packet_in{};
    packet_in.vtol_state = 5;
    packet_in.landed_state = 72;

    mavlink::common::msg::EXTENDED_SYS_STATE packet1{};
    mavlink::common::msg::EXTENDED_SYS_STATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.vtol_state, packet2.vtol_state);
    EXPECT_EQ(packet1.landed_state, packet2.landed_state);
}

#ifdef TEST_INTEROP
TEST(common_interop, EXTENDED_SYS_STATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_extended_sys_state_t packet_c {
         5, 72
    };

    mavlink::common::msg::EXTENDED_SYS_STATE packet_in{};
    packet_in.vtol_state = 5;
    packet_in.landed_state = 72;

    mavlink::common::msg::EXTENDED_SYS_STATE packet2{};

    mavlink_msg_extended_sys_state_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.vtol_state, packet2.vtol_state);
    EXPECT_EQ(packet_in.landed_state, packet2.landed_state);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, ADSB_VEHICLE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::ADSB_VEHICLE packet_in{};
    packet_in.ICAO_address = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.altitude_type = 211;
    packet_in.altitude = 963498088;
    packet_in.heading = 18067;
    packet_in.hor_velocity = 18171;
    packet_in.ver_velocity = 18275;
    packet_in.callsign = to_char_array("BCDEFGHI");
    packet_in.emitter_type = 113;
    packet_in.tslc = 180;
    packet_in.flags = 18379;
    packet_in.squawk = 18483;

    mavlink::common::msg::ADSB_VEHICLE packet1{};
    mavlink::common::msg::ADSB_VEHICLE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.ICAO_address, packet2.ICAO_address);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.altitude_type, packet2.altitude_type);
    EXPECT_EQ(packet1.altitude, packet2.altitude);
    EXPECT_EQ(packet1.heading, packet2.heading);
    EXPECT_EQ(packet1.hor_velocity, packet2.hor_velocity);
    EXPECT_EQ(packet1.ver_velocity, packet2.ver_velocity);
    EXPECT_EQ(packet1.callsign, packet2.callsign);
    EXPECT_EQ(packet1.emitter_type, packet2.emitter_type);
    EXPECT_EQ(packet1.tslc, packet2.tslc);
    EXPECT_EQ(packet1.flags, packet2.flags);
    EXPECT_EQ(packet1.squawk, packet2.squawk);
}

#ifdef TEST_INTEROP
TEST(common_interop, ADSB_VEHICLE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_adsb_vehicle_t packet_c {
         963497464, 963497672, 963497880, 963498088, 18067, 18171, 18275, 18379, 18483, 211, "BCDEFGHI", 113, 180
    };

    mavlink::common::msg::ADSB_VEHICLE packet_in{};
    packet_in.ICAO_address = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.altitude_type = 211;
    packet_in.altitude = 963498088;
    packet_in.heading = 18067;
    packet_in.hor_velocity = 18171;
    packet_in.ver_velocity = 18275;
    packet_in.callsign = to_char_array("BCDEFGHI");
    packet_in.emitter_type = 113;
    packet_in.tslc = 180;
    packet_in.flags = 18379;
    packet_in.squawk = 18483;

    mavlink::common::msg::ADSB_VEHICLE packet2{};

    mavlink_msg_adsb_vehicle_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.ICAO_address, packet2.ICAO_address);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.altitude_type, packet2.altitude_type);
    EXPECT_EQ(packet_in.altitude, packet2.altitude);
    EXPECT_EQ(packet_in.heading, packet2.heading);
    EXPECT_EQ(packet_in.hor_velocity, packet2.hor_velocity);
    EXPECT_EQ(packet_in.ver_velocity, packet2.ver_velocity);
    EXPECT_EQ(packet_in.callsign, packet2.callsign);
    EXPECT_EQ(packet_in.emitter_type, packet2.emitter_type);
    EXPECT_EQ(packet_in.tslc, packet2.tslc);
    EXPECT_EQ(packet_in.flags, packet2.flags);
    EXPECT_EQ(packet_in.squawk, packet2.squawk);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, COLLISION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::COLLISION packet_in{};
    packet_in.src = 53;
    packet_in.id = 963497464;
    packet_in.action = 120;
    packet_in.threat_level = 187;
    packet_in.time_to_minimum_delta = 45.0;
    packet_in.altitude_minimum_delta = 73.0;
    packet_in.horizontal_minimum_delta = 101.0;

    mavlink::common::msg::COLLISION packet1{};
    mavlink::common::msg::COLLISION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.src, packet2.src);
    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.action, packet2.action);
    EXPECT_EQ(packet1.threat_level, packet2.threat_level);
    EXPECT_EQ(packet1.time_to_minimum_delta, packet2.time_to_minimum_delta);
    EXPECT_EQ(packet1.altitude_minimum_delta, packet2.altitude_minimum_delta);
    EXPECT_EQ(packet1.horizontal_minimum_delta, packet2.horizontal_minimum_delta);
}

#ifdef TEST_INTEROP
TEST(common_interop, COLLISION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_collision_t packet_c {
         963497464, 45.0, 73.0, 101.0, 53, 120, 187
    };

    mavlink::common::msg::COLLISION packet_in{};
    packet_in.src = 53;
    packet_in.id = 963497464;
    packet_in.action = 120;
    packet_in.threat_level = 187;
    packet_in.time_to_minimum_delta = 45.0;
    packet_in.altitude_minimum_delta = 73.0;
    packet_in.horizontal_minimum_delta = 101.0;

    mavlink::common::msg::COLLISION packet2{};

    mavlink_msg_collision_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.src, packet2.src);
    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.action, packet2.action);
    EXPECT_EQ(packet_in.threat_level, packet2.threat_level);
    EXPECT_EQ(packet_in.time_to_minimum_delta, packet2.time_to_minimum_delta);
    EXPECT_EQ(packet_in.altitude_minimum_delta, packet2.altitude_minimum_delta);
    EXPECT_EQ(packet_in.horizontal_minimum_delta, packet2.horizontal_minimum_delta);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, STATUSTEXT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::STATUSTEXT packet_in{};
    packet_in.severity = 5;
    packet_in.text = to_char_array("BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX");

    mavlink::common::msg::STATUSTEXT packet1{};
    mavlink::common::msg::STATUSTEXT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.severity, packet2.severity);
    EXPECT_EQ(packet1.text, packet2.text);
}

#ifdef TEST_INTEROP
TEST(common_interop, STATUSTEXT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_statustext_t packet_c {
         5, "BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX"
    };

    mavlink::common::msg::STATUSTEXT packet_in{};
    packet_in.severity = 5;
    packet_in.text = to_char_array("BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX");

    mavlink::common::msg::STATUSTEXT packet2{};

    mavlink_msg_statustext_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.severity, packet2.severity);
    EXPECT_EQ(packet_in.text, packet2.text);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_INFORMATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.camera_id = 65;
    packet_in.vendor_name = {{ 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163 }};
    packet_in.model_name = {{ 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3 }};
    packet_in.focal_length = 45.0;
    packet_in.sensor_size_h = 73.0;
    packet_in.sensor_size_v = 101.0;
    packet_in.resolution_h = 18067;
    packet_in.resolution_v = 18171;
    packet_in.lense_id = 68;

    mavlink::common::msg::CAMERA_INFORMATION packet1{};
    mavlink::common::msg::CAMERA_INFORMATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.camera_id, packet2.camera_id);
    EXPECT_EQ(packet1.vendor_name, packet2.vendor_name);
    EXPECT_EQ(packet1.model_name, packet2.model_name);
    EXPECT_EQ(packet1.focal_length, packet2.focal_length);
    EXPECT_EQ(packet1.sensor_size_h, packet2.sensor_size_h);
    EXPECT_EQ(packet1.sensor_size_v, packet2.sensor_size_v);
    EXPECT_EQ(packet1.resolution_h, packet2.resolution_h);
    EXPECT_EQ(packet1.resolution_v, packet2.resolution_v);
    EXPECT_EQ(packet1.lense_id, packet2.lense_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_INFORMATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_information_t packet_c {
         963497464, 45.0, 73.0, 101.0, 18067, 18171, 65, { 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163 }, { 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3 }, 68
    };

    mavlink::common::msg::CAMERA_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.camera_id = 65;
    packet_in.vendor_name = {{ 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163 }};
    packet_in.model_name = {{ 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3 }};
    packet_in.focal_length = 45.0;
    packet_in.sensor_size_h = 73.0;
    packet_in.sensor_size_v = 101.0;
    packet_in.resolution_h = 18067;
    packet_in.resolution_v = 18171;
    packet_in.lense_id = 68;

    mavlink::common::msg::CAMERA_INFORMATION packet2{};

    mavlink_msg_camera_information_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.camera_id, packet2.camera_id);
    EXPECT_EQ(packet_in.vendor_name, packet2.vendor_name);
    EXPECT_EQ(packet_in.model_name, packet2.model_name);
    EXPECT_EQ(packet_in.focal_length, packet2.focal_length);
    EXPECT_EQ(packet_in.sensor_size_h, packet2.sensor_size_h);
    EXPECT_EQ(packet_in.sensor_size_v, packet2.sensor_size_v);
    EXPECT_EQ(packet_in.resolution_h, packet2.resolution_h);
    EXPECT_EQ(packet_in.resolution_v, packet2.resolution_v);
    EXPECT_EQ(packet_in.lense_id, packet2.lense_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_SETTINGS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_SETTINGS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.camera_id = 65;
    packet_in.aperture = 45.0;
    packet_in.aperture_locked = 132;
    packet_in.shutter_speed = 73.0;
    packet_in.shutter_speed_locked = 199;
    packet_in.iso_sensitivity = 101.0;
    packet_in.iso_sensitivity_locked = 10;
    packet_in.white_balance = 129.0;
    packet_in.white_balance_locked = 77;
    packet_in.mode_id = 144;
    packet_in.color_mode_id = 211;
    packet_in.image_format_id = 22;

    mavlink::common::msg::CAMERA_SETTINGS packet1{};
    mavlink::common::msg::CAMERA_SETTINGS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.camera_id, packet2.camera_id);
    EXPECT_EQ(packet1.aperture, packet2.aperture);
    EXPECT_EQ(packet1.aperture_locked, packet2.aperture_locked);
    EXPECT_EQ(packet1.shutter_speed, packet2.shutter_speed);
    EXPECT_EQ(packet1.shutter_speed_locked, packet2.shutter_speed_locked);
    EXPECT_EQ(packet1.iso_sensitivity, packet2.iso_sensitivity);
    EXPECT_EQ(packet1.iso_sensitivity_locked, packet2.iso_sensitivity_locked);
    EXPECT_EQ(packet1.white_balance, packet2.white_balance);
    EXPECT_EQ(packet1.white_balance_locked, packet2.white_balance_locked);
    EXPECT_EQ(packet1.mode_id, packet2.mode_id);
    EXPECT_EQ(packet1.color_mode_id, packet2.color_mode_id);
    EXPECT_EQ(packet1.image_format_id, packet2.image_format_id);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_SETTINGS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_settings_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 65, 132, 199, 10, 77, 144, 211, 22
    };

    mavlink::common::msg::CAMERA_SETTINGS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.camera_id = 65;
    packet_in.aperture = 45.0;
    packet_in.aperture_locked = 132;
    packet_in.shutter_speed = 73.0;
    packet_in.shutter_speed_locked = 199;
    packet_in.iso_sensitivity = 101.0;
    packet_in.iso_sensitivity_locked = 10;
    packet_in.white_balance = 129.0;
    packet_in.white_balance_locked = 77;
    packet_in.mode_id = 144;
    packet_in.color_mode_id = 211;
    packet_in.image_format_id = 22;

    mavlink::common::msg::CAMERA_SETTINGS packet2{};

    mavlink_msg_camera_settings_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.camera_id, packet2.camera_id);
    EXPECT_EQ(packet_in.aperture, packet2.aperture);
    EXPECT_EQ(packet_in.aperture_locked, packet2.aperture_locked);
    EXPECT_EQ(packet_in.shutter_speed, packet2.shutter_speed);
    EXPECT_EQ(packet_in.shutter_speed_locked, packet2.shutter_speed_locked);
    EXPECT_EQ(packet_in.iso_sensitivity, packet2.iso_sensitivity);
    EXPECT_EQ(packet_in.iso_sensitivity_locked, packet2.iso_sensitivity_locked);
    EXPECT_EQ(packet_in.white_balance, packet2.white_balance);
    EXPECT_EQ(packet_in.white_balance_locked, packet2.white_balance_locked);
    EXPECT_EQ(packet_in.mode_id, packet2.mode_id);
    EXPECT_EQ(packet_in.color_mode_id, packet2.color_mode_id);
    EXPECT_EQ(packet_in.image_format_id, packet2.image_format_id);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_CAPTURE_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_CAPTURE_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.camera_id = 89;
    packet_in.image_status = 156;
    packet_in.video_status = 223;
    packet_in.image_interval = 45.0;
    packet_in.video_framerate = 73.0;
    packet_in.image_resolution_h = 18275;
    packet_in.image_resolution_v = 18379;
    packet_in.video_resolution_h = 18483;
    packet_in.video_resolution_v = 18587;
    packet_in.recording_time_ms = 963498088;
    packet_in.available_capacity = 129.0;

    mavlink::common::msg::CAMERA_CAPTURE_STATUS packet1{};
    mavlink::common::msg::CAMERA_CAPTURE_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.camera_id, packet2.camera_id);
    EXPECT_EQ(packet1.image_status, packet2.image_status);
    EXPECT_EQ(packet1.video_status, packet2.video_status);
    EXPECT_EQ(packet1.image_interval, packet2.image_interval);
    EXPECT_EQ(packet1.video_framerate, packet2.video_framerate);
    EXPECT_EQ(packet1.image_resolution_h, packet2.image_resolution_h);
    EXPECT_EQ(packet1.image_resolution_v, packet2.image_resolution_v);
    EXPECT_EQ(packet1.video_resolution_h, packet2.video_resolution_h);
    EXPECT_EQ(packet1.video_resolution_v, packet2.video_resolution_v);
    EXPECT_EQ(packet1.recording_time_ms, packet2.recording_time_ms);
    EXPECT_EQ(packet1.available_capacity, packet2.available_capacity);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_CAPTURE_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_capture_status_t packet_c {
         963497464, 45.0, 73.0, 963498088, 129.0, 18275, 18379, 18483, 18587, 89, 156, 223
    };

    mavlink::common::msg::CAMERA_CAPTURE_STATUS packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.camera_id = 89;
    packet_in.image_status = 156;
    packet_in.video_status = 223;
    packet_in.image_interval = 45.0;
    packet_in.video_framerate = 73.0;
    packet_in.image_resolution_h = 18275;
    packet_in.image_resolution_v = 18379;
    packet_in.video_resolution_h = 18483;
    packet_in.video_resolution_v = 18587;
    packet_in.recording_time_ms = 963498088;
    packet_in.available_capacity = 129.0;

    mavlink::common::msg::CAMERA_CAPTURE_STATUS packet2{};

    mavlink_msg_camera_capture_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.camera_id, packet2.camera_id);
    EXPECT_EQ(packet_in.image_status, packet2.image_status);
    EXPECT_EQ(packet_in.video_status, packet2.video_status);
    EXPECT_EQ(packet_in.image_interval, packet2.image_interval);
    EXPECT_EQ(packet_in.video_framerate, packet2.video_framerate);
    EXPECT_EQ(packet_in.image_resolution_h, packet2.image_resolution_h);
    EXPECT_EQ(packet_in.image_resolution_v, packet2.image_resolution_v);
    EXPECT_EQ(packet_in.video_resolution_h, packet2.video_resolution_h);
    EXPECT_EQ(packet_in.video_resolution_v, packet2.video_resolution_v);
    EXPECT_EQ(packet_in.recording_time_ms, packet2.recording_time_ms);
    EXPECT_EQ(packet_in.available_capacity, packet2.available_capacity);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, CAMERA_IMAGE_CAPTURED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::CAMERA_IMAGE_CAPTURED packet_in{};
    packet_in.time_boot_ms = 963497880;
    packet_in.time_utc = 93372036854775807ULL;
    packet_in.camera_id = 149;
    packet_in.lat = 963498088;
    packet_in.lon = 963498296;
    packet_in.alt = 963498504;
    packet_in.relative_alt = 963498712;
    packet_in.q = {{ 213.0, 214.0, 215.0, 216.0 }};
    packet_in.image_index = 963499752;
    packet_in.capture_result = -40;
    packet_in.file_url = to_char_array("YZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRST");

    mavlink::common::msg::CAMERA_IMAGE_CAPTURED packet1{};
    mavlink::common::msg::CAMERA_IMAGE_CAPTURED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.time_utc, packet2.time_utc);
    EXPECT_EQ(packet1.camera_id, packet2.camera_id);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.image_index, packet2.image_index);
    EXPECT_EQ(packet1.capture_result, packet2.capture_result);
    EXPECT_EQ(packet1.file_url, packet2.file_url);
}

#ifdef TEST_INTEROP
TEST(common_interop, CAMERA_IMAGE_CAPTURED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_camera_image_captured_t packet_c {
         93372036854775807ULL, 963497880, 963498088, 963498296, 963498504, 963498712, { 213.0, 214.0, 215.0, 216.0 }, 963499752, 149, -40, "YZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRST"
    };

    mavlink::common::msg::CAMERA_IMAGE_CAPTURED packet_in{};
    packet_in.time_boot_ms = 963497880;
    packet_in.time_utc = 93372036854775807ULL;
    packet_in.camera_id = 149;
    packet_in.lat = 963498088;
    packet_in.lon = 963498296;
    packet_in.alt = 963498504;
    packet_in.relative_alt = 963498712;
    packet_in.q = {{ 213.0, 214.0, 215.0, 216.0 }};
    packet_in.image_index = 963499752;
    packet_in.capture_result = -40;
    packet_in.file_url = to_char_array("YZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRST");

    mavlink::common::msg::CAMERA_IMAGE_CAPTURED packet2{};

    mavlink_msg_camera_image_captured_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.time_utc, packet2.time_utc);
    EXPECT_EQ(packet_in.camera_id, packet2.camera_id);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.relative_alt, packet2.relative_alt);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.image_index, packet2.image_index);
    EXPECT_EQ(packet_in.capture_result, packet2.capture_result);
    EXPECT_EQ(packet_in.file_url, packet2.file_url);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(common, FLIGHT_INFORMATION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::common::msg::FLIGHT_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963498712;
    packet_in.arming_time_utc = 93372036854775807ULL;
    packet_in.takeoff_time_utc = 93372036854776311ULL;
    packet_in.flight_uuid = 93372036854776815ULL;

    mavlink::common::msg::FLIGHT_INFORMATION packet1{};
    mavlink::common::msg::FLIGHT_INFORMATION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.arming_time_utc, packet2.arming_time_utc);
    EXPECT_EQ(packet1.takeoff_time_utc, packet2.takeoff_time_utc);
    EXPECT_EQ(packet1.flight_uuid, packet2.flight_uuid);
}

#ifdef TEST_INTEROP
TEST(common_interop, FLIGHT_INFORMATION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_flight_information_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 93372036854776815ULL, 963498712
    };

    mavlink::common::msg::FLIGHT_INFORMATION packet_in{};
    packet_in.time_boot_ms = 963498712;
    packet_in.arming_time_utc = 93372036854775807ULL;
    packet_in.takeoff_time_utc = 93372036854776311ULL;
    packet_in.flight_uuid = 93372036854776815ULL;

    mavlink::common::msg::FLIGHT_INFORMATION packet2{};

    mavlink_msg_flight_information_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.arming_time_utc, packet2.arming_time_utc);
    EXPECT_EQ(packet_in.takeoff_time_utc, packet2.takeoff_time_utc);
    EXPECT_EQ(packet_in.flight_uuid, packet2.flight_uuid);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
