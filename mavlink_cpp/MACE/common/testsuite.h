/** @file
 *    @brief MAVLink comm protocol testsuite generated from common.xml
 *    @see http://qgroundcontrol.org/mace/
 */
#pragma once
#ifndef COMMON_TESTSUITE_H
#define COMMON_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MACE_TEST_ALL
#define MACE_TEST_ALL

static void mace_test_common(uint8_t, uint8_t, mace_message_t *last_msg);

static void mace_test_all(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{

    mace_test_common(system_id, component_id, last_msg);
}
#endif




static void mace_test_heartbeat(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_HEARTBEAT >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_heartbeat_t packet_in = {
        5,72,139,206,17,3,151
    };
    mace_heartbeat_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.protocol = packet_in.protocol;
        packet1.type = packet_in.type;
        packet1.autopilot = packet_in.autopilot;
        packet1.mission_state = packet_in.mission_state;
        packet1.mace_companion = packet_in.mace_companion;
        packet1.mavlink_version = packet_in.mavlink_version;
        packet1.mavlinkID = packet_in.mavlinkID;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_HEARTBEAT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_HEARTBEAT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_heartbeat_encode(system_id, component_id, &msg, &packet1);
    mace_msg_heartbeat_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_heartbeat_pack(system_id, component_id, &msg , packet1.protocol , packet1.type , packet1.autopilot , packet1.mission_state , packet1.mace_companion , packet1.mavlinkID );
    mace_msg_heartbeat_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_heartbeat_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.protocol , packet1.type , packet1.autopilot , packet1.mission_state , packet1.mace_companion , packet1.mavlinkID );
    mace_msg_heartbeat_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_heartbeat_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_heartbeat_send(MACE_COMM_1 , packet1.protocol , packet1.type , packet1.autopilot , packet1.mission_state , packet1.mace_companion , packet1.mavlinkID );
    mace_msg_heartbeat_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_vehicle_mode(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_VEHICLE_MODE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_vehicle_mode_t packet_in = {
        "ABCDEFGHI"
    };
    mace_vehicle_mode_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mace_array_memcpy(packet1.vehicle_mode, packet_in.vehicle_mode, sizeof(char)*10);
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_VEHICLE_MODE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_VEHICLE_MODE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vehicle_mode_encode(system_id, component_id, &msg, &packet1);
    mace_msg_vehicle_mode_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vehicle_mode_pack(system_id, component_id, &msg , packet1.vehicle_mode );
    mace_msg_vehicle_mode_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vehicle_mode_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.vehicle_mode );
    mace_msg_vehicle_mode_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_vehicle_mode_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vehicle_mode_send(MACE_COMM_1 , packet1.vehicle_mode );
    mace_msg_vehicle_mode_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_vehicle_armed(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_VEHICLE_ARMED >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_vehicle_armed_t packet_in = {
        5
    };
    mace_vehicle_armed_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vehicle_armed = packet_in.vehicle_armed;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_VEHICLE_ARMED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_VEHICLE_ARMED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vehicle_armed_encode(system_id, component_id, &msg, &packet1);
    mace_msg_vehicle_armed_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vehicle_armed_pack(system_id, component_id, &msg , packet1.vehicle_armed );
    mace_msg_vehicle_armed_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vehicle_armed_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.vehicle_armed );
    mace_msg_vehicle_armed_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_vehicle_armed_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vehicle_armed_send(MACE_COMM_1 , packet1.vehicle_armed );
    mace_msg_vehicle_armed_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_battery_status(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_BATTERY_STATUS >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_battery_status_t packet_in = {
        17235,17339,17443,151,218,29,96
    };
    mace_battery_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.temperature = packet_in.temperature;
        packet1.voltage_battery = packet_in.voltage_battery;
        packet1.current_battery = packet_in.current_battery;
        packet1.id = packet_in.id;
        packet1.battery_function = packet_in.battery_function;
        packet1.type = packet_in.type;
        packet1.battery_remaining = packet_in.battery_remaining;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_BATTERY_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_BATTERY_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_battery_status_encode(system_id, component_id, &msg, &packet1);
    mace_msg_battery_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_battery_status_pack(system_id, component_id, &msg , packet1.id , packet1.battery_function , packet1.type , packet1.temperature , packet1.voltage_battery , packet1.current_battery , packet1.battery_remaining );
    mace_msg_battery_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_battery_status_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.id , packet1.battery_function , packet1.type , packet1.temperature , packet1.voltage_battery , packet1.current_battery , packet1.battery_remaining );
    mace_msg_battery_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_battery_status_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_battery_status_send(MACE_COMM_1 , packet1.id , packet1.battery_function , packet1.type , packet1.temperature , packet1.voltage_battery , packet1.current_battery , packet1.battery_remaining );
    mace_msg_battery_status_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_system_time(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_SYSTEM_TIME >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_system_time_t packet_in = {
        93372036854775807ULL,963497880
    };
    mace_system_time_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_unix_usec = packet_in.time_unix_usec;
        packet1.time_boot_ms = packet_in.time_boot_ms;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_SYSTEM_TIME_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_SYSTEM_TIME_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_system_time_encode(system_id, component_id, &msg, &packet1);
    mace_msg_system_time_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_system_time_pack(system_id, component_id, &msg , packet1.time_unix_usec , packet1.time_boot_ms );
    mace_msg_system_time_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_system_time_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_unix_usec , packet1.time_boot_ms );
    mace_msg_system_time_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_system_time_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_system_time_send(MACE_COMM_1 , packet1.time_unix_usec , packet1.time_boot_ms );
    mace_msg_system_time_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_ping(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_PING >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_ping_t packet_in = {
        93372036854775807ULL,963497880,41,108
    };
    mace_ping_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.seq = packet_in.seq;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_PING_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_PING_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_ping_encode(system_id, component_id, &msg, &packet1);
    mace_msg_ping_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_ping_pack(system_id, component_id, &msg , packet1.time_usec , packet1.seq , packet1.target_system , packet1.target_component );
    mace_msg_ping_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_ping_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_usec , packet1.seq , packet1.target_system , packet1.target_component );
    mace_msg_ping_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_ping_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_ping_send(MACE_COMM_1 , packet1.time_usec , packet1.seq , packet1.target_system , packet1.target_component );
    mace_msg_ping_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_change_operator_control(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_CHANGE_OPERATOR_CONTROL >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_change_operator_control_t packet_in = {
        5,72,139,"DEFGHIJKLMNOPQRSTUVWXYZA"
    };
    mace_change_operator_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.control_request = packet_in.control_request;
        packet1.version = packet_in.version;
        
        mace_array_memcpy(packet1.passkey, packet_in.passkey, sizeof(char)*25);
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_change_operator_control_encode(system_id, component_id, &msg, &packet1);
    mace_msg_change_operator_control_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_change_operator_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.control_request , packet1.version , packet1.passkey );
    mace_msg_change_operator_control_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_change_operator_control_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.control_request , packet1.version , packet1.passkey );
    mace_msg_change_operator_control_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_change_operator_control_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_change_operator_control_send(MACE_COMM_1 , packet1.target_system , packet1.control_request , packet1.version , packet1.passkey );
    mace_msg_change_operator_control_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_change_operator_control_ack(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_change_operator_control_ack_t packet_in = {
        5,72,139
    };
    mace_change_operator_control_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.gcs_system_id = packet_in.gcs_system_id;
        packet1.control_request = packet_in.control_request;
        packet1.ack = packet_in.ack;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_change_operator_control_ack_encode(system_id, component_id, &msg, &packet1);
    mace_msg_change_operator_control_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_change_operator_control_ack_pack(system_id, component_id, &msg , packet1.gcs_system_id , packet1.control_request , packet1.ack );
    mace_msg_change_operator_control_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_change_operator_control_ack_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.gcs_system_id , packet1.control_request , packet1.ack );
    mace_msg_change_operator_control_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_change_operator_control_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_change_operator_control_ack_send(MACE_COMM_1 , packet1.gcs_system_id , packet1.control_request , packet1.ack );
    mace_msg_change_operator_control_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auth_key(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUTH_KEY >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auth_key_t packet_in = {
        "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE"
    };
    mace_auth_key_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mace_array_memcpy(packet1.key, packet_in.key, sizeof(char)*32);
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUTH_KEY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUTH_KEY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auth_key_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auth_key_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auth_key_pack(system_id, component_id, &msg , packet1.key );
    mace_msg_auth_key_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auth_key_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.key );
    mace_msg_auth_key_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auth_key_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auth_key_send(MACE_COMM_1 , packet1.key );
    mace_msg_auth_key_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_param_request_read(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_PARAM_REQUEST_READ >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_param_request_read_t packet_in = {
        17235,139,206,"EFGHIJKLMNOPQRS"
    };
    mace_param_request_read_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_index = packet_in.param_index;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mace_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_PARAM_REQUEST_READ_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_PARAM_REQUEST_READ_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_request_read_encode(system_id, component_id, &msg, &packet1);
    mace_msg_param_request_read_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_request_read_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_index );
    mace_msg_param_request_read_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_request_read_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_index );
    mace_msg_param_request_read_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_param_request_read_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_request_read_send(MACE_COMM_1 , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_index );
    mace_msg_param_request_read_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_param_request_list(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_PARAM_REQUEST_LIST >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_param_request_list_t packet_in = {
        5,72
    };
    mace_param_request_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_PARAM_REQUEST_LIST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_PARAM_REQUEST_LIST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_request_list_encode(system_id, component_id, &msg, &packet1);
    mace_msg_param_request_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_request_list_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component );
    mace_msg_param_request_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_request_list_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.target_component );
    mace_msg_param_request_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_param_request_list_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_request_list_send(MACE_COMM_1 , packet1.target_system , packet1.target_component );
    mace_msg_param_request_list_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_param_value(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_PARAM_VALUE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_param_value_t packet_in = {
        17.0,17443,17547,"IJKLMNOPQRSTUVW",77
    };
    mace_param_value_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_value = packet_in.param_value;
        packet1.param_count = packet_in.param_count;
        packet1.param_index = packet_in.param_index;
        packet1.param_type = packet_in.param_type;
        
        mace_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_PARAM_VALUE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_PARAM_VALUE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_value_encode(system_id, component_id, &msg, &packet1);
    mace_msg_param_value_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_value_pack(system_id, component_id, &msg , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_count , packet1.param_index );
    mace_msg_param_value_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_value_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_count , packet1.param_index );
    mace_msg_param_value_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_param_value_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_value_send(MACE_COMM_1 , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_count , packet1.param_index );
    mace_msg_param_value_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_param_set(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_PARAM_SET >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_param_set_t packet_in = {
        17.0,17,84,"GHIJKLMNOPQRSTU",199
    };
    mace_param_set_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_value = packet_in.param_value;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.param_type = packet_in.param_type;
        
        mace_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_PARAM_SET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_PARAM_SET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_set_encode(system_id, component_id, &msg, &packet1);
    mace_msg_param_set_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_set_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_value , packet1.param_type );
    mace_msg_param_set_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_set_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_value , packet1.param_type );
    mace_msg_param_set_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_param_set_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_param_set_send(MACE_COMM_1 , packet1.target_system , packet1.target_component , packet1.param_id , packet1.param_value , packet1.param_type );
    mace_msg_param_set_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_gps_raw_int(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_GPS_RAW_INT >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_gps_raw_int_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,18275,18379,18483,18587,89,156
    };
    mace_gps_raw_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.eph = packet_in.eph;
        packet1.epv = packet_in.epv;
        packet1.vel = packet_in.vel;
        packet1.cog = packet_in.cog;
        packet1.fix_type = packet_in.fix_type;
        packet1.satellites_visible = packet_in.satellites_visible;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_GPS_RAW_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_GPS_RAW_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_gps_raw_int_encode(system_id, component_id, &msg, &packet1);
    mace_msg_gps_raw_int_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_gps_raw_int_pack(system_id, component_id, &msg , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible );
    mace_msg_gps_raw_int_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_gps_raw_int_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible );
    mace_msg_gps_raw_int_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_gps_raw_int_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_gps_raw_int_send(MACE_COMM_1 , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible );
    mace_msg_gps_raw_int_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_gps_status(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_GPS_STATUS >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_gps_status_t packet_in = {
        5,{ 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91 },{ 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151 },{ 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 },{ 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },{ 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75 }
    };
    mace_gps_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.satellites_visible = packet_in.satellites_visible;
        
        mace_array_memcpy(packet1.satellite_prn, packet_in.satellite_prn, sizeof(uint8_t)*20);
        mace_array_memcpy(packet1.satellite_used, packet_in.satellite_used, sizeof(uint8_t)*20);
        mace_array_memcpy(packet1.satellite_elevation, packet_in.satellite_elevation, sizeof(uint8_t)*20);
        mace_array_memcpy(packet1.satellite_azimuth, packet_in.satellite_azimuth, sizeof(uint8_t)*20);
        mace_array_memcpy(packet1.satellite_snr, packet_in.satellite_snr, sizeof(uint8_t)*20);
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_GPS_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_GPS_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_gps_status_encode(system_id, component_id, &msg, &packet1);
    mace_msg_gps_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_gps_status_pack(system_id, component_id, &msg , packet1.satellites_visible , packet1.satellite_prn , packet1.satellite_used , packet1.satellite_elevation , packet1.satellite_azimuth , packet1.satellite_snr );
    mace_msg_gps_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_gps_status_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.satellites_visible , packet1.satellite_prn , packet1.satellite_used , packet1.satellite_elevation , packet1.satellite_azimuth , packet1.satellite_snr );
    mace_msg_gps_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_gps_status_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_gps_status_send(MACE_COMM_1 , packet1.satellites_visible , packet1.satellite_prn , packet1.satellite_used , packet1.satellite_elevation , packet1.satellite_azimuth , packet1.satellite_snr );
    mace_msg_gps_status_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_scaled_pressure(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_SCALED_PRESSURE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_scaled_pressure_t packet_in = {
        963497464,45.0,73.0,17859
    };
    mace_scaled_pressure_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.press_abs = packet_in.press_abs;
        packet1.press_diff = packet_in.press_diff;
        packet1.temperature = packet_in.temperature;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_SCALED_PRESSURE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_SCALED_PRESSURE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_scaled_pressure_encode(system_id, component_id, &msg, &packet1);
    mace_msg_scaled_pressure_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_scaled_pressure_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature );
    mace_msg_scaled_pressure_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_scaled_pressure_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature );
    mace_msg_scaled_pressure_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_scaled_pressure_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_scaled_pressure_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.press_abs , packet1.press_diff , packet1.temperature );
    mace_msg_scaled_pressure_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_attitude(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_ATTITUDE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_attitude_t packet_in = {
        17.0,45.0,73.0
    };
    mace_attitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_ATTITUDE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_ATTITUDE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_encode(system_id, component_id, &msg, &packet1);
    mace_msg_attitude_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_pack(system_id, component_id, &msg , packet1.roll , packet1.pitch , packet1.yaw );
    mace_msg_attitude_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.roll , packet1.pitch , packet1.yaw );
    mace_msg_attitude_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_attitude_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_send(MACE_COMM_1 , packet1.roll , packet1.pitch , packet1.yaw );
    mace_msg_attitude_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_attitude_rates(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_ATTITUDE_RATES >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_attitude_rates_t packet_in = {
        17.0,45.0,73.0
    };
    mace_attitude_rates_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.rollspeed = packet_in.rollspeed;
        packet1.pitchspeed = packet_in.pitchspeed;
        packet1.yawspeed = packet_in.yawspeed;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_ATTITUDE_RATES_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_ATTITUDE_RATES_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_rates_encode(system_id, component_id, &msg, &packet1);
    mace_msg_attitude_rates_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_rates_pack(system_id, component_id, &msg , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mace_msg_attitude_rates_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_rates_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mace_msg_attitude_rates_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_attitude_rates_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_rates_send(MACE_COMM_1 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mace_msg_attitude_rates_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_attitude_state_full(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_ATTITUDE_STATE_FULL >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_attitude_state_full_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0
    };
    mace_attitude_state_full_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.rollspeed = packet_in.rollspeed;
        packet1.pitchspeed = packet_in.pitchspeed;
        packet1.yawspeed = packet_in.yawspeed;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_ATTITUDE_STATE_FULL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_ATTITUDE_STATE_FULL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_state_full_encode(system_id, component_id, &msg, &packet1);
    mace_msg_attitude_state_full_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_state_full_pack(system_id, component_id, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mace_msg_attitude_state_full_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_state_full_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mace_msg_attitude_state_full_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_attitude_state_full_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_state_full_send(MACE_COMM_1 , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mace_msg_attitude_state_full_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_attitude_quaternion(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_ATTITUDE_QUATERNION >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_attitude_quaternion_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0
    };
    mace_attitude_quaternion_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.q1 = packet_in.q1;
        packet1.q2 = packet_in.q2;
        packet1.q3 = packet_in.q3;
        packet1.q4 = packet_in.q4;
        packet1.rollspeed = packet_in.rollspeed;
        packet1.pitchspeed = packet_in.pitchspeed;
        packet1.yawspeed = packet_in.yawspeed;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_quaternion_encode(system_id, component_id, &msg, &packet1);
    mace_msg_attitude_quaternion_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_quaternion_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mace_msg_attitude_quaternion_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_quaternion_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mace_msg_attitude_quaternion_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_attitude_quaternion_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_attitude_quaternion_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mace_msg_attitude_quaternion_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_local_position_ned(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_LOCAL_POSITION_NED >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_local_position_ned_t packet_in = {
        963497464,45.0,73.0,101.0
    };
    mace_local_position_ned_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_LOCAL_POSITION_NED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_LOCAL_POSITION_NED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_local_position_ned_encode(system_id, component_id, &msg, &packet1);
    mace_msg_local_position_ned_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_local_position_ned_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z );
    mace_msg_local_position_ned_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_local_position_ned_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z );
    mace_msg_local_position_ned_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_local_position_ned_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_local_position_ned_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z );
    mace_msg_local_position_ned_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_local_velocity_ned(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_LOCAL_VELOCITY_NED >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_local_velocity_ned_t packet_in = {
        963497464,45.0,73.0,101.0
    };
    mace_local_velocity_ned_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_LOCAL_VELOCITY_NED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_LOCAL_VELOCITY_NED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_local_velocity_ned_encode(system_id, component_id, &msg, &packet1);
    mace_msg_local_velocity_ned_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_local_velocity_ned_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.vx , packet1.vy , packet1.vz );
    mace_msg_local_velocity_ned_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_local_velocity_ned_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.vx , packet1.vy , packet1.vz );
    mace_msg_local_velocity_ned_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_local_velocity_ned_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_local_velocity_ned_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.vx , packet1.vy , packet1.vz );
    mace_msg_local_velocity_ned_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_local_state_full_ned(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_LOCAL_STATE_FULL_NED >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_local_state_full_ned_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0
    };
    mace_local_state_full_ned_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_LOCAL_STATE_FULL_NED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_LOCAL_STATE_FULL_NED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_local_state_full_ned_encode(system_id, component_id, &msg, &packet1);
    mace_msg_local_state_full_ned_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_local_state_full_ned_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz );
    mace_msg_local_state_full_ned_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_local_state_full_ned_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz );
    mace_msg_local_state_full_ned_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_local_state_full_ned_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_local_state_full_ned_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz );
    mace_msg_local_state_full_ned_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_global_position_int(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_GLOBAL_POSITION_INT >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_global_position_int_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,18275
    };
    mace_global_position_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.relative_alt = packet_in.relative_alt;
        packet1.hdg = packet_in.hdg;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_global_position_int_encode(system_id, component_id, &msg, &packet1);
    mace_msg_global_position_int_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_global_position_int_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.hdg );
    mace_msg_global_position_int_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_global_position_int_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.hdg );
    mace_msg_global_position_int_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_global_position_int_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_global_position_int_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.hdg );
    mace_msg_global_position_int_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_global_velocity_int(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_GLOBAL_VELOCITY_INT >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_global_velocity_int_t packet_in = {
        963497464,17443,17547,17651
    };
    mace_global_velocity_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_GLOBAL_VELOCITY_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_GLOBAL_VELOCITY_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_global_velocity_int_encode(system_id, component_id, &msg, &packet1);
    mace_msg_global_velocity_int_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_global_velocity_int_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.vx , packet1.vy , packet1.vz );
    mace_msg_global_velocity_int_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_global_velocity_int_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.vx , packet1.vy , packet1.vz );
    mace_msg_global_velocity_int_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_global_velocity_int_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_global_velocity_int_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.vx , packet1.vy , packet1.vz );
    mace_msg_global_velocity_int_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_global_position_state_full(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_global_position_state_full_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,18275,18379,18483,18587
    };
    mace_global_position_state_full_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.relative_alt = packet_in.relative_alt;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.hdg = packet_in.hdg;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_GLOBAL_POSITION_STATE_FULL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_global_position_state_full_encode(system_id, component_id, &msg, &packet1);
    mace_msg_global_position_state_full_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_global_position_state_full_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.relative_alt , packet1.hdg );
    mace_msg_global_position_state_full_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_global_position_state_full_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.relative_alt , packet1.hdg );
    mace_msg_global_position_state_full_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_global_position_state_full_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_global_position_state_full_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.vx , packet1.vy , packet1.vz , packet1.relative_alt , packet1.hdg );
    mace_msg_global_position_state_full_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_set_gps_global_origin(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_SET_GPS_GLOBAL_ORIGIN >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_set_gps_global_origin_t packet_in = {
        963497464,963497672,963497880,41
    };
    mace_set_gps_global_origin_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.altitude = packet_in.altitude;
        packet1.target_system = packet_in.target_system;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_SET_GPS_GLOBAL_ORIGIN_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_SET_GPS_GLOBAL_ORIGIN_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_set_gps_global_origin_encode(system_id, component_id, &msg, &packet1);
    mace_msg_set_gps_global_origin_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_set_gps_global_origin_pack(system_id, component_id, &msg , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude );
    mace_msg_set_gps_global_origin_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_set_gps_global_origin_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude );
    mace_msg_set_gps_global_origin_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_set_gps_global_origin_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_set_gps_global_origin_send(MACE_COMM_1 , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude );
    mace_msg_set_gps_global_origin_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_gps_global_origin(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_GPS_GLOBAL_ORIGIN >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_gps_global_origin_t packet_in = {
        963497464,963497672,963497880
    };
    mace_gps_global_origin_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.altitude = packet_in.altitude;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_gps_global_origin_encode(system_id, component_id, &msg, &packet1);
    mace_msg_gps_global_origin_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_gps_global_origin_pack(system_id, component_id, &msg , packet1.latitude , packet1.longitude , packet1.altitude );
    mace_msg_gps_global_origin_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_gps_global_origin_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.latitude , packet1.longitude , packet1.altitude );
    mace_msg_gps_global_origin_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_gps_global_origin_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_gps_global_origin_send(MACE_COMM_1 , packet1.latitude , packet1.longitude , packet1.altitude );
    mace_msg_gps_global_origin_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_vfr_hud(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_VFR_HUD >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_vfr_hud_t packet_in = {
        17.0,45.0,73.0,101.0,18067,18171
    };
    mace_vfr_hud_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.airspeed = packet_in.airspeed;
        packet1.groundspeed = packet_in.groundspeed;
        packet1.alt = packet_in.alt;
        packet1.climb = packet_in.climb;
        packet1.heading = packet_in.heading;
        packet1.throttle = packet_in.throttle;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_VFR_HUD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_VFR_HUD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vfr_hud_encode(system_id, component_id, &msg, &packet1);
    mace_msg_vfr_hud_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vfr_hud_pack(system_id, component_id, &msg , packet1.airspeed , packet1.groundspeed , packet1.heading , packet1.throttle , packet1.alt , packet1.climb );
    mace_msg_vfr_hud_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vfr_hud_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.airspeed , packet1.groundspeed , packet1.heading , packet1.throttle , packet1.alt , packet1.climb );
    mace_msg_vfr_hud_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_vfr_hud_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vfr_hud_send(MACE_COMM_1 , packet1.airspeed , packet1.groundspeed , packet1.heading , packet1.throttle , packet1.alt , packet1.climb );
    mace_msg_vfr_hud_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_command_int(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_COMMAND_INT >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_command_int_t packet_in = {
        17.0,45.0,73.0,101.0,963498296,963498504,185.0,18691,223,34,101,168,235
    };
    mace_command_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param1 = packet_in.param1;
        packet1.param2 = packet_in.param2;
        packet1.param3 = packet_in.param3;
        packet1.param4 = packet_in.param4;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.command = packet_in.command;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.frame = packet_in.frame;
        packet1.current = packet_in.current;
        packet1.autocontinue = packet_in.autocontinue;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_COMMAND_INT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_COMMAND_INT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_int_encode(system_id, component_id, &msg, &packet1);
    mace_msg_command_int_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_int_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
    mace_msg_command_int_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_int_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
    mace_msg_command_int_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_command_int_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_int_send(MACE_COMM_1 , packet1.target_system , packet1.target_component , packet1.frame , packet1.command , packet1.current , packet1.autocontinue , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.x , packet1.y , packet1.z );
    mace_msg_command_int_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_command_long(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_COMMAND_LONG >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_command_long_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,18691,223,34,101
    };
    mace_command_long_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param1 = packet_in.param1;
        packet1.param2 = packet_in.param2;
        packet1.param3 = packet_in.param3;
        packet1.param4 = packet_in.param4;
        packet1.param5 = packet_in.param5;
        packet1.param6 = packet_in.param6;
        packet1.param7 = packet_in.param7;
        packet1.command = packet_in.command;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.confirmation = packet_in.confirmation;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_COMMAND_LONG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_COMMAND_LONG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_long_encode(system_id, component_id, &msg, &packet1);
    mace_msg_command_long_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_long_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.command , packet1.confirmation , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.param7 );
    mace_msg_command_long_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_long_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.command , packet1.confirmation , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.param7 );
    mace_msg_command_long_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_command_long_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_long_send(MACE_COMM_1 , packet1.target_system , packet1.target_component , packet1.command , packet1.confirmation , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.param7 );
    mace_msg_command_long_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_command_short(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_COMMAND_SHORT >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_command_short_t packet_in = {
        17.0,17443,151,218,29
    };
    mace_command_short_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param = packet_in.param;
        packet1.command = packet_in.command;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.confirmation = packet_in.confirmation;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_COMMAND_SHORT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_COMMAND_SHORT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_short_encode(system_id, component_id, &msg, &packet1);
    mace_msg_command_short_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_short_pack(system_id, component_id, &msg , packet1.command , packet1.target_system , packet1.target_component , packet1.confirmation , packet1.param );
    mace_msg_command_short_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_short_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.command , packet1.target_system , packet1.target_component , packet1.confirmation , packet1.param );
    mace_msg_command_short_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_command_short_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_short_send(MACE_COMM_1 , packet1.command , packet1.target_system , packet1.target_component , packet1.confirmation , packet1.param );
    mace_msg_command_short_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_command_ack(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_COMMAND_ACK >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_command_ack_t packet_in = {
        17235,139
    };
    mace_command_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.command = packet_in.command;
        packet1.result = packet_in.result;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_COMMAND_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_COMMAND_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_ack_encode(system_id, component_id, &msg, &packet1);
    mace_msg_command_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_ack_pack(system_id, component_id, &msg , packet1.command , packet1.result );
    mace_msg_command_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_ack_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.command , packet1.result );
    mace_msg_command_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_command_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_ack_send(MACE_COMM_1 , packet1.command , packet1.result );
    mace_msg_command_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_command_system_mode(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_COMMAND_SYSTEM_MODE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_command_system_mode_t packet_in = {
        5,"BCDEFGHIJKLMNOPQRST"
    };
    mace_command_system_mode_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        
        mace_array_memcpy(packet1.mode, packet_in.mode, sizeof(char)*20);
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_COMMAND_SYSTEM_MODE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_COMMAND_SYSTEM_MODE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_system_mode_encode(system_id, component_id, &msg, &packet1);
    mace_msg_command_system_mode_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_system_mode_pack(system_id, component_id, &msg , packet1.target_system , packet1.mode );
    mace_msg_command_system_mode_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_system_mode_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.mode );
    mace_msg_command_system_mode_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_command_system_mode_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_command_system_mode_send(MACE_COMM_1 , packet1.target_system , packet1.mode );
    mace_msg_command_system_mode_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_system_mode_ack(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_SYSTEM_MODE_ACK >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_system_mode_ack_t packet_in = {
        5
    };
    mace_system_mode_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.result = packet_in.result;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_SYSTEM_MODE_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_SYSTEM_MODE_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_system_mode_ack_encode(system_id, component_id, &msg, &packet1);
    mace_msg_system_mode_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_system_mode_ack_pack(system_id, component_id, &msg , packet1.result );
    mace_msg_system_mode_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_system_mode_ack_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.result );
    mace_msg_system_mode_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_system_mode_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_system_mode_ack_send(MACE_COMM_1 , packet1.result );
    mace_msg_system_mode_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_execute_spatial_action(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_EXECUTE_SPATIAL_ACTION >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_execute_spatial_action_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,18691,18795,101,168,235,46
    };
    mace_execute_spatial_action_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param1 = packet_in.param1;
        packet1.param2 = packet_in.param2;
        packet1.param3 = packet_in.param3;
        packet1.param4 = packet_in.param4;
        packet1.param5 = packet_in.param5;
        packet1.param6 = packet_in.param6;
        packet1.param7 = packet_in.param7;
        packet1.action = packet_in.action;
        packet1.mask = packet_in.mask;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.frame = packet_in.frame;
        packet1.dimension = packet_in.dimension;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_EXECUTE_SPATIAL_ACTION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_EXECUTE_SPATIAL_ACTION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_execute_spatial_action_encode(system_id, component_id, &msg, &packet1);
    mace_msg_execute_spatial_action_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_execute_spatial_action_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.action , packet1.frame , packet1.dimension , packet1.mask , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.param7 );
    mace_msg_execute_spatial_action_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_execute_spatial_action_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.action , packet1.frame , packet1.dimension , packet1.mask , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.param7 );
    mace_msg_execute_spatial_action_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_execute_spatial_action_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_execute_spatial_action_send(MACE_COMM_1 , packet1.target_system , packet1.target_component , packet1.action , packet1.frame , packet1.dimension , packet1.mask , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.param5 , packet1.param6 , packet1.param7 );
    mace_msg_execute_spatial_action_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_execute_spatial_action_ack(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_EXECUTE_SPATIAL_ACTION_ACK >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_execute_spatial_action_ack_t packet_in = {
        5
    };
    mace_execute_spatial_action_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.result = packet_in.result;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_EXECUTE_SPATIAL_ACTION_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_EXECUTE_SPATIAL_ACTION_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_execute_spatial_action_ack_encode(system_id, component_id, &msg, &packet1);
    mace_msg_execute_spatial_action_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_execute_spatial_action_ack_pack(system_id, component_id, &msg , packet1.result );
    mace_msg_execute_spatial_action_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_execute_spatial_action_ack_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.result );
    mace_msg_execute_spatial_action_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_execute_spatial_action_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_execute_spatial_action_ack_send(MACE_COMM_1 , packet1.result );
    mace_msg_execute_spatial_action_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_radio_status(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_RADIO_STATUS >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_radio_status_t packet_in = {
        17235,17339,17,84,151,218,29
    };
    mace_radio_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.rxerrors = packet_in.rxerrors;
        packet1.fixed = packet_in.fixed;
        packet1.rssi = packet_in.rssi;
        packet1.remrssi = packet_in.remrssi;
        packet1.txbuf = packet_in.txbuf;
        packet1.noise = packet_in.noise;
        packet1.remnoise = packet_in.remnoise;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_RADIO_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_RADIO_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_radio_status_encode(system_id, component_id, &msg, &packet1);
    mace_msg_radio_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_radio_status_pack(system_id, component_id, &msg , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
    mace_msg_radio_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_radio_status_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
    mace_msg_radio_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_radio_status_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_radio_status_send(MACE_COMM_1 , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
    mace_msg_radio_status_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_timesync(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_TIMESYNC >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_timesync_t packet_in = {
        93372036854775807LL,93372036854776311LL
    };
    mace_timesync_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.tc1 = packet_in.tc1;
        packet1.ts1 = packet_in.ts1;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_TIMESYNC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_TIMESYNC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_timesync_encode(system_id, component_id, &msg, &packet1);
    mace_msg_timesync_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_timesync_pack(system_id, component_id, &msg , packet1.tc1 , packet1.ts1 );
    mace_msg_timesync_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_timesync_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.tc1 , packet1.ts1 );
    mace_msg_timesync_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_timesync_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_timesync_send(MACE_COMM_1 , packet1.tc1 , packet1.ts1 );
    mace_msg_timesync_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_power_status(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_POWER_STATUS >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_power_status_t packet_in = {
        17235,17339,17443
    };
    mace_power_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.Vcc = packet_in.Vcc;
        packet1.Vservo = packet_in.Vservo;
        packet1.flags = packet_in.flags;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_POWER_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_POWER_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_power_status_encode(system_id, component_id, &msg, &packet1);
    mace_msg_power_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_power_status_pack(system_id, component_id, &msg , packet1.Vcc , packet1.Vservo , packet1.flags );
    mace_msg_power_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_power_status_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.Vcc , packet1.Vservo , packet1.flags );
    mace_msg_power_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_power_status_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_power_status_send(MACE_COMM_1 , packet1.Vcc , packet1.Vservo , packet1.flags );
    mace_msg_power_status_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_distance_sensor(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_DISTANCE_SENSOR >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_distance_sensor_t packet_in = {
        963497464,17443,17547,17651,163,230,41,108
    };
    mace_distance_sensor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.min_distance = packet_in.min_distance;
        packet1.max_distance = packet_in.max_distance;
        packet1.current_distance = packet_in.current_distance;
        packet1.type = packet_in.type;
        packet1.id = packet_in.id;
        packet1.orientation = packet_in.orientation;
        packet1.covariance = packet_in.covariance;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_DISTANCE_SENSOR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_DISTANCE_SENSOR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_distance_sensor_encode(system_id, component_id, &msg, &packet1);
    mace_msg_distance_sensor_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_distance_sensor_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.min_distance , packet1.max_distance , packet1.current_distance , packet1.type , packet1.id , packet1.orientation , packet1.covariance );
    mace_msg_distance_sensor_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_distance_sensor_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.min_distance , packet1.max_distance , packet1.current_distance , packet1.type , packet1.id , packet1.orientation , packet1.covariance );
    mace_msg_distance_sensor_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_distance_sensor_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_distance_sensor_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.min_distance , packet1.max_distance , packet1.current_distance , packet1.type , packet1.id , packet1.orientation , packet1.covariance );
    mace_msg_distance_sensor_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_altitude(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_ALTITUDE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_altitude_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0
    };
    mace_altitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.altitude_monotonic = packet_in.altitude_monotonic;
        packet1.altitude_amsl = packet_in.altitude_amsl;
        packet1.altitude_local = packet_in.altitude_local;
        packet1.altitude_relative = packet_in.altitude_relative;
        packet1.altitude_terrain = packet_in.altitude_terrain;
        packet1.bottom_clearance = packet_in.bottom_clearance;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_ALTITUDE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_ALTITUDE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_altitude_encode(system_id, component_id, &msg, &packet1);
    mace_msg_altitude_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_altitude_pack(system_id, component_id, &msg , packet1.time_usec , packet1.altitude_monotonic , packet1.altitude_amsl , packet1.altitude_local , packet1.altitude_relative , packet1.altitude_terrain , packet1.bottom_clearance );
    mace_msg_altitude_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_altitude_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_usec , packet1.altitude_monotonic , packet1.altitude_amsl , packet1.altitude_local , packet1.altitude_relative , packet1.altitude_terrain , packet1.bottom_clearance );
    mace_msg_altitude_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_altitude_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_altitude_send(MACE_COMM_1 , packet1.time_usec , packet1.altitude_monotonic , packet1.altitude_amsl , packet1.altitude_local , packet1.altitude_relative , packet1.altitude_terrain , packet1.bottom_clearance );
    mace_msg_altitude_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_message_interval(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_MESSAGE_INTERVAL >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_message_interval_t packet_in = {
        963497464,17443
    };
    mace_message_interval_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.interval_us = packet_in.interval_us;
        packet1.message_id = packet_in.message_id;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_MESSAGE_INTERVAL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_MESSAGE_INTERVAL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_message_interval_encode(system_id, component_id, &msg, &packet1);
    mace_msg_message_interval_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_message_interval_pack(system_id, component_id, &msg , packet1.message_id , packet1.interval_us );
    mace_msg_message_interval_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_message_interval_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.message_id , packet1.interval_us );
    mace_msg_message_interval_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_message_interval_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_message_interval_send(MACE_COMM_1 , packet1.message_id , packet1.interval_us );
    mace_msg_message_interval_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_extended_sys_state(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_EXTENDED_SYS_STATE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_extended_sys_state_t packet_in = {
        5,72
    };
    mace_extended_sys_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vtol_state = packet_in.vtol_state;
        packet1.landed_state = packet_in.landed_state;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_extended_sys_state_encode(system_id, component_id, &msg, &packet1);
    mace_msg_extended_sys_state_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_extended_sys_state_pack(system_id, component_id, &msg , packet1.vtol_state , packet1.landed_state );
    mace_msg_extended_sys_state_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_extended_sys_state_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.vtol_state , packet1.landed_state );
    mace_msg_extended_sys_state_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_extended_sys_state_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_extended_sys_state_send(MACE_COMM_1 , packet1.vtol_state , packet1.landed_state );
    mace_msg_extended_sys_state_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_adsb_vehicle(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_ADSB_VEHICLE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_adsb_vehicle_t packet_in = {
        963497464,963497672,963497880,963498088,18067,18171,18275,18379,18483,211,"BCDEFGHI",113,180
    };
    mace_adsb_vehicle_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ICAO_address = packet_in.ICAO_address;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.altitude = packet_in.altitude;
        packet1.heading = packet_in.heading;
        packet1.hor_velocity = packet_in.hor_velocity;
        packet1.ver_velocity = packet_in.ver_velocity;
        packet1.flags = packet_in.flags;
        packet1.squawk = packet_in.squawk;
        packet1.altitude_type = packet_in.altitude_type;
        packet1.emitter_type = packet_in.emitter_type;
        packet1.tslc = packet_in.tslc;
        
        mace_array_memcpy(packet1.callsign, packet_in.callsign, sizeof(char)*9);
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_ADSB_VEHICLE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_ADSB_VEHICLE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_adsb_vehicle_encode(system_id, component_id, &msg, &packet1);
    mace_msg_adsb_vehicle_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_adsb_vehicle_pack(system_id, component_id, &msg , packet1.ICAO_address , packet1.lat , packet1.lon , packet1.altitude_type , packet1.altitude , packet1.heading , packet1.hor_velocity , packet1.ver_velocity , packet1.callsign , packet1.emitter_type , packet1.tslc , packet1.flags , packet1.squawk );
    mace_msg_adsb_vehicle_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_adsb_vehicle_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.ICAO_address , packet1.lat , packet1.lon , packet1.altitude_type , packet1.altitude , packet1.heading , packet1.hor_velocity , packet1.ver_velocity , packet1.callsign , packet1.emitter_type , packet1.tslc , packet1.flags , packet1.squawk );
    mace_msg_adsb_vehicle_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_adsb_vehicle_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_adsb_vehicle_send(MACE_COMM_1 , packet1.ICAO_address , packet1.lat , packet1.lon , packet1.altitude_type , packet1.altitude , packet1.heading , packet1.hor_velocity , packet1.ver_velocity , packet1.callsign , packet1.emitter_type , packet1.tslc , packet1.flags , packet1.squawk );
    mace_msg_adsb_vehicle_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_collision(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_COLLISION >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_collision_t packet_in = {
        963497464,45.0,73.0,101.0,53,120,187
    };
    mace_collision_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.id = packet_in.id;
        packet1.time_to_minimum_delta = packet_in.time_to_minimum_delta;
        packet1.altitude_minimum_delta = packet_in.altitude_minimum_delta;
        packet1.horizontal_minimum_delta = packet_in.horizontal_minimum_delta;
        packet1.src = packet_in.src;
        packet1.action = packet_in.action;
        packet1.threat_level = packet_in.threat_level;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_COLLISION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_COLLISION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_collision_encode(system_id, component_id, &msg, &packet1);
    mace_msg_collision_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_collision_pack(system_id, component_id, &msg , packet1.src , packet1.id , packet1.action , packet1.threat_level , packet1.time_to_minimum_delta , packet1.altitude_minimum_delta , packet1.horizontal_minimum_delta );
    mace_msg_collision_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_collision_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.src , packet1.id , packet1.action , packet1.threat_level , packet1.time_to_minimum_delta , packet1.altitude_minimum_delta , packet1.horizontal_minimum_delta );
    mace_msg_collision_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_collision_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_collision_send(MACE_COMM_1 , packet1.src , packet1.id , packet1.action , packet1.threat_level , packet1.time_to_minimum_delta , packet1.altitude_minimum_delta , packet1.horizontal_minimum_delta );
    mace_msg_collision_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_statustext(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_STATUSTEXT >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_statustext_t packet_in = {
        5,"BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX"
    };
    mace_statustext_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.severity = packet_in.severity;
        
        mace_array_memcpy(packet1.text, packet_in.text, sizeof(char)*50);
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_STATUSTEXT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_STATUSTEXT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_statustext_encode(system_id, component_id, &msg, &packet1);
    mace_msg_statustext_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_statustext_pack(system_id, component_id, &msg , packet1.severity , packet1.text );
    mace_msg_statustext_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_statustext_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.severity , packet1.text );
    mace_msg_statustext_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_statustext_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_statustext_send(MACE_COMM_1 , packet1.severity , packet1.text );
    mace_msg_statustext_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_camera_information(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_CAMERA_INFORMATION >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_camera_information_t packet_in = {
        963497464,45.0,73.0,101.0,18067,18171,65,{ 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163 },{ 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3 },68
    };
    mace_camera_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.focal_length = packet_in.focal_length;
        packet1.sensor_size_h = packet_in.sensor_size_h;
        packet1.sensor_size_v = packet_in.sensor_size_v;
        packet1.resolution_h = packet_in.resolution_h;
        packet1.resolution_v = packet_in.resolution_v;
        packet1.camera_id = packet_in.camera_id;
        packet1.lense_id = packet_in.lense_id;
        
        mace_array_memcpy(packet1.vendor_name, packet_in.vendor_name, sizeof(uint8_t)*32);
        mace_array_memcpy(packet1.model_name, packet_in.model_name, sizeof(uint8_t)*32);
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_CAMERA_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_CAMERA_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_information_encode(system_id, component_id, &msg, &packet1);
    mace_msg_camera_information_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_information_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.camera_id , packet1.vendor_name , packet1.model_name , packet1.focal_length , packet1.sensor_size_h , packet1.sensor_size_v , packet1.resolution_h , packet1.resolution_v , packet1.lense_id );
    mace_msg_camera_information_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_information_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.camera_id , packet1.vendor_name , packet1.model_name , packet1.focal_length , packet1.sensor_size_h , packet1.sensor_size_v , packet1.resolution_h , packet1.resolution_v , packet1.lense_id );
    mace_msg_camera_information_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_camera_information_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_information_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.camera_id , packet1.vendor_name , packet1.model_name , packet1.focal_length , packet1.sensor_size_h , packet1.sensor_size_v , packet1.resolution_h , packet1.resolution_v , packet1.lense_id );
    mace_msg_camera_information_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_camera_settings(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_CAMERA_SETTINGS >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_camera_settings_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,65,132,199,10,77,144,211,22
    };
    mace_camera_settings_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.aperture = packet_in.aperture;
        packet1.shutter_speed = packet_in.shutter_speed;
        packet1.iso_sensitivity = packet_in.iso_sensitivity;
        packet1.white_balance = packet_in.white_balance;
        packet1.camera_id = packet_in.camera_id;
        packet1.aperture_locked = packet_in.aperture_locked;
        packet1.shutter_speed_locked = packet_in.shutter_speed_locked;
        packet1.iso_sensitivity_locked = packet_in.iso_sensitivity_locked;
        packet1.white_balance_locked = packet_in.white_balance_locked;
        packet1.mode_id = packet_in.mode_id;
        packet1.color_mode_id = packet_in.color_mode_id;
        packet1.image_format_id = packet_in.image_format_id;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_CAMERA_SETTINGS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_CAMERA_SETTINGS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_settings_encode(system_id, component_id, &msg, &packet1);
    mace_msg_camera_settings_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_settings_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.camera_id , packet1.aperture , packet1.aperture_locked , packet1.shutter_speed , packet1.shutter_speed_locked , packet1.iso_sensitivity , packet1.iso_sensitivity_locked , packet1.white_balance , packet1.white_balance_locked , packet1.mode_id , packet1.color_mode_id , packet1.image_format_id );
    mace_msg_camera_settings_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_settings_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.camera_id , packet1.aperture , packet1.aperture_locked , packet1.shutter_speed , packet1.shutter_speed_locked , packet1.iso_sensitivity , packet1.iso_sensitivity_locked , packet1.white_balance , packet1.white_balance_locked , packet1.mode_id , packet1.color_mode_id , packet1.image_format_id );
    mace_msg_camera_settings_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_camera_settings_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_settings_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.camera_id , packet1.aperture , packet1.aperture_locked , packet1.shutter_speed , packet1.shutter_speed_locked , packet1.iso_sensitivity , packet1.iso_sensitivity_locked , packet1.white_balance , packet1.white_balance_locked , packet1.mode_id , packet1.color_mode_id , packet1.image_format_id );
    mace_msg_camera_settings_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_camera_capture_status(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_CAMERA_CAPTURE_STATUS >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_camera_capture_status_t packet_in = {
        963497464,45.0,73.0,963498088,129.0,18275,18379,18483,18587,89,156,223
    };
    mace_camera_capture_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.image_interval = packet_in.image_interval;
        packet1.video_framerate = packet_in.video_framerate;
        packet1.recording_time_ms = packet_in.recording_time_ms;
        packet1.available_capacity = packet_in.available_capacity;
        packet1.image_resolution_h = packet_in.image_resolution_h;
        packet1.image_resolution_v = packet_in.image_resolution_v;
        packet1.video_resolution_h = packet_in.video_resolution_h;
        packet1.video_resolution_v = packet_in.video_resolution_v;
        packet1.camera_id = packet_in.camera_id;
        packet1.image_status = packet_in.image_status;
        packet1.video_status = packet_in.video_status;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_capture_status_encode(system_id, component_id, &msg, &packet1);
    mace_msg_camera_capture_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_capture_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.camera_id , packet1.image_status , packet1.video_status , packet1.image_interval , packet1.video_framerate , packet1.image_resolution_h , packet1.image_resolution_v , packet1.video_resolution_h , packet1.video_resolution_v , packet1.recording_time_ms , packet1.available_capacity );
    mace_msg_camera_capture_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_capture_status_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.camera_id , packet1.image_status , packet1.video_status , packet1.image_interval , packet1.video_framerate , packet1.image_resolution_h , packet1.image_resolution_v , packet1.video_resolution_h , packet1.video_resolution_v , packet1.recording_time_ms , packet1.available_capacity );
    mace_msg_camera_capture_status_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_camera_capture_status_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_capture_status_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.camera_id , packet1.image_status , packet1.video_status , packet1.image_interval , packet1.video_framerate , packet1.image_resolution_h , packet1.image_resolution_v , packet1.video_resolution_h , packet1.video_resolution_v , packet1.recording_time_ms , packet1.available_capacity );
    mace_msg_camera_capture_status_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_camera_image_captured(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_CAMERA_IMAGE_CAPTURED >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_camera_image_captured_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,963498504,963498712,{ 213.0, 214.0, 215.0, 216.0 },963499752,149,216,"YZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRST"
    };
    mace_camera_image_captured_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_utc = packet_in.time_utc;
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.relative_alt = packet_in.relative_alt;
        packet1.image_index = packet_in.image_index;
        packet1.camera_id = packet_in.camera_id;
        packet1.capture_result = packet_in.capture_result;
        
        mace_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        mace_array_memcpy(packet1.file_url, packet_in.file_url, sizeof(char)*205);
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_image_captured_encode(system_id, component_id, &msg, &packet1);
    mace_msg_camera_image_captured_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_image_captured_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.time_utc , packet1.camera_id , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.q , packet1.image_index , packet1.capture_result , packet1.file_url );
    mace_msg_camera_image_captured_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_image_captured_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.time_utc , packet1.camera_id , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.q , packet1.image_index , packet1.capture_result , packet1.file_url );
    mace_msg_camera_image_captured_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_camera_image_captured_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_camera_image_captured_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.time_utc , packet1.camera_id , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.q , packet1.image_index , packet1.capture_result , packet1.file_url );
    mace_msg_camera_image_captured_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_flight_information(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_FLIGHT_INFORMATION >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_flight_information_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,963498712
    };
    mace_flight_information_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.arming_time_utc = packet_in.arming_time_utc;
        packet1.takeoff_time_utc = packet_in.takeoff_time_utc;
        packet1.flight_uuid = packet_in.flight_uuid;
        packet1.time_boot_ms = packet_in.time_boot_ms;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_FLIGHT_INFORMATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_FLIGHT_INFORMATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_flight_information_encode(system_id, component_id, &msg, &packet1);
    mace_msg_flight_information_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_flight_information_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.arming_time_utc , packet1.takeoff_time_utc , packet1.flight_uuid );
    mace_msg_flight_information_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_flight_information_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.time_boot_ms , packet1.arming_time_utc , packet1.takeoff_time_utc , packet1.flight_uuid );
    mace_msg_flight_information_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_flight_information_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_flight_information_send(MACE_COMM_1 , packet1.time_boot_ms , packet1.arming_time_utc , packet1.takeoff_time_utc , packet1.flight_uuid );
    mace_msg_flight_information_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_common(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
    mace_test_heartbeat(system_id, component_id, last_msg);
    mace_test_vehicle_mode(system_id, component_id, last_msg);
    mace_test_vehicle_armed(system_id, component_id, last_msg);
    mace_test_battery_status(system_id, component_id, last_msg);
    mace_test_system_time(system_id, component_id, last_msg);
    mace_test_ping(system_id, component_id, last_msg);
    mace_test_change_operator_control(system_id, component_id, last_msg);
    mace_test_change_operator_control_ack(system_id, component_id, last_msg);
    mace_test_auth_key(system_id, component_id, last_msg);
    mace_test_param_request_read(system_id, component_id, last_msg);
    mace_test_param_request_list(system_id, component_id, last_msg);
    mace_test_param_value(system_id, component_id, last_msg);
    mace_test_param_set(system_id, component_id, last_msg);
    mace_test_gps_raw_int(system_id, component_id, last_msg);
    mace_test_gps_status(system_id, component_id, last_msg);
    mace_test_scaled_pressure(system_id, component_id, last_msg);
    mace_test_attitude(system_id, component_id, last_msg);
    mace_test_attitude_rates(system_id, component_id, last_msg);
    mace_test_attitude_state_full(system_id, component_id, last_msg);
    mace_test_attitude_quaternion(system_id, component_id, last_msg);
    mace_test_local_position_ned(system_id, component_id, last_msg);
    mace_test_local_velocity_ned(system_id, component_id, last_msg);
    mace_test_local_state_full_ned(system_id, component_id, last_msg);
    mace_test_global_position_int(system_id, component_id, last_msg);
    mace_test_global_velocity_int(system_id, component_id, last_msg);
    mace_test_global_position_state_full(system_id, component_id, last_msg);
    mace_test_set_gps_global_origin(system_id, component_id, last_msg);
    mace_test_gps_global_origin(system_id, component_id, last_msg);
    mace_test_vfr_hud(system_id, component_id, last_msg);
    mace_test_command_int(system_id, component_id, last_msg);
    mace_test_command_long(system_id, component_id, last_msg);
    mace_test_command_short(system_id, component_id, last_msg);
    mace_test_command_ack(system_id, component_id, last_msg);
    mace_test_command_system_mode(system_id, component_id, last_msg);
    mace_test_system_mode_ack(system_id, component_id, last_msg);
    mace_test_execute_spatial_action(system_id, component_id, last_msg);
    mace_test_execute_spatial_action_ack(system_id, component_id, last_msg);
    mace_test_radio_status(system_id, component_id, last_msg);
    mace_test_timesync(system_id, component_id, last_msg);
    mace_test_power_status(system_id, component_id, last_msg);
    mace_test_distance_sensor(system_id, component_id, last_msg);
    mace_test_altitude(system_id, component_id, last_msg);
    mace_test_message_interval(system_id, component_id, last_msg);
    mace_test_extended_sys_state(system_id, component_id, last_msg);
    mace_test_adsb_vehicle(system_id, component_id, last_msg);
    mace_test_collision(system_id, component_id, last_msg);
    mace_test_statustext(system_id, component_id, last_msg);
    mace_test_camera_information(system_id, component_id, last_msg);
    mace_test_camera_settings(system_id, component_id, last_msg);
    mace_test_camera_capture_status(system_id, component_id, last_msg);
    mace_test_camera_image_captured(system_id, component_id, last_msg);
    mace_test_flight_information(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // COMMON_TESTSUITE_H
