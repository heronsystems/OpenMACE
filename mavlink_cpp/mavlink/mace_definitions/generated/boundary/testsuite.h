/** @file
 *    @brief MAVLink comm protocol testsuite generated from boundary.xml
 *    @see http://qgroundcontrol.org/mace/
 */
#pragma once
#ifndef BOUNDARY_TESTSUITE_H
#define BOUNDARY_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MACE_TEST_ALL
#define MACE_TEST_ALL
static void mace_test_common(uint8_t, uint8_t, mace_message_t *last_msg);
static void mace_test_boundary(uint8_t, uint8_t, mace_message_t *last_msg);

static void mace_test_all(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
    mace_test_common(system_id, component_id, last_msg);
    mace_test_boundary(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mace_test_new_boundary_object(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_NEW_BOUNDARY_OBJECT >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_new_boundary_object_t packet_in = {
        5,72,139,206,17,84
    };
    mace_new_boundary_object_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.boundary_host_sysid = packet_in.boundary_host_sysid;
        packet1.boundary_host_compid = packet_in.boundary_host_compid;
        packet1.boundary_type = packet_in.boundary_type;
        packet1.boundary_identifier = packet_in.boundary_identifier;
        packet1.vehicle_aplicable = packet_in.vehicle_aplicable;
        packet1.num_vehicles = packet_in.num_vehicles;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_NEW_BOUNDARY_OBJECT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_NEW_BOUNDARY_OBJECT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_new_boundary_object_encode(system_id, component_id, &msg, &packet1);
    mace_msg_new_boundary_object_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_new_boundary_object_pack(system_id, component_id, &msg , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_type , packet1.boundary_identifier , packet1.vehicle_aplicable , packet1.num_vehicles );
    mace_msg_new_boundary_object_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_new_boundary_object_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_type , packet1.boundary_identifier , packet1.vehicle_aplicable , packet1.num_vehicles );
    mace_msg_new_boundary_object_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_new_boundary_object_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_new_boundary_object_send(MACE_COMM_1 , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_type , packet1.boundary_identifier , packet1.vehicle_aplicable , packet1.num_vehicles );
    mace_msg_new_boundary_object_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_boundary_ack(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_BOUNDARY_ACK >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_boundary_ack_t packet_in = {
        5,72,139,206
    };
    mace_boundary_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.boundary_host_sysid = packet_in.boundary_host_sysid;
        packet1.boundary_host_compid = packet_in.boundary_host_compid;
        packet1.boundary_identifier = packet_in.boundary_identifier;
        packet1.boundary_result = packet_in.boundary_result;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_BOUNDARY_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_BOUNDARY_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_ack_encode(system_id, component_id, &msg, &packet1);
    mace_msg_boundary_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_ack_pack(system_id, component_id, &msg , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier , packet1.boundary_result );
    mace_msg_boundary_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_ack_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier , packet1.boundary_result );
    mace_msg_boundary_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_boundary_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_ack_send(MACE_COMM_1 , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier , packet1.boundary_result );
    mace_msg_boundary_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_boundary_request_list(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_BOUNDARY_REQUEST_LIST >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_boundary_request_list_t packet_in = {
        5,72,139
    };
    mace_boundary_request_list_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.boundary_host_sysid = packet_in.boundary_host_sysid;
        packet1.boundary_host_compid = packet_in.boundary_host_compid;
        packet1.boundary_identifier = packet_in.boundary_identifier;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_BOUNDARY_REQUEST_LIST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_BOUNDARY_REQUEST_LIST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_request_list_encode(system_id, component_id, &msg, &packet1);
    mace_msg_boundary_request_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_request_list_pack(system_id, component_id, &msg , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier );
    mace_msg_boundary_request_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_request_list_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier );
    mace_msg_boundary_request_list_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_boundary_request_list_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_request_list_send(MACE_COMM_1 , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier );
    mace_msg_boundary_request_list_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_boundary_count(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_BOUNDARY_COUNT >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_boundary_count_t packet_in = {
        17235,139,206,17
    };
    mace_boundary_count_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.count = packet_in.count;
        packet1.boundary_host_sysid = packet_in.boundary_host_sysid;
        packet1.boundary_host_compid = packet_in.boundary_host_compid;
        packet1.boundary_identifier = packet_in.boundary_identifier;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_BOUNDARY_COUNT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_BOUNDARY_COUNT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_count_encode(system_id, component_id, &msg, &packet1);
    mace_msg_boundary_count_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_count_pack(system_id, component_id, &msg , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier , packet1.count );
    mace_msg_boundary_count_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_count_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier , packet1.count );
    mace_msg_boundary_count_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_boundary_count_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_count_send(MACE_COMM_1 , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier , packet1.count );
    mace_msg_boundary_count_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_boundary_request_item(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_BOUNDARY_REQUEST_ITEM >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_boundary_request_item_t packet_in = {
        17235,139,206,17
    };
    mace_boundary_request_item_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq = packet_in.seq;
        packet1.boundary_host_sysid = packet_in.boundary_host_sysid;
        packet1.boundary_host_compid = packet_in.boundary_host_compid;
        packet1.boundary_identifier = packet_in.boundary_identifier;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_BOUNDARY_REQUEST_ITEM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_request_item_encode(system_id, component_id, &msg, &packet1);
    mace_msg_boundary_request_item_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_request_item_pack(system_id, component_id, &msg , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier , packet1.seq );
    mace_msg_boundary_request_item_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_request_item_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier , packet1.seq );
    mace_msg_boundary_request_item_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_boundary_request_item_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_request_item_send(MACE_COMM_1 , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier , packet1.seq );
    mace_msg_boundary_request_item_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_boundary_item(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_BOUNDARY_ITEM >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_boundary_item_t packet_in = {
        17.0,45.0,73.0,17859,175,242,53,120
    };
    mace_boundary_item_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.seq = packet_in.seq;
        packet1.boundary_host_sysid = packet_in.boundary_host_sysid;
        packet1.boundary_host_compid = packet_in.boundary_host_compid;
        packet1.boundary_identifier = packet_in.boundary_identifier;
        packet1.frame = packet_in.frame;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_BOUNDARY_ITEM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_BOUNDARY_ITEM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_item_encode(system_id, component_id, &msg, &packet1);
    mace_msg_boundary_item_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_item_pack(system_id, component_id, &msg , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier , packet1.frame , packet1.x , packet1.y , packet1.z , packet1.seq );
    mace_msg_boundary_item_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_item_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier , packet1.frame , packet1.x , packet1.y , packet1.z , packet1.seq );
    mace_msg_boundary_item_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_boundary_item_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_boundary_item_send(MACE_COMM_1 , packet1.boundary_host_sysid , packet1.boundary_host_compid , packet1.boundary_identifier , packet1.frame , packet1.x , packet1.y , packet1.z , packet1.seq );
    mace_msg_boundary_item_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_boundary(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
    mace_test_new_boundary_object(system_id, component_id, last_msg);
    mace_test_boundary_ack(system_id, component_id, last_msg);
    mace_test_boundary_request_list(system_id, component_id, last_msg);
    mace_test_boundary_count(system_id, component_id, last_msg);
    mace_test_boundary_request_item(system_id, component_id, last_msg);
    mace_test_boundary_item(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // BOUNDARY_TESTSUITE_H
