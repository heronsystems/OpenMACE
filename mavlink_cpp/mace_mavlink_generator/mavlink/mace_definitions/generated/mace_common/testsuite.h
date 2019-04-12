/** @file
 *    @brief MAVLink comm protocol testsuite generated from mace_common.xml
 *    @see http://qgroundcontrol.org/mace/
 */
#pragma once
#ifndef MACE_COMMON_TESTSUITE_H
#define MACE_COMMON_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MACE_TEST_ALL
#define MACE_TEST_ALL
static void mace_test_common(uint8_t, uint8_t, mace_message_t *last_msg);
static void mace_test_mission(uint8_t, uint8_t, mace_message_t *last_msg);
static void mace_test_boundary(uint8_t, uint8_t, mace_message_t *last_msg);
static void mace_test_auctioneer(uint8_t, uint8_t, mace_message_t *last_msg);
static void mace_test_mace_common(uint8_t, uint8_t, mace_message_t *last_msg);

static void mace_test_all(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
    mace_test_common(system_id, component_id, last_msg);
    mace_test_mission(system_id, component_id, last_msg);
    mace_test_boundary(system_id, component_id, last_msg);
    mace_test_auctioneer(system_id, component_id, last_msg);
    mace_test_mace_common(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"
#include "../mission/testsuite.h"
#include "../boundary/testsuite.h"
#include "../auctioneer/testsuite.h"


static void mace_test_vehicle_sync(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_VEHICLE_SYNC >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_vehicle_sync_t packet_in = {
        5
    };
    mace_vehicle_sync_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_VEHICLE_SYNC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_VEHICLE_SYNC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vehicle_sync_encode(system_id, component_id, &msg, &packet1);
    mace_msg_vehicle_sync_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vehicle_sync_pack(system_id, component_id, &msg , packet1.target_system );
    mace_msg_vehicle_sync_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vehicle_sync_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system );
    mace_msg_vehicle_sync_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_vehicle_sync_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_vehicle_sync_send(MACE_COMM_1 , packet1.target_system );
    mace_msg_vehicle_sync_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_roi_ag(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_ROI_AG >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_roi_ag_t packet_in = {
        17.0,45.0,73.0,101.0,53,120,187,254,65
    };
    mace_roi_ag_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.stress_value = packet_in.stress_value;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.point_discovery = packet_in.point_discovery;
        packet1.stress_threshold = packet_in.stress_threshold;
        packet1.frame = packet_in.frame;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_ROI_AG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_ROI_AG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_roi_ag_encode(system_id, component_id, &msg, &packet1);
    mace_msg_roi_ag_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_roi_ag_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.point_discovery , packet1.stress_threshold , packet1.stress_value , packet1.frame , packet1.x , packet1.y , packet1.z );
    mace_msg_roi_ag_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_roi_ag_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.point_discovery , packet1.stress_threshold , packet1.stress_value , packet1.frame , packet1.x , packet1.y , packet1.z );
    mace_msg_roi_ag_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_roi_ag_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_roi_ag_send(MACE_COMM_1 , packet1.target_system , packet1.target_component , packet1.point_discovery , packet1.stress_threshold , packet1.stress_value , packet1.frame , packet1.x , packet1.y , packet1.z );
    mace_msg_roi_ag_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_mace_common(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
    mace_test_vehicle_sync(system_id, component_id, last_msg);
    mace_test_roi_ag(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MACE_COMMON_TESTSUITE_H
