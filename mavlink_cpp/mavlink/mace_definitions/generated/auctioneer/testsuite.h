/** @file
 *    @brief MAVLink comm protocol testsuite generated from auctioneer.xml
 *    @see http://qgroundcontrol.org/mace/
 */
#pragma once
#ifndef AUCTIONEER_TESTSUITE_H
#define AUCTIONEER_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MACE_TEST_ALL
#define MACE_TEST_ALL
static void mace_test_common(uint8_t, uint8_t, mace_message_t *last_msg);
static void mace_test_auctioneer(uint8_t, uint8_t, mace_message_t *last_msg);

static void mace_test_all(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
    mace_test_common(system_id, component_id, last_msg);
    mace_test_auctioneer(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mace_test_auction_notify_bid_bundle(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_NOTIFY_BID_BUNDLE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_notify_bid_bundle_t packet_in = {
        93372036854775807ULL,179.0,53
    };
    mace_auction_notify_bid_bundle_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.agentID = packet_in.agentID;
        packet1.bundleGenTime = packet_in.bundleGenTime;
        packet1.numBids = packet_in.numBids;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_NOTIFY_BID_BUNDLE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_NOTIFY_BID_BUNDLE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_notify_bid_bundle_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_notify_bid_bundle_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_notify_bid_bundle_pack(system_id, component_id, &msg , packet1.agentID , packet1.bundleGenTime , packet1.numBids );
    mace_msg_auction_notify_bid_bundle_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_notify_bid_bundle_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.agentID , packet1.bundleGenTime , packet1.numBids );
    mace_msg_auction_notify_bid_bundle_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_notify_bid_bundle_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_notify_bid_bundle_send(MACE_COMM_1 , packet1.agentID , packet1.bundleGenTime , packet1.numBids );
    mace_msg_auction_notify_bid_bundle_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_bid_bundle_next_part(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_bid_bundle_next_part_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,235.0,291.0,347.0,403.0,459.0,93372036854779335ULL,571.0,221,32,99,166
    };
    mace_auction_bid_bundle_next_part_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.requestFrom = packet_in.requestFrom;
        packet1.agentID = packet_in.agentID;
        packet1.bundleGenTime = packet_in.bundleGenTime;
        packet1.utility = packet_in.utility;
        packet1.work = packet_in.work;
        packet1.cost = packet_in.cost;
        packet1.reward = packet_in.reward;
        packet1.creatorID = packet_in.creatorID;
        packet1.taskGenTime = packet_in.taskGenTime;
        packet1.taskID = packet_in.taskID;
        packet1.type = packet_in.type;
        packet1.priority = packet_in.priority;
        packet1.seqNum = packet_in.seqNum;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_BID_BUNDLE_NEXT_PART_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_bid_bundle_next_part_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_bid_bundle_next_part_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_bid_bundle_next_part_pack(system_id, component_id, &msg , packet1.requestFrom , packet1.agentID , packet1.bundleGenTime , packet1.utility , packet1.work , packet1.cost , packet1.reward , packet1.creatorID , packet1.taskID , packet1.taskGenTime , packet1.type , packet1.priority , packet1.seqNum );
    mace_msg_auction_bid_bundle_next_part_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_bid_bundle_next_part_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.requestFrom , packet1.agentID , packet1.bundleGenTime , packet1.utility , packet1.work , packet1.cost , packet1.reward , packet1.creatorID , packet1.taskID , packet1.taskGenTime , packet1.type , packet1.priority , packet1.seqNum );
    mace_msg_auction_bid_bundle_next_part_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_bid_bundle_next_part_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_bid_bundle_next_part_send(MACE_COMM_1 , packet1.requestFrom , packet1.agentID , packet1.bundleGenTime , packet1.utility , packet1.work , packet1.cost , packet1.reward , packet1.creatorID , packet1.taskID , packet1.taskGenTime , packet1.type , packet1.priority , packet1.seqNum );
    mace_msg_auction_bid_bundle_next_part_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_bid_bundle_rcv_part(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_bid_bundle_rcv_part_t packet_in = {
        93372036854775807ULL,179.0,53
    };
    mace_auction_bid_bundle_rcv_part_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.agentID = packet_in.agentID;
        packet1.bundleGenTime = packet_in.bundleGenTime;
        packet1.seqNum = packet_in.seqNum;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_BID_BUNDLE_RCV_PART_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_bid_bundle_rcv_part_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_bid_bundle_rcv_part_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_bid_bundle_rcv_part_pack(system_id, component_id, &msg , packet1.agentID , packet1.bundleGenTime , packet1.seqNum );
    mace_msg_auction_bid_bundle_rcv_part_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_bid_bundle_rcv_part_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.agentID , packet1.bundleGenTime , packet1.seqNum );
    mace_msg_auction_bid_bundle_rcv_part_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_bid_bundle_rcv_part_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_bid_bundle_rcv_part_send(MACE_COMM_1 , packet1.agentID , packet1.bundleGenTime , packet1.seqNum );
    mace_msg_auction_bid_bundle_rcv_part_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_bid_bundle_done(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_BID_BUNDLE_DONE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_bid_bundle_done_t packet_in = {
        93372036854775807ULL,179.0
    };
    mace_auction_bid_bundle_done_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.agentID = packet_in.agentID;
        packet1.bundleGenTime = packet_in.bundleGenTime;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_BID_BUNDLE_DONE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_BID_BUNDLE_DONE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_bid_bundle_done_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_bid_bundle_done_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_bid_bundle_done_pack(system_id, component_id, &msg , packet1.agentID , packet1.bundleGenTime );
    mace_msg_auction_bid_bundle_done_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_bid_bundle_done_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.agentID , packet1.bundleGenTime );
    mace_msg_auction_bid_bundle_done_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_bid_bundle_done_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_bid_bundle_done_send(MACE_COMM_1 , packet1.agentID , packet1.bundleGenTime );
    mace_msg_auction_bid_bundle_done_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_ack_bid_bundle(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_ack_bid_bundle_t packet_in = {
        93372036854775807ULL,179.0
    };
    mace_auction_ack_bid_bundle_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.agentID = packet_in.agentID;
        packet1.bundleGenTime = packet_in.bundleGenTime;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_ACK_BID_BUNDLE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_ack_bid_bundle_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_ack_bid_bundle_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_ack_bid_bundle_pack(system_id, component_id, &msg , packet1.agentID , packet1.bundleGenTime );
    mace_msg_auction_ack_bid_bundle_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_ack_bid_bundle_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.agentID , packet1.bundleGenTime );
    mace_msg_auction_ack_bid_bundle_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_ack_bid_bundle_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_ack_bid_bundle_send(MACE_COMM_1 , packet1.agentID , packet1.bundleGenTime );
    mace_msg_auction_ack_bid_bundle_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_new_task_key(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_NEW_TASK_KEY >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_new_task_key_t packet_in = {
        93372036854775807ULL,179.0,963498296,65
    };
    mace_auction_new_task_key_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.creatorID = packet_in.creatorID;
        packet1.taskGenTime = packet_in.taskGenTime;
        packet1.taskID = packet_in.taskID;
        packet1.type = packet_in.type;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_NEW_TASK_KEY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_NEW_TASK_KEY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_new_task_key_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_new_task_key_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_new_task_key_pack(system_id, component_id, &msg , packet1.creatorID , packet1.taskID , packet1.taskGenTime , packet1.type );
    mace_msg_auction_new_task_key_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_new_task_key_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.creatorID , packet1.taskID , packet1.taskGenTime , packet1.type );
    mace_msg_auction_new_task_key_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_new_task_key_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_new_task_key_send(MACE_COMM_1 , packet1.creatorID , packet1.taskID , packet1.taskGenTime , packet1.type );
    mace_msg_auction_new_task_key_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_task_key_ack(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_TASK_KEY_ACK >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_task_key_ack_t packet_in = {
        93372036854775807ULL,963497880
    };
    mace_auction_task_key_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.creatorID = packet_in.creatorID;
        packet1.taskID = packet_in.taskID;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_TASK_KEY_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_TASK_KEY_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_key_ack_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_task_key_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_key_ack_pack(system_id, component_id, &msg , packet1.creatorID , packet1.taskID );
    mace_msg_auction_task_key_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_key_ack_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.creatorID , packet1.taskID );
    mace_msg_auction_task_key_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_task_key_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_key_ack_send(MACE_COMM_1 , packet1.creatorID , packet1.taskID );
    mace_msg_auction_task_key_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_request_task_descriptor(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_request_task_descriptor_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,963498296
    };
    mace_auction_request_task_descriptor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.requestFrom = packet_in.requestFrom;
        packet1.creatorID = packet_in.creatorID;
        packet1.taskID = packet_in.taskID;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_request_task_descriptor_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_request_task_descriptor_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_request_task_descriptor_pack(system_id, component_id, &msg , packet1.requestFrom , packet1.creatorID , packet1.taskID );
    mace_msg_auction_request_task_descriptor_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_request_task_descriptor_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.requestFrom , packet1.creatorID , packet1.taskID );
    mace_msg_auction_request_task_descriptor_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_request_task_descriptor_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_request_task_descriptor_send(MACE_COMM_1 , packet1.requestFrom , packet1.creatorID , packet1.taskID );
    mace_msg_auction_request_task_descriptor_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_request_task_descriptor_ack(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_ACK >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_request_task_descriptor_ack_t packet_in = {
        93372036854775807ULL,963497880
    };
    mace_auction_request_task_descriptor_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.creatorID = packet_in.creatorID;
        packet1.taskID = packet_in.taskID;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_REQUEST_TASK_DESCRIPTOR_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_request_task_descriptor_ack_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_request_task_descriptor_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_request_task_descriptor_ack_pack(system_id, component_id, &msg , packet1.creatorID , packet1.taskID );
    mace_msg_auction_request_task_descriptor_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_request_task_descriptor_ack_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.creatorID , packet1.taskID );
    mace_msg_auction_request_task_descriptor_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_request_task_descriptor_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_request_task_descriptor_ack_send(MACE_COMM_1 , packet1.creatorID , packet1.taskID );
    mace_msg_auction_request_task_descriptor_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_task_descriptor(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_task_descriptor_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,235.0,291.0,347.0,963499544,137,204,15
    };
    mace_auction_task_descriptor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sentFrom = packet_in.sentFrom;
        packet1.creatorID = packet_in.creatorID;
        packet1.taskGenTime = packet_in.taskGenTime;
        packet1.reqStart = packet_in.reqStart;
        packet1.reqEnd = packet_in.reqEnd;
        packet1.penalty = packet_in.penalty;
        packet1.taskID = packet_in.taskID;
        packet1.type = packet_in.type;
        packet1.numParts = packet_in.numParts;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_task_descriptor_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_pack(system_id, component_id, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.taskGenTime , packet1.type , packet1.penalty , packet1.reqStart , packet1.reqEnd , packet1.numParts );
    mace_msg_auction_task_descriptor_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.taskGenTime , packet1.type , packet1.penalty , packet1.reqStart , packet1.reqEnd , packet1.numParts );
    mace_msg_auction_task_descriptor_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_task_descriptor_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_send(MACE_COMM_1 , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.taskGenTime , packet1.type , packet1.penalty , packet1.reqStart , packet1.reqEnd , packet1.numParts );
    mace_msg_auction_task_descriptor_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_task_descriptor_done(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_DONE >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_task_descriptor_done_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,53
    };
    mace_auction_task_descriptor_done_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sentFrom = packet_in.sentFrom;
        packet1.creatorID = packet_in.creatorID;
        packet1.taskID = packet_in.taskID;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_DONE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_DONE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_done_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_task_descriptor_done_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_done_pack(system_id, component_id, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID );
    mace_msg_auction_task_descriptor_done_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_done_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID );
    mace_msg_auction_task_descriptor_done_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_task_descriptor_done_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_done_send(MACE_COMM_1 , packet1.sentFrom , packet1.creatorID , packet1.taskID );
    mace_msg_auction_task_descriptor_done_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_task_descriptor_ack(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_ACK >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_task_descriptor_ack_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,53
    };
    mace_auction_task_descriptor_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sentFrom = packet_in.sentFrom;
        packet1.creatorID = packet_in.creatorID;
        packet1.taskID = packet_in.taskID;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_ack_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_task_descriptor_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_ack_pack(system_id, component_id, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID );
    mace_msg_auction_task_descriptor_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_ack_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID );
    mace_msg_auction_task_descriptor_ack_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_task_descriptor_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_ack_send(MACE_COMM_1 , packet1.sentFrom , packet1.creatorID , packet1.taskID );
    mace_msg_auction_task_descriptor_ack_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_task_descriptor_req_part(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_task_descriptor_req_part_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,53,120
    };
    mace_auction_task_descriptor_req_part_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sentFrom = packet_in.sentFrom;
        packet1.creatorID = packet_in.creatorID;
        packet1.taskID = packet_in.taskID;
        packet1.seqNum = packet_in.seqNum;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_REQ_PART_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_req_part_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_task_descriptor_req_part_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_req_part_pack(system_id, component_id, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.seqNum );
    mace_msg_auction_task_descriptor_req_part_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_req_part_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.seqNum );
    mace_msg_auction_task_descriptor_req_part_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_task_descriptor_req_part_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_req_part_send(MACE_COMM_1 , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.seqNum );
    mace_msg_auction_task_descriptor_req_part_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_task_descriptor_loiter_data(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LOITER_DATA >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_task_descriptor_loiter_data_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,235.0,291.0,347.0,963499544,137,204,15,82
    };
    mace_auction_task_descriptor_loiter_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sentFrom = packet_in.sentFrom;
        packet1.creatorID = packet_in.creatorID;
        packet1.xPos = packet_in.xPos;
        packet1.yPos = packet_in.yPos;
        packet1.zPos = packet_in.zPos;
        packet1.duration = packet_in.duration;
        packet1.taskID = packet_in.taskID;
        packet1.seqNum = packet_in.seqNum;
        packet1.coordinateType = packet_in.coordinateType;
        packet1.coordinateFrame = packet_in.coordinateFrame;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LOITER_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_LOITER_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_loiter_data_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_task_descriptor_loiter_data_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_loiter_data_pack(system_id, component_id, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.seqNum , packet1.xPos , packet1.yPos , packet1.zPos , packet1.duration , packet1.coordinateType , packet1.coordinateFrame );
    mace_msg_auction_task_descriptor_loiter_data_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_loiter_data_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.seqNum , packet1.xPos , packet1.yPos , packet1.zPos , packet1.duration , packet1.coordinateType , packet1.coordinateFrame );
    mace_msg_auction_task_descriptor_loiter_data_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_task_descriptor_loiter_data_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_loiter_data_send(MACE_COMM_1 , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.seqNum , packet1.xPos , packet1.yPos , packet1.zPos , packet1.duration , packet1.coordinateType , packet1.coordinateFrame );
    mace_msg_auction_task_descriptor_loiter_data_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_task_descriptor_part_survey_first(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_task_descriptor_part_survey_first_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,235.0,291.0,347.0,125,192,3
    };
    mace_auction_task_descriptor_part_survey_first_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sentFrom = packet_in.sentFrom;
        packet1.creatorID = packet_in.creatorID;
        packet1.sensorResolution = packet_in.sensorResolution;
        packet1.overlapHorizontal = packet_in.overlapHorizontal;
        packet1.overlapVertical = packet_in.overlapVertical;
        packet1.taskID = packet_in.taskID;
        packet1.seqNum = packet_in.seqNum;
        packet1.coordinateType = packet_in.coordinateType;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_part_survey_first_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_task_descriptor_part_survey_first_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_part_survey_first_pack(system_id, component_id, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.seqNum , packet1.sensorResolution , packet1.overlapHorizontal , packet1.overlapVertical , packet1.coordinateType );
    mace_msg_auction_task_descriptor_part_survey_first_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_part_survey_first_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.seqNum , packet1.sensorResolution , packet1.overlapHorizontal , packet1.overlapVertical , packet1.coordinateType );
    mace_msg_auction_task_descriptor_part_survey_first_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_task_descriptor_part_survey_first_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_part_survey_first_send(MACE_COMM_1 , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.seqNum , packet1.sensorResolution , packet1.overlapHorizontal , packet1.overlapVertical , packet1.coordinateType );
    mace_msg_auction_task_descriptor_part_survey_first_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auction_task_descriptor_part_survey(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
#ifdef MACE_STATUS_FLAG_OUT_MACE1
    mace_status_t *status = mace_get_channel_status(MACE_COMM_0);
        if ((status->flags & MACE_STATUS_FLAG_OUT_MACE1) && MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY >= 256) {
            return;
        }
#endif
    mace_message_t msg;
        uint8_t buffer[MACE_MAX_PACKET_LEN];
        uint16_t i;
    mace_auction_task_descriptor_part_survey_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,235.0,291.0,101,168
    };
    mace_auction_task_descriptor_part_survey_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sentFrom = packet_in.sentFrom;
        packet1.creatorID = packet_in.creatorID;
        packet1.xPos = packet_in.xPos;
        packet1.yPos = packet_in.yPos;
        packet1.taskID = packet_in.taskID;
        packet1.seqNum = packet_in.seqNum;
        
        
#ifdef MACE_STATUS_FLAG_OUT_MACE1
        if (status->flags & MACE_STATUS_FLAG_OUT_MACE1) {
           // cope with extensions
           memset(MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MACE_MSG_ID_AUCTION_TASK_DESCRIPTOR_PART_SURVEY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_part_survey_encode(system_id, component_id, &msg, &packet1);
    mace_msg_auction_task_descriptor_part_survey_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_part_survey_pack(system_id, component_id, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.seqNum , packet1.xPos , packet1.yPos );
    mace_msg_auction_task_descriptor_part_survey_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_part_survey_pack_chan(system_id, component_id, MACE_COMM_0, &msg , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.seqNum , packet1.xPos , packet1.yPos );
    mace_msg_auction_task_descriptor_part_survey_decode(&msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mace_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mace_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MACE_COMM_0, buffer[i]);
        }
    mace_msg_auction_task_descriptor_part_survey_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mace_msg_auction_task_descriptor_part_survey_send(MACE_COMM_1 , packet1.sentFrom , packet1.creatorID , packet1.taskID , packet1.seqNum , packet1.xPos , packet1.yPos );
    mace_msg_auction_task_descriptor_part_survey_decode(last_msg, &packet2);
        MACE_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mace_test_auctioneer(uint8_t system_id, uint8_t component_id, mace_message_t *last_msg)
{
    mace_test_auction_notify_bid_bundle(system_id, component_id, last_msg);
    mace_test_auction_bid_bundle_next_part(system_id, component_id, last_msg);
    mace_test_auction_bid_bundle_rcv_part(system_id, component_id, last_msg);
    mace_test_auction_bid_bundle_done(system_id, component_id, last_msg);
    mace_test_auction_ack_bid_bundle(system_id, component_id, last_msg);
    mace_test_auction_new_task_key(system_id, component_id, last_msg);
    mace_test_auction_task_key_ack(system_id, component_id, last_msg);
    mace_test_auction_request_task_descriptor(system_id, component_id, last_msg);
    mace_test_auction_request_task_descriptor_ack(system_id, component_id, last_msg);
    mace_test_auction_task_descriptor(system_id, component_id, last_msg);
    mace_test_auction_task_descriptor_done(system_id, component_id, last_msg);
    mace_test_auction_task_descriptor_ack(system_id, component_id, last_msg);
    mace_test_auction_task_descriptor_req_part(system_id, component_id, last_msg);
    mace_test_auction_task_descriptor_loiter_data(system_id, component_id, last_msg);
    mace_test_auction_task_descriptor_part_survey_first(system_id, component_id, last_msg);
    mace_test_auction_task_descriptor_part_survey(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // AUCTIONEER_TESTSUITE_H
