#ifndef AUCTIONEER_COMPONENTS_H
#define AUCTIONEER_COMPONENTS_H

#define BID_TOPIC BidBundleTopic

#include "data_auctioneer/bids/bid_bundle.h"
#include "data_auctioneer/bids/bid_bundle_topic.h"
#include "data_auctioneer/bids/bid_container.h"
#include "data_auctioneer/bids/bid_descriptor.h"

#define TASK_STATUS_TOPIC TaskStatusTopic

#include "data_tasks/task_key.h"
#include "data_auctioneer/task_status_topic.h"

#define BID_DESCRIPTOR_TOPIC BidDescriptorTopic

#include "data_auctioneer/bids/bid_descriptor_topic.h"

#define SCRUB_BID_TOPIC ScrubMessageTopic

#include "data_auctioneer/scrub_message.h"
#include "data_auctioneer/scrub_message_topic.h"

#endif // AUCTIONEER_COMPONENTS_H
