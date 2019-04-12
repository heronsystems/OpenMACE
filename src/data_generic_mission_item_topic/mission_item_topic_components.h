#ifndef MISSION_ITEM_TOPIC_COMPONENTS_H
#define MISSION_ITEM_TOPIC_COMPONENTS_H

#define DATA_MISSION_GENERIC_TOPICS MissionTopic::VehicleTargetTopic, MissionTopic::MissionItemTopic, MissionTopic::MissionListTopic, MissionTopic::MissionHomeTopic, MissionTopic::MissionItemCurrentTopic, MissionTopic::MissionItemReachedTopic
#define DATA_MISSION_ITEM_REQUEST_TOPIC MissionTopic::MissionItemRequestTopic

#include "mission_item_topic.h"
#include "mission_list_topic.h"
#include "mission_home_topic.h"

#include "mission_item_request_topic.h"
#include "mission_item_current_topic.h"
#include "mission_item_reached_topic.h"

#include "vehicle_target_topic.h"
#endif // MISSION_ITEM_TOPIC_COMPONENTS_H
