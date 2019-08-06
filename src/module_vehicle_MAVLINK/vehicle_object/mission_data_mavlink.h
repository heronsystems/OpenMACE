#ifndef MISSION_DATA_MAVLINK_H
#define MISSION_DATA_MAVLINK_H

#include "data/data_get_set_notifier.h"

#include "data_generic_item_topic/data_generic_item_topic_components.h"
#include "data_generic_command_item/command_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

class MissionData_MAVLINK
{
public:
    MissionData_MAVLINK() = default;
    ~MissionData_MAVLINK() = default;

public:
    Data::DataGetSetNotifier<MissionItem::MissionList> currentAutoMission;

    Data::DataGetSetNotifier<MissionItem::MissionList> currentGuidedMission;
    Data::DataGetSetNotifier<TargetItem::DynamicMissionQueue> currentDynamicQueue_LocalCartesian;
    Data::DataGetSetNotifier<TargetItem::DynamicMissionQueue> currentDynamicQueue_GlobalCartesian;

public:
    Data::DataGetSetNotifier<command_item::SpatialHome> vehicleHomePosition;
    Data::DataGetSetNotifier<MissionItem::MissionItemAchieved> missionItemReached;
    Data::DataGetSetNotifier<MissionItem::MissionItemCurrent> missionItemCurrent;
};

#endif // MISSION_DATA_MAVLINK_H
