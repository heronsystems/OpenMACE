#ifndef MISSION_DATA_MAVLINK_OLD_H
#define MISSION_DATA_MAVLINK__OLD_H

#include "data/data_get_set_notifier.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataInterface_MAVLINK {

class MissionData_MAVLINK
{
public:
    MissionData_MAVLINK();

public:
    Data::DataGetSetNotifier<MissionItem::MissionList> currentAutoMission;
    Data::DataGetSetNotifier<MissionItem::MissionList> currentGuidedMission;

public:
    Data::DataGetSetNotifier<command_item::SpatialHome> home;
    Data::DataGetSetNotifier<MissionItem::MissionItemAchieved> missionItemReached;
    Data::DataGetSetNotifier<MissionItem::MissionItemCurrent> missionItemCurrent;

public:

    void setCurrentMission(const MissionItem::MissionList &missionList)
    {
        switch(missionList.getMissionType())
        {
        case MissionItem::MISSIONTYPE::AUTO:
        {
            currentAutoMission.set(missionList);
            break;
        }
        case MissionItem::MISSIONTYPE::GUIDED:
        {
            currentGuidedMission.set(missionList);
            break;
        }
        default:
            break;
        }
    }

    MissionItem::MissionKey getCurrentAutoMissionKey() const
    {
        return currentAutoMission.get().getMissionKey();
    }

    MissionItem::MissionKey getCurrentGuidedMissionKey() const
    {
        return currentGuidedMission.get().getMissionKey();
    }
};

} //end of namespace DataInterface_MAVLINK
#endif // MISSION_DATA_MAVLINK_H
