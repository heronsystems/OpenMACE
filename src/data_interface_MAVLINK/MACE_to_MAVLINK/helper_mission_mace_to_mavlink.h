#ifndef HELPER_MISSION_MACE_TO_MAVLINK_H
#define HELPER_MISSION_MACE_TO_MAVLINK_H

#include <memory>

#include "mavlink.h"
#include "common/common.h"

#include "data/coordinate_frame.h"
#include "data/speed_frame.h"
#include "data/loiter_direction.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataMAVLINK {

class Helper_MissionMACEtoMAVLINK
{
public:
    Helper_MissionMACEtoMAVLINK(const int &originatingSystem, const int &originatingComp);

    ~Helper_MissionMACEtoMAVLINK();

    bool MACEMissionToMAVLINKMission(std::shared_ptr<CommandItem::AbstractCommandItem> missionItem, const uint16_t &itemIndex, const uint8_t &chan, mavlink_message_t &msg);

    static bool MACEMissionToMAVLINKMission(std::shared_ptr<CommandItem::AbstractCommandItem> missionItem, const uint16_t &itemIndex, mavlink_mission_item_t &mavItem);

    static mavlink_mission_item_t convertHome(const CommandItem::SpatialHome &missionItem);

    static mavlink_mission_item_t convertChangeSpeed(const CommandItem::ActionChangeSpeed &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertLand(const CommandItem::SpatialLand &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertLoiterTime(const CommandItem::SpatialLoiter_Time &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertLoiterTurns(const CommandItem::SpatialLoiter_Turns &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertLoiterUnlimited(const CommandItem::SpatialLoiter_Unlimited &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertRTL(const CommandItem::SpatialRTL &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertTakeoff(const CommandItem::SpatialTakeoff &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertWaypoint(const CommandItem::SpatialWaypoint &missionItem, const uint16_t &itemIndex);

    static void updateMissionPosition(const DataState::Base3DPosition &pos, mavlink_mission_item_t &item);

protected:
    static void initializeMAVLINKMissionItem(mavlink_mission_item_t &mavMission);
    mavlink_message_t packMissionItem(const mavlink_mission_item_t &mavMission, const uint8_t &chan);

private:
    int systemID;
    int compID;
};

} //end of namespace DataMAVLINK
#endif // HELPER_MISSION_MACE_TO_MAVLINK_H
