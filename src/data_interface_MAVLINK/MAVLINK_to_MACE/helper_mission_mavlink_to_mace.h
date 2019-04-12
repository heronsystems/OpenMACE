#ifndef HELPER_MISSION_MAVLINK_TO_MACE_H
#define HELPER_MISSION_MAVLINK_TO_MACE_H

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

class Helper_MissionMAVLINKtoMACE
{
public:
    Helper_MissionMAVLINKtoMACE(const int &originatingID);

    ~Helper_MissionMAVLINKtoMACE();

    std::shared_ptr<CommandItem::AbstractCommandItem> Convert_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem);

    static std::shared_ptr<CommandItem::AbstractCommandItem> Convert_MAVLINKTOMACE(const int systemID, const mavlink_mission_item_t &mavlinkItem);


    static void convertHome(const int sysID, const mavlink_set_home_position_t &mavlinkItem, CommandItem::SpatialHome &missionItem);

    static void convertChangespeed(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::ActionChangeSpeed &missionItem);

    static void convertLand(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLand &missionItem);

    static void convertLoiterTime(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Time &missionItem);

    static void convertLoiterTurns(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Turns &missionItem);

    static void convertLoiterUnlimted(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Unlimited &missionItem);

    static void convertRTL(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialRTL &missionItem);

    static void convertTakeoff(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialTakeoff &missionItem);

    static void convertWaypoint(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialWaypoint &missionItem);

    static DataState::Base3DPosition getBasePosition(const mavlink_mission_item_t &mavlinkItem);

private:
    int systemID;
};

} //end of namespace DataMAVLINK

#endif // HELPER_MISSION_MAVLINK_TO_MACE_H
