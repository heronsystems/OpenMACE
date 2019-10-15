#ifndef HELPER_MISSION_MACE_TO_MAVLINK_H
#define HELPER_MISSION_MACE_TO_MAVLINK_H

#include <memory>

#include "mavlink.h"
#include "common/common.h"

#include "data/coordinate_frame.h"
#include "data/speed_frame.h"
#include "data/loiter_direction.h"

#include "base/pose/pose_components.h"
#include "data_generic_command_item/command_item_components.h"

namespace DataMAVLINK {

class Helper_MissionMACEtoMAVLINK
{
public:
    Helper_MissionMACEtoMAVLINK(const int &originatingSystem, const int &originatingComp);

    ~Helper_MissionMACEtoMAVLINK();

    bool MACEMissionToMAVLINKMission(std::shared_ptr<command_item::AbstractCommandItem> missionItem, const uint16_t &itemIndex, const uint8_t &chan, mavlink_message_t &msg);

    static bool MACEMissionToMAVLINKMission(std::shared_ptr<command_item::AbstractCommandItem> missionItem, const uint16_t &itemIndex, mavlink_mission_item_t &mavItem);

    static mavlink_mission_item_t convertHome(const command_item::SpatialHome &missionItem);

    static mavlink_mission_item_t convertChangeSpeed(const command_item::ActionChangeSpeed &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertLand(const command_item::SpatialLand &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertLoiterTime(const command_item::SpatialLoiter_Time &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertLoiterTurns(const command_item::SpatialLoiter_Turns &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertLoiterUnlimited(const command_item::SpatialLoiter_Unlimited &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertRTL(const command_item::SpatialRTL &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertTakeoff(const command_item::SpatialTakeoff &missionItem, const uint16_t &itemIndex);

    static mavlink_mission_item_t convertWaypoint(const command_item::SpatialWaypoint &missionItem, const uint16_t &itemIndex);

    static void updateMissionPosition(const mace::pose::Position* pos, mavlink_mission_item_t &item);

protected:
    static void initializeMAVLINKMissionItem(mavlink_mission_item_t &mavMission);
    mavlink_message_t packMissionItem(const mavlink_mission_item_t &mavMission, const uint8_t &chan);

private:
    unsigned int systemID;
    unsigned int compID;
};

} //end of namespace DataMAVLINK
#endif // HELPER_MISSION_MACE_TO_MAVLINK_H
