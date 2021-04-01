#ifndef HELPER_MISSION_MACE_TO_COMMS_H
#define HELPER_MISSION_MACE_TO_COMMS_H

#include <memory>

#include "mace.h"
#include "common/common.h"

#include "data/coordinate_frame.h"
#include "data/speed_frame.h"
#include "data/loiter_direction.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

namespace DataInterface_MACE {

class Helper_MissionMACEtoCOMMS
{
public:
    Helper_MissionMACEtoCOMMS();

    ~Helper_MissionMACEtoCOMMS();

    void updateIDS(const int &originatingSystem, const int &originatingComp);

    bool MACEMissionToCOMMSMission(std::shared_ptr<CommandItem::AbstractCommandItem> missionItem, const uint16_t &itemIndex, const uint8_t &chan, mace_message_t &msg);

    static bool MACEMissionToCOMMSMission(std::shared_ptr<CommandItem::AbstractCommandItem> missionItem, const uint16_t &itemIndex, mace_mission_item_t &mavItem);

    static mace_mission_item_t convertHome(const CommandItem::SpatialHome &missionItem);

    static mace_mission_item_t convertChangeSpeed(const CommandItem::ActionChangeSpeed &missionItem, const uint16_t &itemIndex);

    static mace_mission_item_t convertLand(const CommandItem::SpatialLand &missionItem, const uint16_t &itemIndex);

    static mace_mission_item_t convertLoiterTime(const CommandItem::SpatialLoiter_Time &missionItem, const uint16_t &itemIndex);

    static mace_mission_item_t convertLoiterTurns(const CommandItem::SpatialLoiter_Turns &missionItem, const uint16_t &itemIndex);

    static mace_mission_item_t convertLoiterUnlimited(const CommandItem::SpatialLoiter_Unlimited &missionItem, const uint16_t &itemIndex);

    static mace_mission_item_t convertRTL(const CommandItem::SpatialRTL &missionItem, const uint16_t &itemIndex);

    static mace_mission_item_t convertTakeoff(const CommandItem::SpatialTakeoff &missionItem, const uint16_t &itemIndex);

    static mace_mission_item_t convertWaypoint(const CommandItem::SpatialWaypoint &missionItem, const uint16_t &itemIndex);

    static void updateMissionPosition(const DataState::Base3DPosition &pos, mace_mission_item_t &item);

    static void updateMissionKey(const MissionItem::MissionKey &key, mace_mission_item_t &missionItem);

protected:
    static void initializeMACEMissionItem(mace_mission_item_t &mavMission);
    mace_message_t packMissionItem(const mace_mission_item_t &mavMission, const uint8_t &chan);

private:
    int systemID;
    int compID;
};

} //end of namespace DataInterface_MACE
#endif // HELPER_MISSION_MACE_TO_COMMS_H
