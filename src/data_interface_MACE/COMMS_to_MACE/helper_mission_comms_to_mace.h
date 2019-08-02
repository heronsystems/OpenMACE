#ifndef HELPER_MISSION_COMMS_TO_MACE_H
#define HELPER_MISSION_COMMS_TO_MACE_H

#include <memory>

#include "mace.h"
#include "common/common.h"

#include "data/coordinate_frame.h"
#include "data/speed_frame.h"
#include "data/loiter_direction.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "mace_core/module_characteristics.h"

using namespace command_item;

namespace DataInterface_MACE {

class Helper_MissionCOMMStoMACE
{
public:
    Helper_MissionCOMMStoMACE();

    ~Helper_MissionCOMMStoMACE();

    static std::shared_ptr<command_item::AbstractCommandItem> Convert_COMMSTOMACE(const mace_mission_item_t &maceItem, const MaceCore::ModuleCharacteristic &target);

    static void convertHome(const mace_set_home_position_t &maceItem, command_item::SpatialHome &missionItem, const MaceCore::ModuleCharacteristic &target);

    static void convertChangespeed(const mace_mission_item_t &maceItem, command_item::ActionChangeSpeed &missionItem, const MaceCore::ModuleCharacteristic &target);

    static void convertLand(const mace_mission_item_t &maceItem, command_item::SpatialLand &missionItem, const MaceCore::ModuleCharacteristic &target);

    static void convertLoiterTime(const mace_mission_item_t &maceItem, command_item::SpatialLoiter_Time &missionItem, const MaceCore::ModuleCharacteristic &target);

    static void convertLoiterTurns(const mace_mission_item_t &maceItem, command_item::SpatialLoiter_Turns &missionItem, const MaceCore::ModuleCharacteristic &target);

    static void convertLoiterUnlimted(const mace_mission_item_t &maceItem, command_item::SpatialLoiter_Unlimited &missionItem, const MaceCore::ModuleCharacteristic &target);

    static void convertRTL(const mace_mission_item_t &maceItem, command_item::SpatialRTL &missionItem, const MaceCore::ModuleCharacteristic &target);

    static void convertTakeoff(const mace_mission_item_t &maceItem, command_item::SpatialTakeoff &missionItem, const MaceCore::ModuleCharacteristic &target);

    static void convertWaypoint(const mace_mission_item_t &maceItem, command_item::SpatialWaypoint &missionItem, const MaceCore::ModuleCharacteristic &target);

    static DataState::Base3DPosition getBasePosition(const mace_mission_item_t &maceItem, const MaceCore::ModuleCharacteristic &target);

    static void updatePosition(const mace_mission_item_t &maceItem, DataState::Base3DPosition &pos, const MaceCore::ModuleCharacteristic &target);
};

} //end of namespace DataInterface_MACE

#endif // HELPER_MISSION_COMMS_TO_MACE_H
