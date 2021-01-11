#include "mission_item_factory.h"

namespace MissionItem {

command_item::AbstractCommandItemPtr MissionItemFactory::generateAbstractCommandItem(const mavlink_mace_mission_item_int_t &maceItem, const unsigned int &targetID, const unsigned int originatingID)
{
    command_item::AbstractCommandItemPtr newMissionItem = nullptr;

    switch (static_cast<MAV_CMD>(maceItem.command)) {
    using namespace command_item;
    case MAV_CMD::MAV_CMD_DO_CHANGE_SPEED:
    {
        newMissionItem = std::make_shared<command_item::ActionChangeSpeed>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_LAND:
    {
        newMissionItem = std::make_shared<command_item::SpatialLand>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_LOITER_TIME:
    {
        newMissionItem = std::make_shared<command_item::SpatialLoiter_Time>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_LOITER_TURNS:
    {
        newMissionItem = std::make_shared<command_item::SpatialLoiter_Turns>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_LOITER_UNLIM:
    {
        newMissionItem = std::make_shared<command_item::SpatialLoiter_Unlimited>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_RETURN_TO_LAUNCH:
    {
        newMissionItem = std::make_shared<command_item::SpatialRTL>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_TAKEOFF:
    {
        newMissionItem = std::make_shared<command_item::SpatialTakeoff>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_WAYPOINT:
        newMissionItem = std::make_shared<command_item::SpatialWaypoint>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    default:
        break;
    }

    if(newMissionItem != nullptr)
    {
        newMissionItem->setTargetSystem(targetID);
        newMissionItem->setOriginatingSystem(originatingID);
    }

    return newMissionItem;
}

void MissionItemFactory::generateMACEMissionItem(const command_item::AbstractCommandItemPtr maceItem, const unsigned int &itemIndex, mavlink_mace_mission_item_int_t &missionItem)
{
    maceItem->populateMACECOMMS_MissionItem(missionItem);
    missionItem.seq = static_cast<uint16_t>(itemIndex);
}

void MissionItemFactory::updateMissionKey(const MissionItem::MissionKey &key, mavlink_mace_mission_item_int_t &missionItem)
{
    missionItem.target_system = static_cast<uint8_t>(key.m_systemID);
    missionItem.mission_creator = static_cast<uint8_t>(key.m_creatorID);
    missionItem.mission_id = static_cast<uint8_t>(key.m_missionID);
    missionItem.mission_type = static_cast<uint8_t>(key.m_missionType);
}


} //end of namespace MissionItem
