#include "mission_item_factory.h"

namespace MissionItem {

command_item::AbstractCommandItemPtr MissionItemFactory::generateAbstractCommandItem(const mace_mission_item_t &maceItem, const unsigned int &targetID, const unsigned int originatingID)
{
    command_item::AbstractCommandItemPtr newMissionItem = nullptr;

    switch (static_cast<command_item::COMMANDTYPE>(maceItem.command)) {
    using namespace command_item;
    case COMMANDTYPE::CI_ACT_CHANGESPEED:
    {
        newMissionItem = std::make_shared<command_item::ActionChangeSpeed>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case COMMANDTYPE::CI_NAV_LAND:
    {
        newMissionItem = std::make_shared<command_item::SpatialLand>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case COMMANDTYPE::CI_NAV_LOITER_TIME:
    {
        newMissionItem = std::make_shared<command_item::SpatialLoiter_Time>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case COMMANDTYPE::CI_NAV_LOITER_TURNS:
    {
        newMissionItem = std::make_shared<command_item::SpatialLoiter_Turns>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case COMMANDTYPE::CI_NAV_LOITER_UNLIM:
    {
        newMissionItem = std::make_shared<command_item::SpatialLoiter_Unlimited>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case COMMANDTYPE::CI_NAV_RETURN_TO_LAUNCH:
    {
        newMissionItem = std::make_shared<command_item::SpatialRTL>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case COMMANDTYPE::CI_NAV_TAKEOFF:
    {
        newMissionItem = std::make_shared<command_item::SpatialTakeoff>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    }
    case COMMANDTYPE::CI_NAV_WAYPOINT:
        newMissionItem = std::make_shared<command_item::SpatialWaypoint>();
        newMissionItem->fromMACECOMMS_MissionItem(maceItem);
        break;
    case COMMANDTYPE::CI_NAV_HOME:
    case COMMANDTYPE::CI_ACT_ARM:
    case COMMANDTYPE::CI_ACT_CHANGEMODE:
    case COMMANDTYPE::CI_ACT_EXECUTE_SPATIAL_ITEM:
    case COMMANDTYPE::CI_ACT_MISSIONCMD:
    case COMMANDTYPE::CI_ACT_MOTORTEST:
    case COMMANDTYPE::CI_ACT_TARGET:
    case COMMANDTYPE::CI_ACT_MSG_INTERVAL:
    case COMMANDTYPE::CI_ACT_MSG_REQUEST:
    case COMMANDTYPE::CI_ACT_SET_GLOBAL_ORIGIN:
    case COMMANDTYPE::CI_ACT_HOME_POSITION:
    case COMMANDTYPE::COMMANDITEMEND:
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

void MissionItemFactory::generateMACEMissionItem(const command_item::AbstractCommandItemPtr maceItem, const unsigned int &itemIndex, mace_mission_item_t &missionItem)
{
    maceItem->populateMACECOMMS_MissionItem(missionItem);
    missionItem.seq = static_cast<uint16_t>(itemIndex);
}

void MissionItemFactory::updateMissionKey(const MissionItem::MissionKey &key, mace_mission_item_t &missionItem)
{
    missionItem.mission_system = static_cast<uint8_t>(key.m_systemID);
    missionItem.mission_creator = static_cast<uint8_t>(key.m_creatorID);
    missionItem.mission_id = static_cast<uint8_t>(key.m_missionID);
    missionItem.mission_type = static_cast<uint8_t>(key.m_missionType);
    missionItem.mission_state = static_cast<uint8_t>(key.m_missionState);
}


} //end of namespace MissionItem
