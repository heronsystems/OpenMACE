#include "helper_mission_mace_to_comms.h"

namespace DataInterface_MACE{

Helper_MissionMACEtoCOMMS::Helper_MissionMACEtoCOMMS()
{

}

Helper_MissionMACEtoCOMMS::~Helper_MissionMACEtoCOMMS()
{

}

void Helper_MissionMACEtoCOMMS::updateIDS(const int &originatingSystem, const int &originatingComp)
{
    this->systemID = originatingSystem;
    this->compID = originatingComp;
}

void Helper_MissionMACEtoCOMMS::initializeMACEMissionItem(mace_mission_item_t &mavMission)
{
    mavMission.seq = 0;
    mavMission.command = 0;
    mavMission.target_system = 0; //this is who this should go to
    mavMission.mission_system = 0; //this is who the mission item is about
    mavMission.mission_creator = 0; //this is who created the mission item
    mavMission.mission_id = 0;
    mavMission.mission_type = MAV_MISSION_TYPE_AUTO;
    mavMission.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    mavMission.current = 0;
    mavMission.autocontinue = 1;

    mavMission.param1 = 0.0;
    mavMission.param2 = 0.0;
    mavMission.param3 = 0.0;
    mavMission.param4 = 0.0;
    mavMission.x = 0.0;
    mavMission.y = 0.0;
    mavMission.z = 0.0;
}

mace_message_t Helper_MissionMACEtoCOMMS::packMissionItem(const mace_mission_item_t &mavMission, const uint8_t &chan)
{
    mace_message_t msg;
    mace_mission_item_t tmpItem = mavMission;
    mace_msg_mission_item_encode_chan(systemID,compID,chan,&msg,&tmpItem);
    return msg;
}

bool Helper_MissionMACEtoCOMMS::MACEMissionToCOMMSMission(std::shared_ptr<CommandItem::AbstractCommandItem> missionItem, const uint16_t &itemIndex, const uint8_t &chan, mace_message_t &msg)
{
    mace_mission_item_t mavItem;
    bool rtnBool = this->MACEMissionToCOMMSMission(missionItem,itemIndex,mavItem);
    msg = packMissionItem(mavItem,chan);
    return rtnBool;
}

bool Helper_MissionMACEtoCOMMS::MACEMissionToCOMMSMission(std::shared_ptr<CommandItem::AbstractCommandItem> missionItem, const uint16_t &itemIndex, mace_mission_item_t &mavItem)
{
    switch(missionItem->getCommandType())
    {
    case(CommandItem::COMMANDITEM::CI_ACT_CHANGESPEED):
    {
        std::shared_ptr<CommandItem::ActionChangeSpeed> castItem = std::dynamic_pointer_cast<CommandItem::ActionChangeSpeed>(missionItem);
        CommandItem::ActionChangeSpeed baseItem = *castItem.get();
        mavItem = convertChangeSpeed(baseItem,itemIndex);
        return true;
        break;
    }
    case(CommandItem::COMMANDITEM::CI_NAV_LAND):
    {
        std::shared_ptr<CommandItem::SpatialLand> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLand>(missionItem);
        CommandItem::SpatialLand baseItem = *castItem.get();
        mavItem = convertLand(baseItem,itemIndex);
        return true;
        break;
    }
    case(CommandItem::COMMANDITEM::CI_NAV_LOITER_TIME):
    {
        std::shared_ptr<CommandItem::SpatialLoiter_Time> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Time>(missionItem);
        CommandItem::SpatialLoiter_Time baseItem = *castItem.get();
        mavItem = convertLoiterTime(baseItem,itemIndex);
        return true;
        break;
    }
    case(CommandItem::COMMANDITEM::CI_NAV_LOITER_TURNS):
    {
        std::shared_ptr<CommandItem::SpatialLoiter_Turns> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Turns>(missionItem);
        CommandItem::SpatialLoiter_Turns baseItem = *castItem.get();
        mavItem = convertLoiterTurns(baseItem,itemIndex);
        return true;
        break;
    }
    case(CommandItem::COMMANDITEM::CI_NAV_LOITER_UNLIM):
    {
        std::shared_ptr<CommandItem::SpatialLoiter_Unlimited> castItem = std::dynamic_pointer_cast<CommandItem::SpatialLoiter_Unlimited>(missionItem);
        CommandItem::SpatialLoiter_Unlimited baseItem = *castItem.get();
        mavItem = convertLoiterUnlimited(baseItem,itemIndex);
        return true;
        break;
    }
    case(CommandItem::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH):
    {
        std::shared_ptr<CommandItem::SpatialRTL> castItem = std::dynamic_pointer_cast<CommandItem::SpatialRTL>(missionItem);
        CommandItem::SpatialRTL baseItem = *castItem.get();
        mavItem = convertRTL(baseItem,itemIndex);
        return true;
        break;
    }
    case(CommandItem::COMMANDITEM::CI_NAV_TAKEOFF):
    {
        std::shared_ptr<CommandItem::SpatialTakeoff> castItem = std::dynamic_pointer_cast<CommandItem::SpatialTakeoff>(missionItem);
        CommandItem::SpatialTakeoff baseItem = *castItem.get();
        mavItem = convertTakeoff(baseItem,itemIndex);
        return true;
        break;
    }
    case(CommandItem::COMMANDITEM::CI_NAV_WAYPOINT):
    {
        std::shared_ptr<CommandItem::SpatialWaypoint> castItem = std::dynamic_pointer_cast<CommandItem::SpatialWaypoint>(missionItem);
        CommandItem::SpatialWaypoint baseItem = *castItem.get();
        mavItem = convertWaypoint(baseItem,itemIndex);
        return true;
        break;
    }
    default:
        return false;
    } //end of switch statement
}

mace_mission_item_t Helper_MissionMACEtoCOMMS::convertHome(const CommandItem::SpatialHome &missionItem)
{
    mace_mission_item_t item;
    initializeMACEMissionItem(item);
    item.command = MAV_CMD_DO_SET_HOME;
    if(missionItem.position->has2DPositionSet())
    {
        item.param1 = 0; //denotes to use specified location
        updateMissionPosition(*missionItem.position,item);
    }
    else
        item.param1 = 1; //denotes to use current location

    return item;
}
mace_mission_item_t Helper_MissionMACEtoCOMMS::convertChangeSpeed(const CommandItem::ActionChangeSpeed &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    initializeMACEMissionItem(item);
    item.command = (uint16_t)COMMANDITEM::CI_ACT_CHANGESPEED;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.param1 = 0.0; //assume the default required is AIRSPEED
    item.param2 = missionItem.getDesiredSpeed();
    if(missionItem.getSpeedFrame() == Data::SpeedFrame::GROUNDSPEED)
    {
        item.param1 = 1.0;
    }
    return item;
}

mace_mission_item_t Helper_MissionMACEtoCOMMS::convertLand(const CommandItem::SpatialLand &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    initializeMACEMissionItem(item);
    item.command = (uint16_t)COMMANDITEM::CI_NAV_LAND;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    updateMissionPosition(*missionItem.position,item);
    return item;
}

mace_mission_item_t Helper_MissionMACEtoCOMMS::convertLoiterTime(const CommandItem::SpatialLoiter_Time &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    initializeMACEMissionItem(item);
    item.command = (uint16_t)COMMANDITEM::CI_NAV_LOITER_TIME;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.param1 = missionItem.duration;
    updateMissionPosition(*missionItem.position,item);

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    return item;
}

mace_mission_item_t Helper_MissionMACEtoCOMMS::convertLoiterTurns(const CommandItem::SpatialLoiter_Turns &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    initializeMACEMissionItem(item);
    item.command = (uint16_t)COMMANDITEM::CI_NAV_LOITER_TURNS;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.param1 = missionItem.turns;
    updateMissionPosition(*missionItem.position,item);

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    return item;
}

mace_mission_item_t Helper_MissionMACEtoCOMMS::convertLoiterUnlimited(const CommandItem::SpatialLoiter_Unlimited &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    initializeMACEMissionItem(item);
    item.command = (uint16_t)COMMANDITEM::CI_NAV_LOITER_UNLIM;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    updateMissionPosition(*missionItem.position,item);

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    return item;
}

mace_mission_item_t Helper_MissionMACEtoCOMMS::convertRTL(const CommandItem::SpatialRTL &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    initializeMACEMissionItem(item);
    item.command = (uint16_t)COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    return item;
}

mace_mission_item_t Helper_MissionMACEtoCOMMS::convertTakeoff(const CommandItem::SpatialTakeoff &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    initializeMACEMissionItem(item);
    item.command = (uint16_t)COMMANDITEM::CI_NAV_TAKEOFF;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    updateMissionPosition(*missionItem.position,item);
    return item;
}

mace_mission_item_t Helper_MissionMACEtoCOMMS::convertWaypoint(const CommandItem::SpatialWaypoint &missionItem, const uint16_t &itemIndex)
{
    mace_mission_item_t item;
    initializeMACEMissionItem(item);
    item.command = (uint16_t)COMMANDITEM::CI_NAV_TAKEOFF;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    updateMissionPosition(*missionItem.position,item);
    return item;
}

void Helper_MissionMACEtoCOMMS::updateMissionPosition(const DataState::Base3DPosition &pos, mace_mission_item_t &item)
{
    if(pos.getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT){
        item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    }
    else if(pos.getCoordinateFrame() == Data::CoordinateFrameType::CF_LOCAL_ENU)
    {
        item.frame = MAV_FRAME_LOCAL_ENU;
    }
    else{
        //KEN FIX THIS
        item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    }

    item.x = pos.getX();
    item.y = pos.getY();
    item.z = pos.getZ();
}

void Helper_MissionMACEtoCOMMS::updateMissionKey(const MissionItem::MissionKey &key, mace_mission_item_t &missionItem)
{
    missionItem.mission_system = key.m_systemID;
    missionItem.mission_creator = key.m_creatorID;
    missionItem.mission_id = key.m_missionID;
    missionItem.mission_type = static_cast<MAV_MISSION_TYPE>(key.m_missionType);
    missionItem.mission_state = static_cast<MAV_MISSION_STATE>(key.m_missionState);
}
} //end of namespace DataMAVLINK
