#include "helper_mission_comms_to_mace.h"

namespace DataInterface_MACE {

Helper_MissionCOMMStoMACE::Helper_MissionCOMMStoMACE()

{

}

Helper_MissionCOMMStoMACE::~Helper_MissionCOMMStoMACE()
{

}

std::shared_ptr<CommandItem::AbstractCommandItem> Helper_MissionCOMMStoMACE::Convert_COMMSTOMACE(const mace_mission_item_t &maceItem, const MaceCore::ModuleCharacteristic &target)
{
    std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = NULL;

    switch(static_cast<COMMANDITEM>(maceItem.command))
    {
    case COMMANDITEM::CI_ACT_CHANGESPEED:
    {
        CommandItem::ActionChangeSpeed missionItem;
        Helper_MissionCOMMStoMACE::convertChangespeed(maceItem,missionItem, target);
        newMissionItem = std::make_shared<CommandItem::ActionChangeSpeed>(missionItem);
        break;
    }
    case COMMANDITEM::CI_NAV_LAND:
    {
        CommandItem::SpatialLand missionItem;
        Helper_MissionCOMMStoMACE::convertLand(maceItem,missionItem, target);
        newMissionItem = std::make_shared<CommandItem::SpatialLand>(missionItem);
        break;
    }        
    case COMMANDITEM::CI_NAV_LOITER_TIME:
    {
        CommandItem::SpatialLoiter_Time missionItem;
        Helper_MissionCOMMStoMACE::convertLoiterTime(maceItem,missionItem, target);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Time>(missionItem);
        break;
    }
    case COMMANDITEM::CI_NAV_LOITER_TURNS:
    {
        CommandItem::SpatialLoiter_Turns missionItem;
        Helper_MissionCOMMStoMACE::convertLoiterTurns(maceItem,missionItem, target);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Turns>(missionItem);
        break;
    }
    case COMMANDITEM::CI_NAV_LOITER_UNLIM:
    {
        CommandItem::SpatialLoiter_Unlimited missionItem;
        Helper_MissionCOMMStoMACE::convertLoiterUnlimted(maceItem,missionItem, target);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Unlimited>(missionItem);
        break;
    }
    case COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH:
    {
        CommandItem::SpatialRTL missionItem;
        Helper_MissionCOMMStoMACE::convertRTL(maceItem,missionItem, target);
        newMissionItem = std::make_shared<CommandItem::SpatialRTL>(missionItem);
        break;
    }
    case COMMANDITEM::CI_NAV_TAKEOFF:
    {
        CommandItem::SpatialTakeoff missionItem;
        Helper_MissionCOMMStoMACE::convertTakeoff(maceItem,missionItem, target);
        newMissionItem = std::make_shared<CommandItem::SpatialTakeoff>(missionItem);
        break;
    }
    case COMMANDITEM::CI_NAV_WAYPOINT:
    {
        CommandItem::SpatialWaypoint missionItem;
        Helper_MissionCOMMStoMACE::convertWaypoint(maceItem,missionItem, target);
        newMissionItem = std::make_shared<CommandItem::SpatialWaypoint>(missionItem);
        break;
    }
    default:
    {
        break;
    }
    } //end of switch statement

    if(newMissionItem)
    {
        newMissionItem->setOriginatingSystem(target.ModuleID);
        newMissionItem->setTargetSystem(target.ModuleID);
    }
    return newMissionItem;
}


void Helper_MissionCOMMStoMACE::convertHome(const mace_set_home_position_t &maceItem, CommandItem::SpatialHome &missionItem, const MaceCore::ModuleCharacteristic &target)
{
    missionItem.setTargetSystem(target.ModuleID);
    missionItem.setOriginatingSystem(target.ModuleID);
    missionItem.position->setX(maceItem.latitude / pow(10,7));
    missionItem.position->setY(maceItem.longitude / pow(10,7));
    missionItem.position->setZ(maceItem.altitude / pow(10,3));
}

void Helper_MissionCOMMStoMACE::convertChangespeed(const mace_mission_item_t &maceItem, CommandItem::ActionChangeSpeed &missionItem, const MaceCore::ModuleCharacteristic &target)
{
    missionItem.setTargetSystem(target.ModuleID);
    missionItem.setOriginatingSystem(target.ModuleID);
    if(maceItem.command == MAV_CMD_DO_CHANGE_SPEED){
        missionItem.setDesiredSpeed(maceItem.param2);
        if(maceItem.param1 > 0.0)
        {
            missionItem.setSpeedFrame(Data::SpeedFrame::GROUNDSPEED);
        }else{
            missionItem.setSpeedFrame(Data::SpeedFrame::AIRSPEED);
        }
    }
}

void Helper_MissionCOMMStoMACE::convertLand(const mace_mission_item_t &maceItem, CommandItem::SpatialLand &missionItem, const MaceCore::ModuleCharacteristic &target)
{
    missionItem.setTargetSystem(target.ModuleID);
    missionItem.setOriginatingSystem(target.ModuleID);
    missionItem.setPosition(Helper_MissionCOMMStoMACE::getBasePosition(maceItem, target));
}

void Helper_MissionCOMMStoMACE::convertLoiterTime(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Time &missionItem, const MaceCore::ModuleCharacteristic &target)
{
    missionItem.setTargetSystem(target.ModuleID);
    missionItem.setOriginatingSystem(target.ModuleID);
    missionItem.setPosition(Helper_MissionCOMMStoMACE::getBasePosition(maceItem, target));
    missionItem.duration = maceItem.param1;
    missionItem.radius = fabs(maceItem.param3);
    missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Helper_MissionCOMMStoMACE::convertLoiterTurns(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Turns &missionItem, const MaceCore::ModuleCharacteristic &target)
{
    missionItem.setTargetSystem(target.ModuleID);
    missionItem.setOriginatingSystem(target.ModuleID);
    missionItem.setPosition(Helper_MissionCOMMStoMACE::getBasePosition(maceItem, target));
    missionItem.turns = maceItem.param1;
    missionItem.radius = fabs(maceItem.param3);
    missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Helper_MissionCOMMStoMACE::convertLoiterUnlimted(const mace_mission_item_t &maceItem, CommandItem::SpatialLoiter_Unlimited &missionItem, const MaceCore::ModuleCharacteristic &target)
{
    missionItem.setTargetSystem(target.ModuleID);
    missionItem.setOriginatingSystem(target.ModuleID);
    if((maceItem.command == MAV_CMD_NAV_LOITER_UNLIM) && (maceItem.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)){
        missionItem.setPosition(Helper_MissionCOMMStoMACE::getBasePosition(maceItem, target));
        missionItem.radius = fabs(maceItem.param3);
        missionItem.direction = (maceItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
    }
}

void Helper_MissionCOMMStoMACE::convertRTL(const mace_mission_item_t &maceItem, CommandItem::SpatialRTL &missionItem, const MaceCore::ModuleCharacteristic &target)
{
    missionItem.setTargetSystem(target.ModuleID);
    missionItem.setOriginatingSystem(target.ModuleID);
    UNUSED(missionItem);
    if(maceItem.command == MAV_CMD_NAV_RETURN_TO_LAUNCH){

    }
}

void Helper_MissionCOMMStoMACE::convertTakeoff(const mace_mission_item_t &maceItem, CommandItem::SpatialTakeoff &missionItem, const MaceCore::ModuleCharacteristic &target)
{
    missionItem.setTargetSystem(target.ModuleID);
    missionItem.setOriginatingSystem(target.ModuleID);
    missionItem.setPosition(Helper_MissionCOMMStoMACE::getBasePosition(maceItem, target));
}

void Helper_MissionCOMMStoMACE::convertWaypoint(const mace_mission_item_t &maceItem, CommandItem::SpatialWaypoint &missionItem, const MaceCore::ModuleCharacteristic &target)
{
    missionItem.setTargetSystem(target.ModuleID);
    missionItem.setOriginatingSystem(target.ModuleID);
    missionItem.setPosition(Helper_MissionCOMMStoMACE::getBasePosition(maceItem, target));
}

DataState::Base3DPosition Helper_MissionCOMMStoMACE::getBasePosition(const mace_mission_item_t &maceItem, const MaceCore::ModuleCharacteristic &target)
{
    DataState::Base3DPosition pos;
    Data::CoordinateFrameType frame = static_cast<Data::CoordinateFrameType>(maceItem.frame);
    pos.setCoordinateFrame(frame);
    pos.setPosition3D(maceItem.x,maceItem.y,maceItem.z);
    return pos;
}

} //end of namespace DataMAVLINK
