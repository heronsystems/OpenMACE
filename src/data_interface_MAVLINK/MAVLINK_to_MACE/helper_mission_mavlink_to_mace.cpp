#include "helper_mission_mavlink_to_mace.h"

namespace DataMAVLINK {

Helper_MissionMAVLINKtoMACE::Helper_MissionMAVLINKtoMACE(const int &originatingID):
    systemID(originatingID)
{

}

Helper_MissionMAVLINKtoMACE::~Helper_MissionMAVLINKtoMACE()
{

}

std::shared_ptr<CommandItem::AbstractCommandItem> Helper_MissionMAVLINKtoMACE::Convert_MAVLINKTOMACE(const mavlink_mission_item_t &mavlinkItem)
{
    return Helper_MissionMAVLINKtoMACE::Convert_MAVLINKTOMACE(systemID, mavlinkItem);
}

std::shared_ptr<CommandItem::AbstractCommandItem> Helper_MissionMAVLINKtoMACE::Convert_MAVLINKTOMACE(const int sysID, const mavlink_mission_item_t &mavlinkItem)
{
    std::shared_ptr<CommandItem::AbstractCommandItem> newMissionItem = NULL;

    switch(mavlinkItem.command)
    {
    case MAV_CMD_DO_CHANGE_SPEED:
    {
        CommandItem::ActionChangeSpeed missionItem;
        Helper_MissionMAVLINKtoMACE::convertChangespeed(sysID, mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::ActionChangeSpeed>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LAND:
    {
        CommandItem::SpatialLand missionItem;
        Helper_MissionMAVLINKtoMACE::convertLand(sysID, mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLand>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LOITER_TIME:
    {
        CommandItem::SpatialLoiter_Time missionItem;
        Helper_MissionMAVLINKtoMACE::convertLoiterTime(sysID, mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Time>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LOITER_TURNS:
    {
        CommandItem::SpatialLoiter_Turns missionItem;
        Helper_MissionMAVLINKtoMACE::convertLoiterTurns(sysID, mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Turns>(missionItem);
        break;
    }
    case MAV_CMD_NAV_LOITER_UNLIM:
    {
        CommandItem::SpatialLoiter_Unlimited missionItem;
        Helper_MissionMAVLINKtoMACE::convertLoiterUnlimted(sysID, mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialLoiter_Unlimited>(missionItem);
        break;
    }
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
    {
        CommandItem::SpatialRTL missionItem;
        Helper_MissionMAVLINKtoMACE::convertRTL(sysID, mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialRTL>(missionItem);
        break;
    }
    case MAV_CMD_NAV_TAKEOFF:
    {
        CommandItem::SpatialTakeoff missionItem;
        Helper_MissionMAVLINKtoMACE::convertTakeoff(sysID, mavlinkItem,missionItem);
        newMissionItem = std::make_shared<CommandItem::SpatialTakeoff>(missionItem);
        break;
    }
    case MAV_CMD_NAV_WAYPOINT:
    {
        CommandItem::SpatialWaypoint missionItem;
        Helper_MissionMAVLINKtoMACE::convertWaypoint(sysID, mavlinkItem,missionItem);
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
        newMissionItem->setOriginatingSystem(sysID);
        newMissionItem->setTargetSystem(sysID);
    }
    return newMissionItem;
}

void Helper_MissionMAVLINKtoMACE::convertHome(const int sysID, const mavlink_set_home_position_t &mavlinkItem, CommandItem::SpatialHome &missionItem)
{
    missionItem.setTargetSystem(sysID);
    missionItem.setOriginatingSystem(sysID);
    missionItem.position->setX(mavlinkItem.latitude / pow(10,7));
    missionItem.position->setY(mavlinkItem.longitude / pow(10,7));
    missionItem.position->setZ(mavlinkItem.altitude / pow(10,3));
}

void Helper_MissionMAVLINKtoMACE::convertChangespeed(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::ActionChangeSpeed &missionItem)
{
    missionItem.setTargetSystem(sysID);
    missionItem.setOriginatingSystem(sysID);
    if(mavlinkItem.command == MAV_CMD_DO_CHANGE_SPEED){
        missionItem.setDesiredSpeed(mavlinkItem.param2);
        if(mavlinkItem.param1 > 0.0)
        {
            missionItem.setSpeedFrame(Data::SpeedFrame::GROUNDSPEED);
        }else{
            missionItem.setSpeedFrame(Data::SpeedFrame::AIRSPEED);
        }
    }
}

void Helper_MissionMAVLINKtoMACE::convertLand(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLand &missionItem)
{
    missionItem.setTargetSystem(sysID);
    missionItem.setOriginatingSystem(sysID);
    missionItem.setPosition(getBasePosition(mavlinkItem));
}

void Helper_MissionMAVLINKtoMACE::convertLoiterTime(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Time &missionItem)
{
    missionItem.setTargetSystem(sysID);
    missionItem.setOriginatingSystem(sysID);
    missionItem.setPosition(getBasePosition(mavlinkItem));
    missionItem.duration = mavlinkItem.param1;
    missionItem.radius = fabs(mavlinkItem.param3);
    missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Helper_MissionMAVLINKtoMACE::convertLoiterTurns(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Turns &missionItem)
{
    missionItem.setTargetSystem(sysID);
    missionItem.setOriginatingSystem(sysID);
    missionItem.setPosition(getBasePosition(mavlinkItem));
    missionItem.turns = mavlinkItem.param1;
    missionItem.radius = fabs(mavlinkItem.param3);
    missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Helper_MissionMAVLINKtoMACE::convertLoiterUnlimted(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialLoiter_Unlimited &missionItem)
{
    missionItem.setTargetSystem(sysID);
    missionItem.setOriginatingSystem(sysID);
    missionItem.setPosition(getBasePosition(mavlinkItem));
    missionItem.radius = fabs(mavlinkItem.param3);
    missionItem.direction = (mavlinkItem.param3 > 0.0) ? Data::LoiterDirection::CW : Data::LoiterDirection::CCW;
}

void Helper_MissionMAVLINKtoMACE::convertRTL(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialRTL &missionItem)
{
    missionItem.setTargetSystem(sysID);
    missionItem.setOriginatingSystem(sysID);
    UNUSED(missionItem);
    if(mavlinkItem.command == MAV_CMD_NAV_RETURN_TO_LAUNCH){

    }
}

void Helper_MissionMAVLINKtoMACE::convertTakeoff(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialTakeoff &missionItem)
{
    missionItem.setTargetSystem(sysID);
    missionItem.setOriginatingSystem(sysID);
    missionItem.setPosition(getBasePosition(mavlinkItem));
}

void Helper_MissionMAVLINKtoMACE::convertWaypoint(const int sysID, const mavlink_mission_item_t &mavlinkItem, CommandItem::SpatialWaypoint &missionItem)
{
    missionItem.setTargetSystem(sysID);
    missionItem.setOriginatingSystem(sysID);
    missionItem.setPosition(getBasePosition(mavlinkItem));
}

DataState::Base3DPosition Helper_MissionMAVLINKtoMACE::getBasePosition(const mavlink_mission_item_t &mavlinkItem)
{
    DataState::Base3DPosition pos;
    Data::CoordinateFrameType frame = static_cast<Data::CoordinateFrameType>(mavlinkItem.frame);
    pos.setCoordinateFrame(frame);
    pos.setPosition3D(mavlinkItem.x,mavlinkItem.y,mavlinkItem.z);
    return pos;
}

} //end of namespace DataMAVLINK
