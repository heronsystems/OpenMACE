#include "helper_mission_mace_to_mavlink.h"

namespace DataMAVLINK{

Helper_MissionMACEtoMAVLINK::Helper_MissionMACEtoMAVLINK(const int &originatingSystem, const int &originatingComp):
    systemID(originatingSystem),compID(originatingComp)
{

}

Helper_MissionMACEtoMAVLINK::~Helper_MissionMACEtoMAVLINK()
{

}

void Helper_MissionMACEtoMAVLINK::initializeMAVLINKMissionItem(mavlink_mission_item_t &mavMission)
{
    mavMission.autocontinue = 1;
    mavMission.command = 0;
    mavMission.current = 0;
    mavMission.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    mavMission.param1 = 0.0;
    mavMission.param2 = 0.0;
    mavMission.param3 = 0.0;
    mavMission.param4 = 0.0;
    mavMission.seq = 0;
    mavMission.target_system = 0;
    mavMission.target_component = 0;
    mavMission.x = 0.0;
    mavMission.y = 0.0;
    mavMission.z = 0.0;
    mavMission.mission_type = MAV_MISSION_TYPE_MISSION;
}

mavlink_message_t Helper_MissionMACEtoMAVLINK::packMissionItem(const mavlink_mission_item_t &mavMission, const uint8_t &chan)
{
    mavlink_message_t msg;
    mavlink_mission_item_t tmpItem = mavMission;
    mavlink_msg_mission_item_encode_chan(systemID,compID,chan,&msg,&tmpItem);
    return msg;
}

bool Helper_MissionMACEtoMAVLINK::MACEMissionToMAVLINKMission(std::shared_ptr<command_item::AbstractCommandItem> missionItem, const uint16_t &itemIndex, const uint8_t &chan, mavlink_message_t &msg)
{
    mavlink_mission_item_t mavItem;
    bool rtnBool = this->MACEMissionToMAVLINKMission(missionItem,itemIndex,mavItem);
    msg = packMissionItem(mavItem,chan);
    return rtnBool;
}

bool Helper_MissionMACEtoMAVLINK::MACEMissionToMAVLINKMission(std::shared_ptr<command_item::AbstractCommandItem> missionItem, const uint16_t &itemIndex, mavlink_mission_item_t &mavItem)
{
    switch(missionItem->getCommandType())
    {
    case(MAV_CMD::MAV_CMD_DO_CHANGE_SPEED):
    {
        std::shared_ptr<command_item::ActionChangeSpeed> castItem = std::dynamic_pointer_cast<command_item::ActionChangeSpeed>(missionItem);
        command_item::ActionChangeSpeed baseItem = *castItem.get();
        mavItem = convertChangeSpeed(baseItem,itemIndex);
        return true;
        break;
    }
    case(MAV_CMD::MAV_CMD_NAV_LAND):
    {
        std::shared_ptr<command_item::SpatialLand> castItem = std::dynamic_pointer_cast<command_item::SpatialLand>(missionItem);
        command_item::SpatialLand baseItem = *castItem.get();
        mavItem = convertLand(baseItem,itemIndex);
        return true;
        break;
    }
    case(MAV_CMD::MAV_CMD_NAV_LOITER_TIME):
    {
        std::shared_ptr<command_item::SpatialLoiter_Time> castItem = std::dynamic_pointer_cast<command_item::SpatialLoiter_Time>(missionItem);
        command_item::SpatialLoiter_Time baseItem = *castItem.get();
        mavItem = convertLoiterTime(baseItem,itemIndex);
        return true;
        break;
    }
    case(MAV_CMD::MAV_CMD_NAV_LOITER_TURNS):
    {
        std::shared_ptr<command_item::SpatialLoiter_Turns> castItem = std::dynamic_pointer_cast<command_item::SpatialLoiter_Turns>(missionItem);
        command_item::SpatialLoiter_Turns baseItem = *castItem.get();
        mavItem = convertLoiterTurns(baseItem,itemIndex);
        return true;
        break;
    }
    case(MAV_CMD::MAV_CMD_NAV_LOITER_UNLIM):
    {
        std::shared_ptr<command_item::SpatialLoiter_Unlimited> castItem = std::dynamic_pointer_cast<command_item::SpatialLoiter_Unlimited>(missionItem);
        command_item::SpatialLoiter_Unlimited baseItem = *castItem.get();
        mavItem = convertLoiterUnlimited(baseItem,itemIndex);
        return true;
        break;
    }
    case(MAV_CMD::MAV_CMD_NAV_RETURN_TO_LAUNCH):
    {
        std::shared_ptr<command_item::SpatialRTL> castItem = std::dynamic_pointer_cast<command_item::SpatialRTL>(missionItem);
        command_item::SpatialRTL baseItem = *castItem.get();
        mavItem = convertRTL(baseItem,itemIndex);
        return true;
        break;
    }
    case(MAV_CMD::MAV_CMD_NAV_TAKEOFF):
    {
        std::shared_ptr<command_item::SpatialTakeoff> castItem = std::dynamic_pointer_cast<command_item::SpatialTakeoff>(missionItem);
        command_item::SpatialTakeoff baseItem = *castItem.get();
        mavItem = convertTakeoff(baseItem,itemIndex);
        return true;
        break;
    }
    case(MAV_CMD::MAV_CMD_NAV_WAYPOINT):
    {
        std::shared_ptr<command_item::SpatialWaypoint> castItem = std::dynamic_pointer_cast<command_item::SpatialWaypoint>(missionItem);
        command_item::SpatialWaypoint baseItem = *castItem.get();
        mavItem = convertWaypoint(baseItem,itemIndex);
        return true;
        break;
    }
    default:
        return false;
    } //end of switch statement
}

mavlink_mission_item_t Helper_MissionMACEtoMAVLINK::convertHome(const command_item::SpatialHome &missionItem)
{
    UNUSED(missionItem);

    mavlink_mission_item_t item;
    initializeMAVLINKMissionItem(item);
    item.frame = MAV_FRAME_GLOBAL;
    item.command = MAV_CMD_NAV_WAYPOINT;
    if(missionItem.position->isAnyPositionValid())
    {
        item.param1 = 0; //denotes to use specified location
        updateMissionPosition(missionItem.position,item);
    }
    else
        item.param1 = 1; //denotes to use current location

    return item;
}
mavlink_mission_item_t Helper_MissionMACEtoMAVLINK::convertChangeSpeed(const command_item::ActionChangeSpeed &missionItem, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_DO_CHANGE_SPEED;
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

mavlink_mission_item_t Helper_MissionMACEtoMAVLINK::convertLand(const command_item::SpatialLand &missionItem, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LAND;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    updateMissionPosition(missionItem.position,item);
    return item;
}

mavlink_mission_item_t Helper_MissionMACEtoMAVLINK::convertLoiterTime(const command_item::SpatialLoiter_Time &missionItem, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LOITER_TIME;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.param1 = missionItem.duration;
    updateMissionPosition(missionItem.position,item);

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    return item;
}

mavlink_mission_item_t Helper_MissionMACEtoMAVLINK::convertLoiterTurns(const command_item::SpatialLoiter_Turns &missionItem, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LOITER_TURNS;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    item.param1 = missionItem.turns;
    updateMissionPosition(missionItem.position,item);

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    return item;
}

mavlink_mission_item_t Helper_MissionMACEtoMAVLINK::convertLoiterUnlimited(const command_item::SpatialLoiter_Unlimited &missionItem, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_LOITER_UNLIM;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    updateMissionPosition(missionItem.position,item);

    if(missionItem.direction == Data::LoiterDirection::CW)
    {
        item.param3 = missionItem.radius;
    }else{
        item.param3 = 0-missionItem.radius;
    }
    return item;
}

mavlink_mission_item_t Helper_MissionMACEtoMAVLINK::convertRTL(const command_item::SpatialRTL &missionItem, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    return item;
}

mavlink_mission_item_t Helper_MissionMACEtoMAVLINK::convertTakeoff(const command_item::SpatialTakeoff &missionItem, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_TAKEOFF;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    updateMissionPosition(missionItem.position,item);
    return item;
}

mavlink_mission_item_t Helper_MissionMACEtoMAVLINK::convertWaypoint(const command_item::SpatialWaypoint &missionItem, const uint16_t &itemIndex)
{
    mavlink_mission_item_t item;
    initializeMAVLINKMissionItem(item);
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.seq = itemIndex;
    item.target_system = missionItem.getTargetSystem();
    updateMissionPosition(missionItem.position,item);
    return item;
}

void Helper_MissionMACEtoMAVLINK::updateMissionPosition(const mace::pose::Position* pos, mavlink_mission_item_t &item)
{
    if(pos == nullptr)
        return;

    item.frame = static_cast<uint8_t>(pos->getExplicitCoordinateFrame());

    switch (pos->getCoordinateSystemType()) {
    case CoordinateSystemTypes::CARTESIAN:
    {
        if(pos->is2D())
        {
            item.x = pos->positionAs<mace::pose::CartesianPosition_2D>()->getXPosition();
            item.y = pos->positionAs<mace::pose::CartesianPosition_2D>()->getYPosition();
        }
        if(pos->is3D())
        {
            item.x = pos->positionAs<mace::pose::CartesianPosition_3D>()->getXPosition();
            item.y = pos->positionAs<mace::pose::CartesianPosition_3D>()->getYPosition();
            item.z = pos->positionAs<mace::pose::CartesianPosition_3D>()->getZPosition();
        }
        break;
    }
    case CoordinateSystemTypes::GEODETIC:
    {
        if(pos->is2D())
        {
            item.x = pos->positionAs<mace::pose::GeodeticPosition_3D>()->getLatitude();
            item.y = pos->positionAs<mace::pose::GeodeticPosition_3D>()->getLongitude();
        }
        if(pos->is3D())
        {
            item.x = pos->positionAs<mace::pose::GeodeticPosition_3D>()->getLatitude();
            item.y = pos->positionAs<mace::pose::GeodeticPosition_3D>()->getLongitude();
            item.z = pos->positionAs<mace::pose::GeodeticPosition_3D>()->getAltitude();
        }
        break;
    }
    default:
        break;
    }
}


} //end of namespace DataMAVLINK
