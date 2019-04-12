#include "command_interface_mavlink.h"

namespace DataInterface_MAVLINK {

CommandInterface_MAVLINK::CommandInterface_MAVLINK(const int &targetSystem, const int &targetComp):
    targetSystemID(targetSystem), targetCompID(targetComp)
{

}

void CommandInterface_MAVLINK::connectCallback_CommandLong(CallbackFunctionPtr_CmdLng cb, void *p)
{
    m_CBCmdLng = cb;
    m_p = p;
}

CommandInterface_MAVLINK::~CommandInterface_MAVLINK()
{
    m_p = NULL;
}

void CommandInterface_MAVLINK::getSystemHome(const int &compID)
{
    mavlink_command_long_t cmd = initializeCommandLong();
    cmd.command = MAV_CMD_GET_HOME_POSITION;
    cmd.target_system = targetSystemID;
    cmd.target_component = targetCompID;
    m_CBCmdLng(m_p,cmd);
}

mavlink_command_long_t CommandInterface_MAVLINK::initializeCommandLong()
{
    mavlink_command_long_t cmdLong;
    cmdLong.command = 0;
    cmdLong.confirmation = 0;
    cmdLong.param1 = 0.0;
    cmdLong.param2 = 0.0;
    cmdLong.param3 = 0.0;
    cmdLong.param4 = 0.0;
    cmdLong.param5 = 0.0;
    cmdLong.param6 = 0.0;
    cmdLong.param7 = 0.0;
    cmdLong.target_system = targetSystemID;
    cmdLong.target_component = targetCompID;
    return cmdLong;
}

mavlink_message_t CommandInterface_MAVLINK::packLongMessage(const mavlink_command_long_t &cmdLong, const int &originatingSystemID, const int &originatingCompID, const int &chan)
{
    mavlink_message_t msg;
    mavlink_command_long_t tmpItem = cmdLong;
    mavlink_msg_command_long_encode_chan(originatingSystemID,originatingCompID,chan,&msg,&tmpItem);
    return msg;
}

mavlink_message_t CommandInterface_MAVLINK::setNewMode(const int &newMode, const int &originatingSystemID, const int &chan)
{
    mavlink_message_t msg;
    //mavlink_set_mode_t mode;

    mavlink_msg_set_mode_pack_chan(originatingSystemID,0,chan,&msg,targetSystemID,MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,newMode);
    return msg;
}

mavlink_message_t CommandInterface_MAVLINK::setHomePosition(const CommandItem::SpatialHome &commandItem, const int &compID)
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd = initializeCommandLong();
    if(commandItem.position->isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
    {
        cmd.command = MAV_CMD_DO_SET_HOME;
        cmd.target_system = commandItem.getTargetSystem();
        cmd.target_component = compID;
        cmd.param5 = commandItem.position->getX();
        cmd.param6 = commandItem.position->getY();
        cmd.param7 = commandItem.position->getZ();
        m_CBCmdLng(m_p,cmd);
    }

    return msg;
}

void CommandInterface_MAVLINK::setSystemArm(const CommandItem::ActionArm &commandItem, const int &compID)
{
    mavlink_command_long_t cmd = initializeCommandLong();
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.target_system = commandItem.getTargetSystem();
    cmd.target_component = compID;
    cmd.param1 = commandItem.getRequestArm();
    m_CBCmdLng(m_p,cmd);
}

void CommandInterface_MAVLINK::setSystemTakeoff(const CommandItem::SpatialTakeoff &commandItem, const int &compID)
{
    mavlink_command_long_t cmd = initializeCommandLong();
    cmd.command = MAV_CMD_NAV_TAKEOFF;
    cmd.target_system = commandItem.getTargetSystem();
    cmd.target_component = compID;
    Data::CoordinateFrameType cf = commandItem.position->getCoordinateFrame();

    if(cf == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
    {
        cmd.param7 = commandItem.position->getZ();
    }
    m_CBCmdLng(m_p,cmd);
}

void CommandInterface_MAVLINK::setSystemLand(const CommandItem::SpatialLand &commandItem, const int &compID)
{
    mavlink_command_long_t cmd = initializeCommandLong();
    cmd.command = MAV_CMD_NAV_LAND;
    cmd.target_system = commandItem.getTargetSystem();
    cmd.target_component = compID;

    if(commandItem.position->isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
    {
        cmd.param5 = commandItem.position->getX() * pow(10,7);
        cmd.param6 = commandItem.position->getY() * pow(10,7);
        cmd.param7 = commandItem.position->getZ() * 1000;
    }
    m_CBCmdLng(m_p,cmd);
}

void CommandInterface_MAVLINK::setSystemRTL(const CommandItem::SpatialRTL &commandItem, const int &compID)
{
    mavlink_command_long_t cmd = initializeCommandLong();
    cmd.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    cmd.target_system = commandItem.getTargetSystem();
    cmd.target_component = compID;
    m_CBCmdLng(m_p,cmd);
}

} //end of namespace DataInterface_MAVLINK
