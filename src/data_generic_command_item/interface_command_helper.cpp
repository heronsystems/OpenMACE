#include "interface_command_helper.h"

template<>
void Interface_CommandHelper<mace_command_long_t>::initializeCommandItem(mace_command_long_t &obj) const
{
    obj.param1=0; /*< Parameter 1, as defined by MAV_CMD enum.*/
    obj.param2=0; /*< Parameter 2, as defined by MAV_CMD enum.*/
    obj.param3=0; /*< Parameter 3, as defined by MAV_CMD enum.*/
    obj.param4=0; /*< Parameter 4, as defined by MAV_CMD enum.*/
    obj.param5=0; /*< Parameter 5, as defined by MAV_CMD enum.*/
    obj.param6=0; /*< Parameter 6, as defined by MAV_CMD enum.*/
    obj.param7=0; /*< Parameter 7, as defined by MAV_CMD enum.*/
    obj.command=0; /*< Command ID, as defined by MAV_CMD enum.*/
    obj.target_system=0; /*< System which should execute the command*/
    obj.target_component=0; /*< Component which should execute the command, 0 for all components*/
    obj.confirmation=0; /*< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)*/
}

template<>
void Interface_CommandHelper<mace_command_short_t>::initializeCommandItem(mace_command_short_t &obj) const
{
    obj.param=0; /*< Parameter as defined by MAV_CMD enum.*/
    obj.command=0; /*< Command ID, as defined by MAV_CMD enum.is was established to reduce the bandwidth required of messages not requiring as much parameterized data.*/
    obj.target_system=0; /*< System which should execute the command*/
    obj.target_component=0; /*< Component which should execute the command, 0 for all components*/
    obj.confirmation=0; /*< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)*/
}

template<>
void Interface_CommandHelper<mace_command_long_t>::transferToMissionItem(const mace_command_long_t &cmdObj, mace_mission_item_t &misObj) const
{
    misObj.command = cmdObj.command;
    misObj.target_system = cmdObj.target_system;

    misObj.param1 = cmdObj.param1;
    misObj.param2 = cmdObj.param2;
    misObj.param3 = cmdObj.param3;
    misObj.param4 = cmdObj.param4;
    misObj.x = cmdObj.param5;
    misObj.y = cmdObj.param6;
    misObj.z = cmdObj.param7;
}

template<>
void Interface_CommandHelper<mace_command_long_t>::transferFromMissionItem(const mace_mission_item_t &misObj, mace_command_long_t &cmdObj) const
{
    cmdObj.command = misObj.command;
    cmdObj.target_system = misObj.target_system;

    cmdObj.param1 = misObj.param1;
    cmdObj.param2 = misObj.param2;
    cmdObj.param3 = misObj.param3;
    cmdObj.param4 = misObj.param4;
    cmdObj.param5 = misObj.x;
    cmdObj.param6 = misObj.y;
    cmdObj.param7 = misObj.z;
}

template<>
void Interface_CommandHelper<mace_command_short_t>::transferToMissionItem(const mace_command_short_t &cmdObj, mace_mission_item_t &misObj) const
{
    misObj.command = cmdObj.command;
    misObj.target_system = cmdObj.target_system;

    misObj.param1 = cmdObj.param;
}
template<>
void Interface_CommandHelper<mace_command_short_t>::transferFromMissionItem(const mace_mission_item_t &misObj, mace_command_short_t &cmdObj) const
{
    cmdObj.command = misObj.command;
    cmdObj.target_system = misObj.target_system;

    cmdObj.param = misObj.param1;
}

