#ifndef INTERFACE_COMMAND_ITEM_H
#define INTERFACE_COMMAND_ITEM_H

#include "mace.h"
#include "command_item_type.h"

template <const command_item::COMMANDTYPE type, class T>
struct Interface_CommandHelper;

template <const command_item::COMMANDTYPE type>
struct Interface_CommandHelper<type, mace_command_long_t>
{
protected:
    void initializeCommandItem(mace_command_long_t &obj) const
    {
        obj.param1=0; /*< Parameter 1, as defined by MAV_CMD enum.*/
        obj.param2=0; /*< Parameter 2, as defined by MAV_CMD enum.*/
        obj.param3=0; /*< Parameter 3, as defined by MAV_CMD enum.*/
        obj.param4=0; /*< Parameter 4, as defined by MAV_CMD enum.*/
        obj.param5=0; /*< Parameter 5, as defined by MAV_CMD enum.*/
        obj.param6=0; /*< Parameter 6, as defined by MAV_CMD enum.*/
        obj.param7=0; /*< Parameter 7, as defined by MAV_CMD enum.*/
        obj.command=static_cast<uint8_t>(type); /*< Command ID, as defined by MAV_CMD enum.*/
        obj.target_system=0; /*< System which should execute the command*/
        obj.target_component=0; /*< Component which should execute the command, 0 for all components*/
        obj.confirmation=0; /*< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)*/
    }
};

template <const command_item::COMMANDTYPE type>
struct Interface_CommandHelper<type, mace_command_short_t>
{
protected:
    void initializeCommandItem(mace_command_short_t &obj) const
    {
        obj.param=0; /*< Parameter as defined by MAV_CMD enum.*/
        obj.command=static_cast<uint8_t>(type); /*< Command ID, as defined by MAV_CMD enum.is was established to reduce the bandwidth required of messages not requiring as much parameterized data.*/
        obj.target_system=0; /*< System which should execute the command*/
        obj.target_component=0; /*< Component which should execute the command, 0 for all components*/
        obj.confirmation=0; /*< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)*/
    }
};

template <const command_item::COMMANDTYPE type>
struct Interface_CommandHelper<type, mace_command_goto_t>
{
protected:
    void initializeCommandItem(mace_command_goto_t &obj) const
    {
        obj.param1=0; /*< Parameter 1, as defined by MAV_CMD enum.*/
        obj.param2=0; /*< Parameter 2, as defined by MAV_CMD enum.*/
        obj.param3=0; /*< Parameter 3, as defined by MAV_CMD enum.*/
        obj.param4=0; /*< Parameter 4, as defined by MAV_CMD enum.*/
        obj.param5=0; /*< Parameter 5, as defined by MAV_CMD enum.*/
        obj.param6=0; /*< Parameter 6, as defined by MAV_CMD enum.*/
        obj.param7=0; /*< Parameter 7, as defined by MAV_CMD enum.*/
        obj.action=static_cast<uint8_t>(type); /*< Command ID, as defined by MAV_CMD enum.*/
        obj.target_system=0; /*< System which should execute the command*/
        obj.target_component=0; /*< Component which should execute the command, 0 for all components*/
        obj.frame=0; /*< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h*/
    }
};

template <command_item::COMMANDTYPE type, class T>
class Interface_CommandItem : public Interface_CommandHelper<type, T>
{
public:
    Interface_CommandItem() = default;

    virtual ~Interface_CommandItem() = default;

public:
    virtual void toMACEComms_CommandItem(T &obj) const = 0;
};

#endif // INTERFACE_COMMAND_ITEM_H
