#ifndef MAVLINK_CONTROLLER_COMMAND_LAND_H
#define MAVLINK_CONTROLLER_COMMAND_LAND_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKUXVControllers {

namespace VehicleController {

class CommandLand : public Controller_GenericLongCommand<command_item::SpatialLand, MAV_CMD_NAV_LAND>
{
public:
    //!
    //! \brief CommandLand This constructor is used to communicate to an explicit vehicle instance
    //! \param cb
    //! \param queue
    //! \param linkChan
    //!
    CommandLand(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::SpatialLand, MAV_CMD_NAV_LAND>(cb, queue, linkChan, "CommandLand")
    {

    }

    virtual ~CommandLand() = default;

    virtual void FillCommand(const command_item::SpatialLand &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = commandItem.getTargetSystem();
//        if(commandItem.position->isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
//        {
//            cmd.param5 = commandItem.position->getX() * pow(10,7); //this doesnt mean anything for ardupilot
//            cmd.param6 = commandItem.position->getY() * pow(10,7); //this doesnt mean anything for ardupilot
//            cmd.param7 = commandItem.position->getZ() * 1000; //this doesnt mean anything for ardupilot
//        }
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::SpatialLand &data) const
    {
        data.setTargetSystem(message.target_system);
    }
};

} //end of namespace VehicleController

namespace ModuleController {

class CommandLand : public ModuleController_GenericLongCommand<command_item::SpatialLand, MAV_CMD_NAV_LAND>
{
public:
    //!
    //! \brief CommandLand This constructor is used to communicate to an explicit vehicle instance
    //! \param cb
    //! \param queue
    //! \param linkChan
    //!
    CommandLand(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        ModuleController_GenericLongCommand<command_item::SpatialLand, MAV_CMD_NAV_LAND>(cb, queue, linkChan, "CommandLand")
    {

    }

    virtual ~CommandLand() = default;

    virtual void FillCommand(const command_item::SpatialLand &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = commandItem.getTargetSystem();
//        if(commandItem.position->isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
//        {
//            cmd.param5 = commandItem.position->getX() * pow(10,7); //this doesnt mean anything for ardupilot
//            cmd.param6 = commandItem.position->getY() * pow(10,7); //this doesnt mean anything for ardupilot
//            cmd.param7 = commandItem.position->getZ() * 1000; //this doesnt mean anything for ardupilot
//        }
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::SpatialLand &data) const
    {
        data.setTargetSystem(message.target_system);
    }
};

} //end of namespace ModuleController

} //end of namespace MAVLINKUXVControllers

#endif // MAVLINK_CONTROLLER_COMMAND_LAND_H
