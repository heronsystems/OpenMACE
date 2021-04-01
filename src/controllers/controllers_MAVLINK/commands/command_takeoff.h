#ifndef MAVLINK_CONTROLLER_COMMAND_TAKEOFF_H
#define MAVLINK_CONTROLLER_COMMAND_TAKEOFF_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKUXVControllers {

namespace VehicleController {

class CommandTakeoff : public Controller_GenericLongCommand<command_item::SpatialTakeoff, MAV_CMD_NAV_TAKEOFF>
{
public:

    //!
    //! \brief CommandTakeoff This constructor is used to communicate to an explicit vehicle instance
    //! \param cb
    //! \param queue
    //! \param linkChan
    //!
    CommandTakeoff(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::SpatialTakeoff, MAV_CMD_NAV_TAKEOFF>(cb, queue, linkChan, "CommandTakeoff")
    {

    }

    virtual ~CommandTakeoff() = default;

    virtual void FillCommand(const command_item::SpatialTakeoff &commandItem, mavlink_command_long_t &cmd) const
    {
        const mace::pose::Position* takeoffPosition = commandItem.getPosition();
        if(takeoffPosition->isAnyPositionValid())
        {
            //The only way the ardupilot can handle this is if the translational component is in the geodetic frame
            if(takeoffPosition->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC)
            {
                if(takeoffPosition->is3D()) //This is going to be the only type we can handle here
                {
                    const mace::pose::GeodeticPosition_3D* castPosition = takeoffPosition->positionAs<mace::pose::GeodeticPosition_3D>();
                    if(castPosition->hasTranslationalComponentBeenSet())
                    {
                        cmd.param5 = static_cast<float>(castPosition->getLatitude());
                        cmd.param6 = static_cast<float>(castPosition->getLongitude());
                    }
                    cmd.param7 = static_cast<float>(castPosition->getAltitude());
                }
            }
            else if(takeoffPosition->getCoordinateSystemType() == CoordinateSystemTypes::CARTESIAN)
            {
                if(takeoffPosition->is3D()) //This is going to be the only type we can handle here
                {
                    const mace::pose::CartesianPosition_3D* castPosition = takeoffPosition->positionAs<mace::pose::CartesianPosition_3D>();
                    if(castPosition->hasTranslationalComponentBeenSet())
                    {
                        //This will not get updated as a mavlink takeoff command is only in the geodetic frame
                    }
                    cmd.param7 = static_cast<float>(castPosition->getAltitude());
                }
            }
        }
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::SpatialTakeoff &data) const
    {
        //Ken: I dont believe this function would ever be called

        data.setTargetSystem(message.target_system);

        mace::pose::GeodeticPosition_3D takeoffPosition;
        if((fabs(message.param5) > std::numeric_limits<double>::epsilon()) && (fabs(message.param6) > std::numeric_limits<double>::epsilon()))
        {
            takeoffPosition.setLatitude(static_cast<double>(message.param5));
            takeoffPosition.setLongitude(static_cast<double>(message.param6));
        }

        if(fabs(message.param7) > std::numeric_limits<double>::epsilon())
        {
            takeoffPosition.setAltitude(static_cast<double>(message.param7));
        }

        data.setPosition(&takeoffPosition);
    }

};

} //end of namespace VehicleController

namespace ModuleController {

class CommandTakeoff : public ModuleController_GenericLongCommand<command_item::SpatialTakeoff, MAV_CMD_NAV_TAKEOFF>
{
public:

    //!
    //! \brief CommandTakeoff::CommandTakeoff This constructor is used to communicate between MACE instances
    //! \param cb
    //! \param queue
    //! \param linkChan
    //!
    CommandTakeoff(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        ModuleController_GenericLongCommand<command_item::SpatialTakeoff, MAV_CMD_NAV_TAKEOFF>(cb, queue, linkChan, "CommandTakeoff")
    {

    }

    virtual ~CommandTakeoff() = default;

    virtual void FillCommand(const command_item::SpatialTakeoff &commandItem, mavlink_command_long_t &cmd) const
    {
        const mace::pose::Position* takeoffPosition = commandItem.getPosition();
        if(takeoffPosition->isAnyPositionValid())
        {
            //The only way the ardupilot can handle this is if the translational component is in the geodetic frame
            if(takeoffPosition->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC)
            {
                if(takeoffPosition->is3D()) //This is going to be the only type we can handle here
                {
                    const mace::pose::GeodeticPosition_3D* castPosition = takeoffPosition->positionAs<mace::pose::GeodeticPosition_3D>();
                    if(castPosition->hasTranslationalComponentBeenSet())
                    {
                        cmd.param5 = static_cast<float>(castPosition->getLatitude());
                        cmd.param6 = static_cast<float>(castPosition->getLongitude());
                    }
                    cmd.param7 = static_cast<float>(castPosition->getAltitude());
                }
            }
            else if(takeoffPosition->getCoordinateSystemType() == CoordinateSystemTypes::CARTESIAN)
            {
                if(takeoffPosition->is3D()) //This is going to be the only type we can handle here
                {
                    const mace::pose::CartesianPosition_3D* castPosition = takeoffPosition->positionAs<mace::pose::CartesianPosition_3D>();
                    if(castPosition->hasTranslationalComponentBeenSet())
                    {
                        //This will not get updated as a mavlink takeoff command is only in the geodetic frame
                    }
                    cmd.param7 = static_cast<float>(castPosition->getAltitude());
                }
            }
        }
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::SpatialTakeoff &data) const
    {
        //Ken: I dont believe this function would ever be called

        data.setTargetSystem(message.target_system);

        mace::pose::GeodeticPosition_3D takeoffPosition;
        if((fabs(message.param5) > std::numeric_limits<double>::epsilon()) && (fabs(message.param6) > std::numeric_limits<double>::epsilon()))
        {
            takeoffPosition.setLatitude(static_cast<double>(message.param5));
            takeoffPosition.setLongitude(static_cast<double>(message.param6));
        }

        if(fabs(message.param7) > std::numeric_limits<double>::epsilon())
        {
            takeoffPosition.setAltitude(static_cast<double>(message.param7));
        }

        data.setPosition(&takeoffPosition);
    }
};

} //end of namespace ModuleController


} //end of namespace MAVLINKUXVControllers

#endif // MAVLINK_CONTROLLER_COMMAND_TAKEOFF_H
