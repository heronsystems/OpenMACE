#include "abstract_spatial_action.h"

namespace command_item {

void AbstractSpatialAction::populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const
{
    AbstractCommandItem::populateMACECOMMS_MissionItem(cmd);

    mace_command_long_t currentCommand;
    initializeCommandItem(currentCommand);

    //update the target and origin components

    //populate via calling the virtual function imposed via the interface
    populateCommandItem(currentCommand);

    //populate the remaining mission components
}

void AbstractSpatialAction::fromMACECOMMS_MissionItem(const mace_mission_item_t &obj)
{
    AbstractCommandItem::fromMACECOMMS_MissionItem(obj);

    delete position; position = nullptr;

    switch (static_cast<mace::CoordinateFrameTypes>(obj.frame)) {
    case mace::CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT:
    {
        position = new mace::pose::GeodeticPosition_3D();
        mace::pose::GeodeticPosition_3D* tmpPos = position->positionAs<mace::pose::GeodeticPosition_3D>();
        tmpPos->updatePosition(static_cast<double>(obj.y), static_cast<double>(obj.x),static_cast<double>(obj.z));
        //get the frame and map it to a Geodetic Type
        //tmpPos->setCoordinateFrame();
        break;
    }
    case mace::CoordinateFrameTypes::CF_LOCAL_ENU:
    {
        position = new mace::pose::CartesianPosition_3D();
        mace::pose::CartesianPosition_3D* tmpPos = position->positionAs<mace::pose::CartesianPosition_3D>();
        tmpPos->updatePosition(static_cast<double>(obj.x), static_cast<double>(obj.y),static_cast<double>(obj.z));
        //get the frame and map it to a Cartesian Type
        //tmpPos->setCoordinateFrame();
        break;
    }
    default:
        break;
    }
}

void AbstractSpatialAction::generateMACEMSG_MissionItem(mace_message_t &msg) const
{
    mace_command_long_t currentCommand;
    initializeCommandItem(currentCommand);

    //update the target and origin components

    //populate via calling the virtual function imposed via the interface
    populateCommandItem(currentCommand);

    //transfer the existing command contents to a mission item
    mace_mission_item_t currentMissionItem;
    transferToMissionItem(currentCommand,currentMissionItem);
    mace_msg_mission_item_encode_chan(0,0,0,&msg,&currentMissionItem);
}

void AbstractSpatialAction::generateMACEMSG_CommandItem(mace_message_t &msg) const
{
    mace_command_long_t currentCommand;
    initializeCommandItem(currentCommand);

    //update the target and origin components

    //populate via calling the virtual function imposed via the interface
    populateCommandItem(currentCommand);

    mace_msg_command_long_encode_chan(0,0,0,&msg,&currentCommand);
}


void AbstractSpatialAction::populateCommandItem(mace_command_long_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);

    switch (position->getCoordinateSystemType()) {
    case CoordinateSystemTypes::GEODETIC:
    {
        if(position->is2D())
        {
            mace::pose::GeodeticPosition_2D* tmpPos = position->positionAs<mace::pose::GeodeticPosition_2D>();
            obj.param5 = static_cast<float>(tmpPos->getLongitude()); obj.param6 = static_cast<float>(tmpPos->getLatitude());
        }
        else if(position->is3D())
        {
            mace::pose::GeodeticPosition_3D* tmpPos = position->positionAs<mace::pose::GeodeticPosition_3D>();
            obj.param5 = static_cast<float>(tmpPos->getLongitude()); obj.param6 = static_cast<float>(tmpPos->getLatitude()); obj.param7 = static_cast<float>(tmpPos->getAltitude());
        }
    }
    case CoordinateSystemTypes::CARTESIAN:
    {
        if(position->is2D())
        {
            mace::pose::CartesianPosition_2D* tmpPos = position->positionAs<mace::pose::CartesianPosition_2D>();
            obj.param5 = static_cast<float>(tmpPos->getXPosition()); obj.param6 = static_cast<float>(tmpPos->getYPosition());
        }
        else if(position->is3D())
        {
            mace::pose::CartesianPosition_3D* tmpPos = position->positionAs<mace::pose::CartesianPosition_3D>();
            obj.param5 = static_cast<float>(tmpPos->getXPosition()); obj.param6 = static_cast<float>(tmpPos->getYPosition()); obj.param7 = static_cast<float>(tmpPos->getAltitude());
        }
        break;
    }
    default:
        break;
    }
}

void AbstractSpatialAction::fromCommandItem(const mace_command_long_t &obj)
{
    UNUSED(obj);
    throw std::runtime_error("A request to generate a AbstractSpatialAction from a command long is not valid. Spatial items are never just a command.");
}


void AbstractSpatialAction::populateMACEComms_GoToCommand(mace_command_goto_t &obj) const
{
    mace_command_long_t currentCommand;
    populateCommandItem(currentCommand);

    transferTo_GoToCommand(currentCommand,obj);
}

void AbstractSpatialAction::fromMACECOMMS_GoToCommand(const mace_command_goto_t &obj)
{
    this->targetSystem = obj.target_system;
    this->targetComponent = obj.target_component;
    //from here we need to populate the position object
    //populatePositionObject()
}

void AbstractSpatialAction::transferTo_GoToCommand(const mace_command_long_t &cmd, mace_command_goto_t &obj) const
{
    obj.target_system = cmd.target_system;
    obj.target_component = cmd.target_component;

    obj.param1 = cmd.param1;
    obj.param2 = cmd.param2;
    obj.param3 = cmd.param3;
    obj.param4 = cmd.param4;
    obj.param5 = cmd.param5;
    obj.param6 = cmd.param6;
    obj.param7 = cmd.param7;
}

void AbstractSpatialAction::populatePositionObject(const CoordinateSystemTypes &generalType, const mace::CoordinateFrameTypes &explicitType, const uint8_t &dim,
                                const double &x, const double &y, const double &z)
{
    delete position; position = nullptr;

    switch (generalType) {
    case CoordinateSystemTypes::GEODETIC:
    {
        if(dim == 2)
        {
            position = new mace::pose::GeodeticPosition_2D();
            mace::pose::GeodeticPosition_2D* tmpPos = position->positionAs<mace::pose::GeodeticPosition_2D>();
            //tmpPos->setCoordinateFrame();
            tmpPos->updateTranslationalComponents(y,x);
        }
        else if(dim == 3)
        {
            position = new mace::pose::GeodeticPosition_3D();
            mace::pose::GeodeticPosition_3D* tmpPos = position->positionAs<mace::pose::GeodeticPosition_3D>();
            //tmpPos->setCoordinateFrame();
            tmpPos->updatePosition(y,x,z);
        }
    }
    case CoordinateSystemTypes::CARTESIAN:
    {
        if(dim == 2)
        {
            position = new mace::pose::CartesianPosition_2D();
            mace::pose::CartesianPosition_2D* tmpPos = position->positionAs<mace::pose::CartesianPosition_2D>();
            //tmpPos->setCoordinateFrame();
            tmpPos->updatePosition(x,y);
        }
        else if(dim == 3)
        {
            position = new mace::pose::CartesianPosition_3D();
            mace::pose::CartesianPosition_3D* tmpPos = position->positionAs<mace::pose::CartesianPosition_3D>();
            //tmpPos->setCoordinateFrame();
            tmpPos->updatePosition(x,y,z);
        }
        break;
    }
    default:
        break;
    }
}

} //end of namespace CommandItem
