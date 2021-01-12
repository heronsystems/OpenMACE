#include "abstract_spatial_action.h"

namespace command_item {

void AbstractSpatialAction::populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const
{
    AbstractCommandItem::populateMACECOMMS_MissionItem(cmd);

    mavlink_command_long_t currentCommand;
    initializeCommandItem(currentCommand);

    //update the target and origin components

    //populate via calling the virtual function imposed via the interface
    populateCommandItem(currentCommand);

    //tranfer the contents from the command to the mission item
    transferToMissionItem(currentCommand, cmd);

    //populate the remaining mission components

}

void AbstractSpatialAction::fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &obj)
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

void AbstractSpatialAction::generateMACEMSG_MissionItem(mavlink_message_t &msg) const
{
    mavlink_command_long_t currentCommand;
    initializeCommandItem(currentCommand);

    //update the target and origin components

    //populate via calling the virtual function imposed via the interface
    populateCommandItem(currentCommand);

    //transfer the existing command contents to a mission item
    mavlink_mace_mission_item_int_t currentMissionItem;
    transferToMissionItem(currentCommand,currentMissionItem);
    mavlink_msg_mace_mission_item_int_encode_chan(0,0,0,&msg,&currentMissionItem);
}

void AbstractSpatialAction::generateMACEMSG_CommandItem(mavlink_message_t &msg) const
{
    mavlink_command_long_t currentCommand;
    initializeCommandItem(currentCommand);

    //update the target and origin components

    //populate via calling the virtual function imposed via the interface
    populateCommandItem(currentCommand);

    mavlink_msg_command_long_encode_chan(0,0,0,&msg,&currentCommand);
}


void AbstractSpatialAction::populateCommandItem(mavlink_command_long_t &obj) const
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
        break;
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

void AbstractSpatialAction::fromCommandItem(const mavlink_command_long_t &obj)
{
    UNUSED(obj);
    throw std::runtime_error("A request to generate a AbstractSpatialAction from a command long is not valid. Spatial items are never just a command.");
}

void AbstractSpatialAction::populatePositionObject(const mace::CoordinateFrameTypes &explicitFrame, const uint8_t &dim, const uint16_t &mask,
                                const double &x, const double &y, const double &z)
{
    if(position != nullptr) {
        delete position;
        position = nullptr;
    }

    CoordinateSystemTypes currentSystemType = mace::getCoordinateSystemType(explicitFrame);

    switch (currentSystemType) {
    case CoordinateSystemTypes::GEODETIC:
    {

        mace::GeodeticFrameTypes geodeticFrame = mace::getGeodeticCoordinateFrame(explicitFrame);
        if(dim == 2)
        {
            position = new mace::pose::GeodeticPosition_2D();
            mace::pose::GeodeticPosition_2D* tmpPos = position->positionAs<mace::pose::GeodeticPosition_2D>();
            tmpPos->setCoordinateFrame(geodeticFrame);
            tmpPos->updateTranslationalComponents(y,x);
            tmpPos->setDimensionMask(mask);
        }
        else if(dim == 3)
        {
            position = new mace::pose::GeodeticPosition_3D();
            mace::pose::GeodeticPosition_3D* tmpPos = position->positionAs<mace::pose::GeodeticPosition_3D>();
            tmpPos->setCoordinateFrame(geodeticFrame);
            tmpPos->updatePosition(y,x,z);
            tmpPos->setDimensionMask(mask);
        }
        break;
    }
    case CoordinateSystemTypes::CARTESIAN:
    {
        mace::CartesianFrameTypes cartesianFrame = mace::getCartesianCoordinateFrame(explicitFrame);
        if(dim == 2)
        {
            position = new mace::pose::CartesianPosition_2D();
            mace::pose::CartesianPosition_2D* tmpPos = position->positionAs<mace::pose::CartesianPosition_2D>();
            tmpPos->setCoordinateFrame(cartesianFrame);
            tmpPos->updatePosition(x,y);
            tmpPos->setDimensionMask(mask);
        }
        else if(dim == 3)
        {
            position = new mace::pose::CartesianPosition_3D();
            mace::pose::CartesianPosition_3D* tmpPos = position->positionAs<mace::pose::CartesianPosition_3D>();
            tmpPos->setCoordinateFrame(cartesianFrame);
            tmpPos->updatePosition(x,y,z);
            tmpPos->setDimensionMask(mask);
        }
        break;
    }
    default:
        break;
    }
}

} //end of namespace CommandItem
