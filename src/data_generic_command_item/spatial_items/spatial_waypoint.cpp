#include "spatial_waypoint.h"

namespace command_item {

COMMANDTYPE SpatialWaypoint::getCommandType() const
{
    return COMMANDTYPE::CI_NAV_WAYPOINT;
}

std::string SpatialWaypoint::getDescription() const
{
    return "This is a waypoint mission item for a vehicle";
}

bool SpatialWaypoint::hasSpatialInfluence() const
{
    return true;
}

std::shared_ptr<AbstractCommandItem> SpatialWaypoint::getClone() const
{
    return std::make_shared<SpatialWaypoint>(*this);
}

void SpatialWaypoint::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<SpatialWaypoint>(*this);
}

SpatialWaypoint::SpatialWaypoint():
    AbstractSpatialAction(0,0)
{

}

SpatialWaypoint::SpatialWaypoint(const SpatialWaypoint &obj):
    AbstractSpatialAction(obj)
{
    this->operator =(obj);
}

SpatialWaypoint::SpatialWaypoint(const int &systemOrigin, const int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{

}

SpatialWaypoint::~SpatialWaypoint()
{

}

void SpatialWaypoint::toMACEComms_CommandItem(mace_command_long_t &obj) const
{
    Interface_CommandItem::initializeCommandItem(obj);
    populateCommandItem_FromPosition(obj);
}


mace_command_goto_t SpatialWaypoint::setGoToCommand(mace_command_goto_t &cmd) const
{
//    cmd.action = (uint16_t)COMMANDITEM::CI_NAV_WAYPOINT;
//    cmd.frame = (uint8_t)this->getPosition().getCoordinateFrame();
//    cmd.param5 = this->getPosition().getX();
//    cmd.param6 = this->getPosition().getY();
//    cmd.param7 = this->getPosition().getZ();
}

void SpatialWaypoint::updateFromGoToCommand(const mace_command_goto_t &cmd)
{
    //Ken Fix: There has to be a better way to produce target systems. The actions should not have knowledge of this, should be taken care of in the command
//    this->setTargetSystem(cmd.target_system);
//    this->position->setCoordinateFrame((Data::CoordinateFrameType)cmd.frame);
//    this->position->setX(cmd.param5);
//    this->position->setY(cmd.param6);
//    this->position->setZ(cmd.param7);
}

//!
//! \brief printPositionalInfo
//! \return
//!
std::string SpatialWaypoint::printSpatialCMDInfo() const
{
    std::stringstream ss;
    if(isPositionSet())
        this->position->printPositionLog(ss);
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const SpatialWaypoint& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Spatial Waypoint: " << t.printSpatialCMDInfo();
    os << stream.str();

    return os;
}

} //end of namepsace CommandItem
