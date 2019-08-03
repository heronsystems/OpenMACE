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
