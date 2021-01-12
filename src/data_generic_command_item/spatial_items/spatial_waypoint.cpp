#include "spatial_waypoint.h"

namespace command_item {

MAV_CMD SpatialWaypoint::getCommandType() const
{
    return MAV_CMD::MAV_CMD_NAV_WAYPOINT;
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

}

SpatialWaypoint::SpatialWaypoint(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{

}

SpatialWaypoint::~SpatialWaypoint()
{

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
