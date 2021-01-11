#include "spatial_takeoff.h"

namespace command_item {

MAV_CMD SpatialTakeoff::getCommandType() const
{
    return MAV_CMD::MAV_CMD_NAV_TAKEOFF;
}

std::string SpatialTakeoff::getDescription() const
{
    return "This causes the vehicle to perform a takeoff action";
}

std::shared_ptr<AbstractCommandItem> SpatialTakeoff::getClone() const
{
    return std::make_shared<SpatialTakeoff>(*this);
}

void SpatialTakeoff::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<SpatialTakeoff>(*this);
}

SpatialTakeoff::SpatialTakeoff():
    AbstractSpatialAction(0,0)

{

}

SpatialTakeoff::SpatialTakeoff(const SpatialTakeoff &obj):
    AbstractSpatialAction(obj)
{
}

SpatialTakeoff::SpatialTakeoff(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{
}

//!
//! \brief printPositionalInfo
//! \return
//!
std::string SpatialTakeoff::printSpatialCMDInfo() const
{
    std::stringstream ss;
    if(isPositionSet())
        this->position->printPositionLog(ss);
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const SpatialTakeoff& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Spatial Takeoff: " <<t.printSpatialCMDInfo()<<".";
    os << stream.str();

    return os;
}

}
