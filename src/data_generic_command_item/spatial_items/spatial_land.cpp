#include "spatial_land.h"

namespace command_item {

MAV_CMD SpatialLand::getCommandType() const
{
    return MAV_CMD::MAV_CMD_NAV_LAND;
}

std::string SpatialLand::getDescription() const
{
    return "This causes the vehicle to land either at the current location or prescribed location";
}

std::shared_ptr<AbstractCommandItem> SpatialLand::getClone() const
{
    return std::make_shared<SpatialLand>(*this);
}

void SpatialLand::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<SpatialLand>(*this);
}

SpatialLand::SpatialLand():
    AbstractSpatialAction(0,0)
{

}

SpatialLand::SpatialLand(const SpatialLand &obj):
    AbstractSpatialAction(obj)
{
    this->operator =(obj);
}

SpatialLand::SpatialLand(const unsigned int &systemOrigin,  const unsigned int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{

}

//!
//! \brief printPositionalInfo
//! \return
//!
std::string SpatialLand::printSpatialCMDInfo() const
{
    std::stringstream ss;
    if(isPositionSet())
        this->position->printPositionLog(ss);
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const SpatialLand& t)
{
    UNUSED(t);
    std::stringstream stream;
    stream.precision(6);
    //stream << std::fixed << "Spatial Land: " << t.position->getX() << ", "<< t.position->getY() << ", "<< t.position->getZ() << ".";
    os << stream.str();

    return os;
}

} //end of namespace CommandItem
