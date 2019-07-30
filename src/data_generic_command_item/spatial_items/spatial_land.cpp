#include "spatial_land.h"

namespace CommandItem {

COMMANDITEM SpatialLand::getCommandType() const
{
    return COMMANDITEM::CI_NAV_LAND;
}

std::string SpatialLand::getDescription() const
{
    return "This causes the vehicle to land either at the current location or prescribed location";
}

bool SpatialLand::hasSpatialInfluence() const
{
    return true;
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

SpatialLand::SpatialLand(const int &systemOrigin,  const int &systemTarget):
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
    std::stringstream stream;
    stream.precision(6);
    //stream << std::fixed << "Spatial Land: " << t.position->getX() << ", "<< t.position->getY() << ", "<< t.position->getZ() << ".";
    os << stream.str();

    return os;
}

} //end of namespace CommandItem
