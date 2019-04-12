#include "spatial_takeoff.h"

namespace CommandItem {

COMMANDITEM SpatialTakeoff::getCommandType() const
{
    return COMMANDITEM::CI_NAV_TAKEOFF;
}

std::string SpatialTakeoff::getDescription() const
{
    return "This causes the vehicle to perform a takeoff action";
}

bool SpatialTakeoff::hasSpatialInfluence() const
{
    return true;
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
    this->operator =(obj);
}

SpatialTakeoff::SpatialTakeoff(const int &systemOrigin, const int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{

}

std::ostream& operator<<(std::ostream& os, const SpatialTakeoff& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Spatial Takeoff: " << t.position->getX() << ", "<< t.position->getY() << ", "<< t.position->getZ() << ".";
    os << stream.str();

    return os;
}

}
