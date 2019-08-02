#include "spatial_takeoff.h"

namespace command_item {

COMMANDTYPE SpatialTakeoff::getCommandType() const
{
    return COMMANDTYPE::CI_NAV_TAKEOFF;
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

void SpatialTakeoff::toMACEComms_CommandItem(mace_command_long_t &obj) const
{
    Interface_CommandItem::initializeCommandItem(obj);
    populateCommandItem_FromPosition(obj);
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
