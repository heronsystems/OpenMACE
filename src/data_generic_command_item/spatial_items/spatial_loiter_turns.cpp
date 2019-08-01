#include "spatial_loiter_turns.h"

namespace CommandItem {

COMMANDITEM SpatialLoiter_Turns::getCommandType() const
{
    return COMMANDITEM::CI_NAV_LOITER_TURNS;
}

std::string SpatialLoiter_Turns::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION for X turns";
}

bool SpatialLoiter_Turns::hasSpatialInfluence() const
{
    return true;
}

std::shared_ptr<AbstractCommandItem> SpatialLoiter_Turns::getClone() const
{
    return std::make_shared<SpatialLoiter_Turns>(*this);
}

void SpatialLoiter_Turns::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<SpatialLoiter_Turns>(*this);
}

SpatialLoiter_Turns::SpatialLoiter_Turns():
    AbstractSpatialAction(0,0)
{

}


SpatialLoiter_Turns::SpatialLoiter_Turns(const SpatialLoiter_Turns &obj):
    AbstractSpatialAction(obj)
{
    this->operator =(obj);
}

SpatialLoiter_Turns::SpatialLoiter_Turns(const int &systemOrigin, const int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{

}

//!
//! \brief printPositionalInfo
//! \return
//!
std::string SpatialLoiter_Turns::printSpatialCMDInfo() const
{
    std::stringstream ss;
    if(isPositionSet())
        this->position->printPositionLog(ss);
    return ss.str();
}

} //end of namespace CommandItem

