#include "spatial_rtl.h"

namespace command_item {

COMMANDTYPE SpatialRTL::getCommandType() const
{
    return COMMANDTYPE::CI_NAV_RETURN_TO_LAUNCH;
}

std::string SpatialRTL::getDescription() const
{
    return "This causes the vehicle to return to the launch location";
}

bool SpatialRTL::hasSpatialInfluence() const
{
    return true;
}

std::shared_ptr<AbstractCommandItem> SpatialRTL::getClone() const
{
    return std::make_shared<SpatialRTL>(*this);
}

void SpatialRTL::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<SpatialRTL>(*this);
}

SpatialRTL::SpatialRTL():
    AbstractSpatialAction(0,0)
{

}

SpatialRTL::SpatialRTL(const SpatialRTL &obj):
    AbstractSpatialAction(obj)
{
    this->operator =(obj);
}

SpatialRTL::SpatialRTL(const int &systemOrigin, const int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{

}


void SpatialRTL::toMACEComms_CommandItem(mace_command_short_t &obj) const
{
    Interface_CommandItem::initializeCommandItem(obj);
}

//!
//! \brief printPositionalInfo
//! \return
//!
std::string SpatialRTL::printSpatialCMDInfo() const
{
    std::stringstream ss;
    if(isPositionSet())
        this->position->printPositionLog(ss);
    return ss.str();
}

} //end of namespace CommandItem
