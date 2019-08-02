#include "spatial_loiter_turns.h"

namespace command_item {

COMMANDTYPE SpatialLoiter_Turns::getCommandType() const
{
    return COMMANDTYPE::CI_NAV_LOITER_TURNS;
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

void SpatialLoiter_Turns::toMACEComms_CommandItem(mace_command_long_t &obj) const
{
    Interface_CommandItem::initializeCommandItem(obj);
    populateCommandItem_FromPosition(obj);
    obj.param1 = static_cast<float>(this->turns);
    obj.param3 = this->direction == Data::LoiterDirection::CW ? static_cast<float>(fabs(this->radius)) : static_cast<float>(-1 * fabs(this->radius));
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

