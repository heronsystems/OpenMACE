#include "spatial_loiter_time.h"

namespace command_item {

COMMANDTYPE SpatialLoiter_Time::getCommandType() const
{
    return COMMANDTYPE::CI_NAV_LOITER_TIME;
}

std::string SpatialLoiter_Time::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION for X seconds";
}

bool SpatialLoiter_Time::hasSpatialInfluence() const
{
    return true;
}

std::shared_ptr<AbstractCommandItem> SpatialLoiter_Time::getClone() const
{
    return std::make_shared<SpatialLoiter_Time>(*this);
}

void SpatialLoiter_Time::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<SpatialLoiter_Time>(*this);
}

SpatialLoiter_Time::SpatialLoiter_Time():
    AbstractSpatialAction(0,0)
{

}

SpatialLoiter_Time::SpatialLoiter_Time(const SpatialLoiter_Time &obj):
    AbstractSpatialAction(0,0)
{
    this->operator =(obj);
}

SpatialLoiter_Time::SpatialLoiter_Time(const int &systemOrigin, const int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{

}

void SpatialLoiter_Time::toMACEComms_CommandItem(mace_command_long_t &obj) const
{
    Interface_CommandItem::initializeCommandItem(obj);
    populateCommandItem_FromPosition(obj);
    obj.param1 = static_cast<float>(this->duration);
    obj.param3 = this->direction == Data::LoiterDirection::CW ? static_cast<float>(fabs(this->radius)) : static_cast<float>(-1 * fabs(this->radius));
}


//!
//! \brief printPositionalInfo
//! \return
//!
std::string SpatialLoiter_Time::printSpatialCMDInfo() const
{
    std::stringstream ss;
    if(isPositionSet())
        this->position->printPositionLog(ss);
    return ss.str();
}

} //end of namespace CommandItem
