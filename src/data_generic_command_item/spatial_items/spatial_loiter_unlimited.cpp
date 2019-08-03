#include "spatial_loiter_unlimited.h"

namespace command_item {

COMMANDTYPE SpatialLoiter_Unlimited::getCommandType() const
{
    return COMMANDTYPE::CI_NAV_LOITER_UNLIM;
}

std::string SpatialLoiter_Unlimited::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION an unlimited amount of time";
}

std::shared_ptr<AbstractCommandItem> SpatialLoiter_Unlimited::getClone() const
{
    return std::make_shared<SpatialLoiter_Unlimited>(*this);
}

void SpatialLoiter_Unlimited::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<SpatialLoiter_Unlimited>(*this);
}

SpatialLoiter_Unlimited::SpatialLoiter_Unlimited():
    AbstractSpatialAction(0,0)
{

}

SpatialLoiter_Unlimited::SpatialLoiter_Unlimited(const SpatialLoiter_Unlimited &obj):
    AbstractSpatialAction(obj)
{
    this->operator =(obj);
}

SpatialLoiter_Unlimited::SpatialLoiter_Unlimited(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{

}

/** Interface imposed via AbstractSpatialAction */

void SpatialLoiter_Unlimited::populateCommandItem(mace_command_long_t &obj) const
{
    AbstractSpatialAction::populateCommandItem(obj);
    obj.param3 = this->direction == Data::LoiterDirection::CW ? static_cast<float>(fabs(this->radius)) : static_cast<float>(-1 * fabs(this->radius));
}

void SpatialLoiter_Unlimited::fromMACECOMMS_MissionItem(const mace_mission_item_t &obj)
{
    AbstractSpatialAction::fromMACECOMMS_MissionItem(obj);
    this->radius = fabs(obj.param3);
    if(obj.param3 < 0)
        this->direction = Data::LoiterDirection::CCW;
    else
        this->direction = Data::LoiterDirection::CW;
}

void SpatialLoiter_Unlimited::fromMACECOMMS_GoToCommand(const mace_command_goto_t &obj)
{
    AbstractSpatialAction::fromMACECOMMS_GoToCommand(obj);
    this->radius = fabs(obj.param3);
    if(obj.param3 < 0)
        this->direction = Data::LoiterDirection::CCW;
    else
        this->direction = Data::LoiterDirection::CW;
}

/** End of interface imposed via AbstractSpatialAction */

//!
//! \brief printPositionalInfo
//! \return
//!
std::string SpatialLoiter_Unlimited::printSpatialCMDInfo() const
{
    std::stringstream ss;
    if(isPositionSet())
        this->position->printPositionLog(ss);
    return ss.str();
}

} //end of namespace CommandItem

