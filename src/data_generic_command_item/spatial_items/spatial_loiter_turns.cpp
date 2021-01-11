#include "spatial_loiter_turns.h"

namespace command_item {

MAV_CMD SpatialLoiter_Turns::getCommandType() const
{
    return MAV_CMD::MAV_CMD_NAV_LOITER_TURNS;
}

std::string SpatialLoiter_Turns::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION for X turns";
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
    this->radius = obj.radius;
    this->direction = obj.direction;
    this->turns = obj.turns;
}

SpatialLoiter_Turns::SpatialLoiter_Turns(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{

}

/** Interface imposed via AbstractSpatialAction */

void SpatialLoiter_Turns::populateCommandItem(mavlink_command_long_t &obj) const
{
    AbstractSpatialAction::populateCommandItem(obj);
    obj.param1 = static_cast<float>(this->turns);
    obj.param3 = this->direction == Data::LoiterDirection::CW ? static_cast<float>(fabs(this->radius)) : static_cast<float>(-1 * fabs(this->radius));
}

void SpatialLoiter_Turns::fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &obj)
{
    AbstractSpatialAction::fromMACECOMMS_MissionItem(obj);
    this->radius = fabs(static_cast<double>(obj.param3));
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
std::string SpatialLoiter_Turns::printSpatialCMDInfo() const
{
    std::stringstream ss;
    if(isPositionSet())
        this->position->printPositionLog(ss);
    return ss.str();
}

} //end of namespace CommandItem

