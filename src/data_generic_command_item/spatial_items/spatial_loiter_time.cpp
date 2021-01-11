#include "spatial_loiter_time.h"

namespace command_item {

MAV_CMD SpatialLoiter_Time::getCommandType() const
{
    return MAV_CMD::MAV_CMD_NAV_LOITER_TIME;
}

std::string SpatialLoiter_Time::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION for X seconds";
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
    this->radius = obj.radius;
    this->direction = obj.direction;
    this->duration = obj.duration;
}

SpatialLoiter_Time::SpatialLoiter_Time(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{

}

/** Interface imposed via AbstractSpatialAction */

void SpatialLoiter_Time::populateCommandItem(mavlink_command_long_t &obj) const
{
    AbstractSpatialAction::populateCommandItem(obj);
    obj.param1 = static_cast<float>(this->duration);
    obj.param3 = this->direction == Data::LoiterDirection::CW ? static_cast<float>(fabs(this->radius)) : static_cast<float>(-1 * fabs(this->radius));
}

void SpatialLoiter_Time::fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &obj)
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
std::string SpatialLoiter_Time::printSpatialCMDInfo() const
{
    std::stringstream ss;
    if(isPositionSet())
        this->position->printPositionLog(ss);
    return ss.str();
}

} //end of namespace CommandItem
