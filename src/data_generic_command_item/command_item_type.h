#ifndef COMMAND_ITEM_TYPE_H
#define COMMAND_ITEM_TYPE_H

#include <mavlink.h>
#include <string>
#include <stdexcept>

namespace command_item
{

inline std::string CommandItemToString(const MAV_CMD &commandItemType) {
    switch (commandItemType) {
    case MAV_CMD::MAV_CMD_NAV_LAND:
        return "CI_NAV_LAND";
    case MAV_CMD::MAV_CMD_NAV_LOITER_TIME:
        return "CI_NAV_LOITER_TIME";
    case MAV_CMD::MAV_CMD_NAV_LOITER_TURNS:
        return "CI_NAV_LOITER_TURNS";
    case MAV_CMD::MAV_CMD_NAV_LOITER_UNLIM:
        return "CI_NAV_LOITER_UNLIM";
    case MAV_CMD::MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return "CI_NAV_RETURN_TO_LAUNCH";
    case MAV_CMD::MAV_CMD_NAV_TAKEOFF:
        return "CI_NAV_TAKEOFF";
    case MAV_CMD::MAV_CMD_NAV_WAYPOINT:
        return "CI_NAV_WAYPOINT";
    case MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM:
        return "CI_ACT_ARM";
    case MAV_CMD::MAV_CMD_DO_CHANGE_SPEED:
        return "CI_ACT_CHANGESPEED";
    case MAV_CMD::MAV_CMD_DO_SET_MODE:
        return "CI_ACT_CHANGEMODE";
    case MAV_CMD::MAV_CMD_DO_MOTOR_TEST:
        return "CI_ACT_MOTORTEST";
    case MAV_CMD::MAV_CMD_USER_1:
        return "MAV_CMD_USER_1";
    case MAV_CMD::MAV_CMD_DO_PAUSE_CONTINUE:
        return "CI_ACT_MISSIONPROCEDURAL";
    default:
        throw std::runtime_error("Unknown mission item enum seen");
    }
}

inline MAV_CMD CommandItemFromString(const std::string &str) {
    if(str == "CI_NAV_LAND")
        return MAV_CMD::MAV_CMD_NAV_LAND;
    if(str == "CI_NAV_LOITER_TIME")
        return MAV_CMD::MAV_CMD_NAV_LOITER_TIME;
    if(str == "CI_NAV_LOITER_TURNS")
        return MAV_CMD::MAV_CMD_NAV_LOITER_TURNS;
    if(str == "CI_NAV_LOITER_UNLIM")
        return MAV_CMD::MAV_CMD_NAV_LOITER_UNLIM;
    if(str == "CI_NAV_RETURN_TO_LAUNCH")
        return MAV_CMD::MAV_CMD_NAV_RETURN_TO_LAUNCH;
    if(str == "CI_NAV_TAKEOFF")
        return MAV_CMD::MAV_CMD_NAV_TAKEOFF;
    if(str == "CI_NAV_WAYPOINT")
        return MAV_CMD::MAV_CMD_NAV_WAYPOINT;
    if(str == "CI_ACT_ARM")
        return MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM;
    if(str == "CI_ACT_CHANGESPEED")
        return MAV_CMD::MAV_CMD_DO_CHANGE_SPEED;
    if(str == "CI_ACT_CHANGEMODE")
        return MAV_CMD::MAV_CMD_DO_SET_MODE;
    if(str == "CI_ACT_MOTORTEST")
        return MAV_CMD::MAV_CMD_DO_MOTOR_TEST;
    if(str == "CI_ACT_MISSIONPROCEDURAL")
        return MAV_CMD::MAV_CMD_DO_PAUSE_CONTINUE;
    if(str == "MAV_CMD_USER_1")
        return MAV_CMD::MAV_CMD_USER_1;
    throw std::runtime_error("Unknown mission item string seen");
}

} //end of namespace Data

#endif // COMMAND_ITEM_TYPE_H
