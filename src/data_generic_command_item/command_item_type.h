#ifndef COMMAND_ITEM_TYPE_H
#define COMMAND_ITEM_TYPE_H

#include <string>
#include <stdexcept>

namespace command_item
{

enum class COMMANDLENGTH : uint8_t
{
    CMD_LONG,
    CMD_SHORT
};

enum class COMMANDTYPE : uint8_t{
    CI_NAV_HOME = 0,
    CI_NAV_LAND=1, /* Land at location |Abort Alt| Empty| Empty| Desired yaw angle. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    CI_NAV_LOITER_TIME=2, /* Loiter around this CISSION for X seconds |Seconds (decimal)| Empty| Radius around CISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    CI_NAV_LOITER_TURNS=3, /* Loiter around this CISSION for X turns |Turns| Empty| Radius around CISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    CI_NAV_LOITER_UNLIM=4, /* Loiter around this CISSION an unliCIted amount of time |Empty| Empty| Radius around CISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
    CI_NAV_RETURN_TO_LAUNCH=5, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CI_NAV_TAKEOFF=6, /* Takeoff from ground / hand |CInimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    CI_NAV_WAYPOINT=7, /* Navigate to CISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at CISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the CISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at CISSION (rotary wing). NaN for unchanged.| Latitude| Longitude| Altitude|  */
    CI_ACT_ARM = 8,
    CI_ACT_CHANGEMODE = 9,
    CI_ACT_CHANGESPEED = 10,
    CI_ACT_EXECUTE_SPATIAL_ITEM = 11,
    CI_ACT_MISSIONCMD = 12,
    CI_ACT_MOTORTEST = 13,
    CI_ACT_TARGET = 14,
    CI_ACT_MSG_INTERVAL = 15,
    CI_ACT_MSG_REQUEST = 16,
    CI_ACT_SET_GLOBAL_ORIGIN = 17,
    CI_ACT_HOME_POSITION = 18,
    CI_UNKNOWN = 19,
    COMMANDITEMEND = 20
};

inline std::string CommandItemToString(const COMMANDTYPE &commandItemType) {
    switch (commandItemType) {
    case COMMANDTYPE::CI_NAV_LAND:
        return "CI_NAV_LAND";
    case COMMANDTYPE::CI_NAV_LOITER_TIME:
        return "CI_NAV_LOITER_TIME";
    case COMMANDTYPE::CI_NAV_LOITER_TURNS:
        return "CI_NAV_LOITER_TURNS";
    case COMMANDTYPE::CI_NAV_LOITER_UNLIM:
        return "CI_NAV_LOITER_UNLIM";
    case COMMANDTYPE::CI_NAV_RETURN_TO_LAUNCH:
        return "CI_NAV_RETURN_TO_LAUNCH";
    case COMMANDTYPE::CI_NAV_TAKEOFF:
        return "CI_NAV_TAKEOFF";
    case COMMANDTYPE::CI_NAV_WAYPOINT:
        return "CI_NAV_WAYPOINT";
    case COMMANDTYPE::CI_ACT_ARM:
        return "CI_ACT_ARM";
    case COMMANDTYPE::CI_ACT_CHANGESPEED:
        return "CI_ACT_CHANGESPEED";
    case COMMANDTYPE::CI_ACT_CHANGEMODE:
        return "CI_ACT_CHANGEMODE";
    case COMMANDTYPE::CI_ACT_MOTORTEST:
        return "CI_ACT_MOTORTEST";
    case COMMANDTYPE::CI_ACT_EXECUTE_SPATIAL_ITEM:
        return "CI_ACT_EXECUTE_SPATIAL_ITEM";
    case COMMANDTYPE::CI_ACT_MISSIONCMD:
        return "CI_ACT_MISSIONCMD";
    case COMMANDTYPE::CI_UNKNOWN:
        return "CI_UNKNOWN";
    default:
        throw std::runtime_error("Unknown mission item enum seen");
    }
}

inline COMMANDTYPE CommandItemFromString(const std::string &str) {
    if(str == "CI_NAV_LAND")
        return COMMANDTYPE::CI_NAV_LAND;
    if(str == "CI_NAV_LOITER_TIME")
        return COMMANDTYPE::CI_NAV_LOITER_TIME;
    if(str == "CI_NAV_LOITER_TURNS")
        return COMMANDTYPE::CI_NAV_LOITER_TURNS;
    if(str == "CI_NAV_LOITER_UNLIM")
        return COMMANDTYPE::CI_NAV_LOITER_UNLIM;
    if(str == "CI_NAV_RETURN_TO_LAUNCH")
        return COMMANDTYPE::CI_NAV_RETURN_TO_LAUNCH;
    if(str == "CI_NAV_TAKEOFF")
        return COMMANDTYPE::CI_NAV_TAKEOFF;
    if(str == "CI_NAV_WAYPOINT")
        return COMMANDTYPE::CI_NAV_WAYPOINT;
    if(str == "CI_ACT_ARM")
        return COMMANDTYPE::CI_ACT_ARM;
    if(str == "CI_ACT_CHANGESPEED")
        return COMMANDTYPE::CI_ACT_CHANGESPEED;
    if(str == "CI_ACT_CHANGEMODE")
        return COMMANDTYPE::CI_ACT_CHANGEMODE;
    if(str == "CI_ACT_MOTORTEST")
        return COMMANDTYPE::CI_ACT_MOTORTEST;
    if(str == "CI_ACT_MISSIONCMD")
        return COMMANDTYPE::CI_ACT_MISSIONCMD;
    if(str == "CI_ACT_EXECUTE_SPATIAL_ITEM")
        return COMMANDTYPE::CI_ACT_EXECUTE_SPATIAL_ITEM;
    if(str == "CI_UNKNOWN")
        return COMMANDTYPE::CI_UNKNOWN;
    throw std::runtime_error("Unknown mission item string seen");
}

} //end of namespace Data

#endif // COMMAND_ITEM_TYPE_H
