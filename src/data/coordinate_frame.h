#ifndef COORDINATE_FRAMEOLD_H
#define COORDINATE_FRAMEOLD_H

#include <string>
#include <stdexcept>

namespace Data
{
enum class CoordinateFrameType : uint8_t{
    CF_GLOBAL=0, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | */
    CF_LOCAL_NED=1, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */
    CF_MISSION=2, /* NOT a coordinate frame, indicates a mission command. | */
    CF_GLOBAL_RELATIVE_ALT=3, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */
    CF_LOCAL_ENU=4, /* Local coordinate frame, Z-down (x: east, y: north, z: up) | */
    CF_GLOBAL_INT=5, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL) | */
    CF_GLOBAL_RELATIVE_ALT_INT=6, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location. | */
    CF_LOCAL_OFFSET_NED=7, /* Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | */
    CF_BODY_NED=8, /* Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | */
    CF_BODY_OFFSET_NED=9, /* Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | */
    CF_GLOBAL_TERRAIN_ALT=10, /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
    CF_GLOBAL_TERRAIN_ALT_INT=11, /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
    CF_ENUM_END=12, /*  | */
};

inline std::string CoordinateFrameToString(const CoordinateFrameType &frame) {
    switch (frame) {
    case CoordinateFrameType::CF_GLOBAL:
        return "CF_GLOBAL";
    case CoordinateFrameType::CF_LOCAL_NED:
        return "CF_LOCAL_NED";
    case CoordinateFrameType::CF_MISSION:
        return "CF_MISSION";
    case CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT:
        return "CF_GLOBAL_RELATIVE_ALT";
    case CoordinateFrameType::CF_LOCAL_ENU:
        return "CF_LOCAL_ENU";
    case CoordinateFrameType::CF_GLOBAL_INT:
        return "CF_GLOBAL_INT";
    case CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT_INT:
        return "CF_GLOBAL_RELATIVE_ALT_INT";
    case CoordinateFrameType::CF_LOCAL_OFFSET_NED:
        return "CF_LOCAL_OFFSET_NED";
    case CoordinateFrameType::CF_BODY_NED:
        return "CF_BODY_NED";
    case CoordinateFrameType::CF_BODY_OFFSET_NED:
        return "CF_BODY_OFFSET_NED";
    case CoordinateFrameType::CF_GLOBAL_TERRAIN_ALT:
        return "CF_GLOBAL_TERRAIN_ALT";
    case CoordinateFrameType::CF_GLOBAL_TERRAIN_ALT_INT:
        return "CF_GLOBAL_TERRAIN_ALT_INT";
    default:
        throw std::runtime_error("Unknown coordinate system seen");
    }
}

inline CoordinateFrameType CoordinateFrameFromString(const std::string &str) {
    if(str == "CF_GLOBAL")
        return CoordinateFrameType::CF_GLOBAL;
    if(str == "CF_LOCAL_NED")
        return CoordinateFrameType::CF_LOCAL_NED;
    if(str == "CF_MISSION")
        return CoordinateFrameType::CF_MISSION;
    if(str == "CF_GLOBAL_RELATIVE_ALT")
        return CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
    if(str == "CF_LOCAL_ENU")
        return CoordinateFrameType::CF_LOCAL_ENU;
    if(str == "CF_GLOBAL_INT")
        return CoordinateFrameType::CF_GLOBAL_INT;
    if(str == "CF_GLOBAL_RELATIVE_ALT_INT")
        return CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT_INT;
    if(str == "CF_LOCAL_OFFSET_NED")
        return CoordinateFrameType::CF_LOCAL_OFFSET_NED;
    if(str == "CF_BODY_NED")
        return CoordinateFrameType::CF_BODY_NED;
    if(str == "CF_BODY_OFFSET_NED")
        return CoordinateFrameType::CF_BODY_OFFSET_NED;
    if(str == "CF_GLOBAL_TERRAIN_ALT")
        return CoordinateFrameType::CF_GLOBAL_TERRAIN_ALT;
    if(str == "CF_GLOBAL_TERRAIN_ALT_INT")
        return CoordinateFrameType::CF_GLOBAL_TERRAIN_ALT_INT;
    throw std::runtime_error("Unknown coordinate system seen");
}

} //end of namespace Data

#endif // COORDINATE_FRAME_H
