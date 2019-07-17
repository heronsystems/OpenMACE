#ifndef COORDINATE_FRAME_H
#define COORDINATE_FRAME_H

#include <string>
#include <stdexcept>

namespace mace {
namespace pose {

#define LOCAL_FRAMES CF_LOCAL_UNKNOWN, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */ \
CF_LOCAL_NED, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */ \
CF_LOCAL_ENU, /* Local coordinate frame, Z-down (x: east, y: north, z: up) | */ \
CF_LOCAL_OFFSET_NED, /* Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | */ \
CF_BODY_NED, /* Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | */ \
CF_BODY_OFFSET_NED /* Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | */

#define GLOBAL_FRAMES CF_GLOBAL_UNKNOWN, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | */ \
CF_GLOBAL_RELATIVE_ALT, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */ \
CF_GLOBAL_INT, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL) | */ \
CF_GLOBAL_RELATIVE_ALT_INT, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location. | */ \
CF_GLOBAL_TERRAIN_ALT, /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */ \
CF_GLOBAL_TERRAIN_ALT_INT /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */


enum class CartesianFrameTypes: uint8_t{
    LOCAL_FRAMES
};

enum class GeodeticFrameTypes : uint8_t{
    GLOBAL_FRAMES
};

enum class CoordinateFrame : uint8_t{
    GLOBAL_FRAMES,
    LOCAL_FRAMES,
    CF_UNKNOWN
//    CF_GLOBAL=0, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | */
//    CF_LOCAL_NED=1, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */
//    CF_GLOBAL_RELATIVE_ALT=2, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */
//    CF_LOCAL_ENU=3, /* Local coordinate frame, Z-down (x: east, y: north, z: up) | */
//    CF_GLOBAL_INT=4, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL) | */
//    CF_GLOBAL_RELATIVE_ALT_INT=5, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location. | */
//    CF_LOCAL_OFFSET_NED=6, /* Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | */
//    CF_BODY_NED=7, /* Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | */
//    CF_BODY_OFFSET_NED=8, /* Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | */
//    CF_GLOBAL_TERRAIN_ALT=9, /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
//    CF_GLOBAL_TERRAIN_ALT_INT=10, /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
//    CF_UNKNOWN = 11
};

inline CoordinateFrame getCoordinateFrame(const CartesianFrameTypes &frame)
{
    switch (frame) {
    case CartesianFrameTypes::CF_LOCAL_NED:
        return CoordinateFrame::CF_LOCAL_NED;
        break;
    case CartesianFrameTypes::CF_LOCAL_ENU:
        return CoordinateFrame::CF_LOCAL_ENU;
        break;
    case CartesianFrameTypes::CF_LOCAL_OFFSET_NED:
        return CoordinateFrame::CF_LOCAL_OFFSET_NED;
        break;
    case CartesianFrameTypes::CF_BODY_NED:
        return CoordinateFrame::CF_BODY_NED;
        break;
    case CartesianFrameTypes::CF_BODY_OFFSET_NED:
        return CoordinateFrame::CF_BODY_OFFSET_NED;
        break;
    default:
        return CoordinateFrame::CF_LOCAL_ENU;
        break;
    }
}

inline CoordinateFrame getCoordinateFrame(const GeodeticFrameTypes &frame)
{
    CoordinateFrame currentFrameType = CoordinateFrame::CF_UNKNOWN;

    switch (frame) {
    case GeodeticFrameTypes::CF_GLOBAL_UNKNOWN:
        currentFrameType = CoordinateFrame::CF_GLOBAL_UNKNOWN;
        break;
    case GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT:
        currentFrameType = CoordinateFrame::CF_GLOBAL_RELATIVE_ALT;
        break;
    case GeodeticFrameTypes::CF_GLOBAL_INT:
        currentFrameType = CoordinateFrame::CF_GLOBAL_INT;
        break;
    case GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT_INT:
        currentFrameType = CoordinateFrame::CF_GLOBAL_RELATIVE_ALT_INT;
        break;
    case GeodeticFrameTypes::CF_GLOBAL_TERRAIN_ALT:
        currentFrameType = CoordinateFrame::CF_GLOBAL_TERRAIN_ALT;
        break;
    case GeodeticFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT:
        currentFrameType = CoordinateFrame::CF_GLOBAL_TERRAIN_ALT_INT;
        break;
    default:
        break;
    }

    return currentFrameType;
}

inline std::string CoordinateFrameToString(const CoordinateFrame &frame) {
    switch (frame) {
    case CoordinateFrame::CF_GLOBAL_UNKNOWN:
        return "CF_GLOBAL";
    case CoordinateFrame::CF_LOCAL_NED:
        return "CF_LOCAL_NED";
    case CoordinateFrame::CF_GLOBAL_RELATIVE_ALT:
        return "CF_GLOBAL_RELATIVE_ALT";
    case CoordinateFrame::CF_LOCAL_ENU:
        return "CF_LOCAL_ENU";
    case CoordinateFrame::CF_GLOBAL_INT:
        return "CF_GLOBAL_INT";
    case CoordinateFrame::CF_GLOBAL_RELATIVE_ALT_INT:
        return "CF_GLOBAL_RELATIVE_ALT_INT";
    case CoordinateFrame::CF_LOCAL_OFFSET_NED:
        return "CF_LOCAL_OFFSET_NED";
    case CoordinateFrame::CF_BODY_NED:
        return "CF_BODY_NED";
    case CoordinateFrame::CF_BODY_OFFSET_NED:
        return "CF_BODY_OFFSET_NED";
    case CoordinateFrame::CF_GLOBAL_TERRAIN_ALT:
        return "CF_GLOBAL_TERRAIN_ALT";
    case CoordinateFrame::CF_GLOBAL_TERRAIN_ALT_INT:
        return "CF_GLOBAL_TERRAIN_ALT_INT";
    default:
        throw std::runtime_error("Unknown coordinate system seen");
    }
}

inline CoordinateFrame CoordinateFrameFromString(const std::string &str) {
    if(str == "CF_GLOBAL")
        return CoordinateFrame::CF_GLOBAL_UNKNOWN;
    if(str == "CF_LOCAL_NED")
        return CoordinateFrame::CF_LOCAL_NED;
    if(str == "CF_GLOBAL_RELATIVE_ALT")
        return CoordinateFrame::CF_GLOBAL_RELATIVE_ALT;
    if(str == "CF_LOCAL_ENU")
        return CoordinateFrame::CF_LOCAL_ENU;
    if(str == "CF_GLOBAL_INT")
        return CoordinateFrame::CF_GLOBAL_INT;
    if(str == "CF_GLOBAL_RELATIVE_ALT_INT")
        return CoordinateFrame::CF_GLOBAL_RELATIVE_ALT_INT;
    if(str == "CF_LOCAL_OFFSET_NED")
        return CoordinateFrame::CF_LOCAL_OFFSET_NED;
    if(str == "CF_BODY_NED")
        return CoordinateFrame::CF_BODY_NED;
    if(str == "CF_BODY_OFFSET_NED")
        return CoordinateFrame::CF_BODY_OFFSET_NED;
    if(str == "CF_GLOBAL_TERRAIN_ALT")
        return CoordinateFrame::CF_GLOBAL_TERRAIN_ALT;
    if(str == "CF_GLOBAL_TERRAIN_ALT_INT")
        return CoordinateFrame::CF_GLOBAL_TERRAIN_ALT_INT;
    throw std::runtime_error("Unknown coordinate system seen");
}

} //end of namespace pose

} //end of namespace mace

#endif // COORDINATE_FRAME_H
