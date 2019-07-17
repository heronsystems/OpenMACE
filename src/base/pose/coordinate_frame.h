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

#define ALTITUDE_FRAMES REF_ALT_UNKNOWN, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */ \
REF_ALT_MSL, /* positive altitude over mean sea level (MSL) | */ \
REF_ALT_RELATIVE, /* positive altitude with 0 being at the altitude of the home location. | */ \
REF_ALT_OFFSET, /* Offset to the current altitude. | */ \
REF_ALT_TERRAIN /* positive altitude in meters with 0 being at ground level in terrain model. | */

enum class AltitudeReferenceTypes: uint8_t{
    ALTITUDE_FRAMES
};

enum class CartesianFrameTypes: uint8_t{
    LOCAL_FRAMES
};

enum class GeodeticFrameTypes : uint8_t{
    GLOBAL_FRAMES
};

enum class CoordinateFrameTypes : uint8_t{
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

inline CoordinateFrameTypes getCoordinateFrame(const CartesianFrameTypes &frame)
{
    switch (frame) {
    case CartesianFrameTypes::CF_LOCAL_NED:
        return CoordinateFrameTypes::CF_LOCAL_NED;
        break;
    case CartesianFrameTypes::CF_LOCAL_ENU:
        return CoordinateFrameTypes::CF_LOCAL_ENU;
        break;
    case CartesianFrameTypes::CF_LOCAL_OFFSET_NED:
        return CoordinateFrameTypes::CF_LOCAL_OFFSET_NED;
        break;
    case CartesianFrameTypes::CF_BODY_NED:
        return CoordinateFrameTypes::CF_BODY_NED;
        break;
    case CartesianFrameTypes::CF_BODY_OFFSET_NED:
        return CoordinateFrameTypes::CF_BODY_OFFSET_NED;
        break;
    default:
        return CoordinateFrameTypes::CF_LOCAL_ENU;
        break;
    }
}

inline CoordinateFrameTypes getCoordinateFrame(const GeodeticFrameTypes &frame)
{
    CoordinateFrameTypes currentFrameType = CoordinateFrameTypes::CF_UNKNOWN;

    switch (frame) {
    case GeodeticFrameTypes::CF_GLOBAL_UNKNOWN:
        currentFrameType = CoordinateFrameTypes::CF_GLOBAL_UNKNOWN;
        break;
    case GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT:
        currentFrameType = CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT;
        break;
    case GeodeticFrameTypes::CF_GLOBAL_INT:
        currentFrameType = CoordinateFrameTypes::CF_GLOBAL_INT;
        break;
    case GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT_INT:
        currentFrameType = CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT_INT;
        break;
    case GeodeticFrameTypes::CF_GLOBAL_TERRAIN_ALT:
        currentFrameType = CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT;
        break;
    case GeodeticFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT:
        currentFrameType = CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT;
        break;
    default:
        break;
    }

    return currentFrameType;
}

inline std::string CoordinateFrameToString(const CoordinateFrameTypes &frame) {
    switch (frame) {
    case CoordinateFrameTypes::CF_GLOBAL_UNKNOWN:
        return "CF_GLOBAL";
    case CoordinateFrameTypes::CF_LOCAL_NED:
        return "CF_LOCAL_NED";
    case CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT:
        return "CF_GLOBAL_RELATIVE_ALT";
    case CoordinateFrameTypes::CF_LOCAL_ENU:
        return "CF_LOCAL_ENU";
    case CoordinateFrameTypes::CF_GLOBAL_INT:
        return "CF_GLOBAL_INT";
    case CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT_INT:
        return "CF_GLOBAL_RELATIVE_ALT_INT";
    case CoordinateFrameTypes::CF_LOCAL_OFFSET_NED:
        return "CF_LOCAL_OFFSET_NED";
    case CoordinateFrameTypes::CF_BODY_NED:
        return "CF_BODY_NED";
    case CoordinateFrameTypes::CF_BODY_OFFSET_NED:
        return "CF_BODY_OFFSET_NED";
    case CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT:
        return "CF_GLOBAL_TERRAIN_ALT";
    case CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT:
        return "CF_GLOBAL_TERRAIN_ALT_INT";
    default:
        throw std::runtime_error("Unknown coordinate system seen");
    }
}

inline CoordinateFrameTypes CoordinateFrameFromString(const std::string &str) {
    if(str == "CF_GLOBAL")
        return CoordinateFrameTypes::CF_GLOBAL_UNKNOWN;
    if(str == "CF_LOCAL_NED")
        return CoordinateFrameTypes::CF_LOCAL_NED;
    if(str == "CF_GLOBAL_RELATIVE_ALT")
        return CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT;
    if(str == "CF_LOCAL_ENU")
        return CoordinateFrameTypes::CF_LOCAL_ENU;
    if(str == "CF_GLOBAL_INT")
        return CoordinateFrameTypes::CF_GLOBAL_INT;
    if(str == "CF_GLOBAL_RELATIVE_ALT_INT")
        return CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT_INT;
    if(str == "CF_LOCAL_OFFSET_NED")
        return CoordinateFrameTypes::CF_LOCAL_OFFSET_NED;
    if(str == "CF_BODY_NED")
        return CoordinateFrameTypes::CF_BODY_NED;
    if(str == "CF_BODY_OFFSET_NED")
        return CoordinateFrameTypes::CF_BODY_OFFSET_NED;
    if(str == "CF_GLOBAL_TERRAIN_ALT")
        return CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT;
    if(str == "CF_GLOBAL_TERRAIN_ALT_INT")
        return CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT;
    throw std::runtime_error("Unknown coordinate system seen");
}

} //end of namespace pose

} //end of namespace mace

#endif // COORDINATE_FRAME_H
