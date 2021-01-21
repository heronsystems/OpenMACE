#ifndef GLOBAL_COORDINATE_FRAMES_H
#define GLOBAL_COORDINATE_FRAMES_H

#include <mavlink.h>

#include <string>

#include "altitude_coordinate_frames.h"

namespace mace {

#define GLOBAL_FRAMES CF_GLOBAL_UNKNOWN = MAV_FRAME::MAV_FRAME_GLOBAL_UNKNOWN, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | */ \
CF_GLOBAL_AMSL = MAV_FRAME::MAV_FRAME_GLOBAL, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: mean sea level (MSL) | */ \
CF_GLOBAL_RELATIVE_ALT = MAV_FRAME::MAV_FRAME_GLOBAL_RELATIVE_ALT, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */ \
CF_GLOBAL_INT = MAV_FRAME::MAV_FRAME_GLOBAL_INT, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL) | */ \
CF_GLOBAL_RELATIVE_ALT_INT = MAV_FRAME::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location. | */ \
CF_GLOBAL_TERRAIN_ALT = MAV_FRAME::MAV_FRAME_GLOBAL_TERRAIN_ALT, /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */ \
CF_GLOBAL_TERRAIN_ALT_INT = MAV_FRAME::MAV_FRAME_GLOBAL_TERRAIN_ALT_INT /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */

enum class GeodeticFrameTypes : uint8_t{
    GLOBAL_FRAMES
};

inline MAV_FRAME getMAVLINKCoordinateFrame(const mace::GeodeticFrameTypes &frame)
{
    MAV_FRAME currentFrame = MAV_FRAME_GLOBAL_RELATIVE_ALT; //we are going to default to LOCAL_ENU

    switch (frame) {
    case mace::GeodeticFrameTypes::CF_GLOBAL_UNKNOWN:
        break;
    case mace::GeodeticFrameTypes::CF_GLOBAL_AMSL:
        currentFrame = MAV_FRAME_GLOBAL;
        break;
    case mace::GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT:
        currentFrame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        break;
    case mace::GeodeticFrameTypes::CF_GLOBAL_INT:
        currentFrame = MAV_FRAME_GLOBAL_INT;
        break;
    case mace::GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT_INT:
        currentFrame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
        break;
    case mace::GeodeticFrameTypes::CF_GLOBAL_TERRAIN_ALT:
        currentFrame = MAV_FRAME_GLOBAL_TERRAIN_ALT;
        break;
    case mace::GeodeticFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT:
        currentFrame = MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
        break;
    } //end of switch statement

    return currentFrame;
}

inline AltitudeReferenceTypes getAltitudeReference(const GeodeticFrameTypes &frame)
{
    AltitudeReferenceTypes altitudeFrame = AltitudeReferenceTypes::REF_ALT_UNKNOWN;
    switch (frame) {
    case GeodeticFrameTypes::CF_GLOBAL_AMSL:
        altitudeFrame = AltitudeReferenceTypes::REF_ALT_MSL;
        break;
    case GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT:
    case GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT_INT:
        altitudeFrame = AltitudeReferenceTypes::REF_ALT_MSL;
        break;
    case GeodeticFrameTypes::CF_GLOBAL_TERRAIN_ALT:
    case GeodeticFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT:
        altitudeFrame = AltitudeReferenceTypes::REF_ALT_TERRAIN;
        break;
    default:
        break;
    }

    return altitudeFrame;
}

} //end of namespace mace

#endif // GLOBAL_COORDINATE_FRAMES_H
