#ifndef COORDINATE_FRAME_COMPONENTS_H
#define COORDINATE_FRAME_COMPONENTS_H

#include <string>
#include <stdexcept>

#include "kinematic_definitions.h"

#include "altitude_coordinate_frames.h"
#include "global_coordinate_frames.h"
#include "local_coordindate_frames.h"

namespace mace {

enum class CoordinateFrameTypes : uint8_t{
    GLOBAL_FRAMES,
    LOCAL_FRAMES,
    CF_NOT_RELEVANT = 100,
    CF_UNKNOWN = 101

};

inline CoordinateSystemTypes getCoordinateSystemType(const CoordinateFrameTypes &frame)
{
    CoordinateSystemTypes currentSystemType;

    switch (frame) {
    case CoordinateFrameTypes::CF_LOCAL_UNKNOWN:
    case CoordinateFrameTypes::CF_LOCAL_NED:
    case CoordinateFrameTypes::CF_LOCAL_ENU:
    case CoordinateFrameTypes::CF_BODY_OFFSET_NED:
    case CoordinateFrameTypes::CF_LOCAL_OFFSET_NED:
    case CoordinateFrameTypes::CF_BODY_NED:
    case CoordinateFrameTypes::CF_BODY_ENU:
    {
        currentSystemType = CoordinateSystemTypes::CARTESIAN;
        break;
    }
    case CoordinateFrameTypes::CF_GLOBAL_UNKNOWN:
    case CoordinateFrameTypes::CF_GLOBAL_AMSL:
    case CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT:
    case CoordinateFrameTypes::CF_GLOBAL_INT:
    case CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT_INT:
    case CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT:
    case CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT:
    {
        currentSystemType = CoordinateSystemTypes::GEODETIC;
        break;
    }
    case CoordinateFrameTypes::CF_UNKNOWN:
    case CoordinateFrameTypes::CF_NOT_RELEVANT:
    {
        currentSystemType = CoordinateSystemTypes::NOT_IMPLIED;
    }
    }

    return currentSystemType;
}

inline CartesianFrameTypes getCartesianCoordinateFrame(const CoordinateFrameTypes &frame)
{
    switch (frame) {
    case CoordinateFrameTypes::CF_LOCAL_NED:
        return CartesianFrameTypes::CF_LOCAL_NED;
    case CoordinateFrameTypes::CF_LOCAL_ENU:
        return CartesianFrameTypes::CF_LOCAL_ENU;
    case CoordinateFrameTypes::CF_BODY_OFFSET_NED:
        return CartesianFrameTypes::CF_BODY_OFFSET_NED;
    case CoordinateFrameTypes::CF_LOCAL_OFFSET_NED:
        return CartesianFrameTypes::CF_LOCAL_OFFSET_NED;
    case CoordinateFrameTypes::CF_BODY_NED:
        return CartesianFrameTypes::CF_BODY_NED;
    case CoordinateFrameTypes::CF_BODY_ENU:
        return CartesianFrameTypes::CF_BODY_ENU;
    default:
        return CartesianFrameTypes::CF_LOCAL_UNKNOWN;
    }
}

inline CoordinateFrameTypes getCoordinateFrame(const CartesianFrameTypes &frame)
{
    switch (frame) {
    case CartesianFrameTypes::CF_LOCAL_NED:
        return CoordinateFrameTypes::CF_LOCAL_NED;
    case CartesianFrameTypes::CF_LOCAL_ENU:
        return CoordinateFrameTypes::CF_LOCAL_ENU;
    case CartesianFrameTypes::CF_BODY_OFFSET_NED:
        return CoordinateFrameTypes::CF_BODY_OFFSET_NED;
    case CartesianFrameTypes::CF_LOCAL_OFFSET_NED:
        return CoordinateFrameTypes::CF_LOCAL_OFFSET_NED;
    case CartesianFrameTypes::CF_BODY_NED:
        return CoordinateFrameTypes::CF_BODY_NED;
    case CartesianFrameTypes::CF_BODY_ENU:
        return CoordinateFrameTypes::CF_BODY_ENU;
    default:
        return CoordinateFrameTypes::CF_LOCAL_UNKNOWN;
    }
}

inline GeodeticFrameTypes getGeodeticCoordinateFrame(const CoordinateFrameTypes &frame)
{
    switch (frame) {
    case CoordinateFrameTypes::CF_GLOBAL_UNKNOWN:
        return GeodeticFrameTypes::CF_GLOBAL_UNKNOWN;
    case CoordinateFrameTypes::CF_GLOBAL_AMSL:
        return GeodeticFrameTypes::CF_GLOBAL_AMSL;
    case CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT:
        return GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT;
    case CoordinateFrameTypes::CF_GLOBAL_INT:
        return GeodeticFrameTypes::CF_GLOBAL_INT;
    case CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT_INT:
        return GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT_INT;
    case CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT:
        return GeodeticFrameTypes::CF_GLOBAL_TERRAIN_ALT;
    case CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT:
        return GeodeticFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT;
    default:
        return GeodeticFrameTypes::CF_GLOBAL_UNKNOWN;
    }
}

inline CoordinateFrameTypes getCoordinateFrame(const GeodeticFrameTypes &frame)
{
    switch (frame) {
    case GeodeticFrameTypes::CF_GLOBAL_UNKNOWN:
        return CoordinateFrameTypes::CF_GLOBAL_UNKNOWN;
    case GeodeticFrameTypes::CF_GLOBAL_AMSL:
        return CoordinateFrameTypes::CF_GLOBAL_AMSL;
    case GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT:
        return CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT;
    case GeodeticFrameTypes::CF_GLOBAL_INT:
        return CoordinateFrameTypes::CF_GLOBAL_INT;
    case GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT_INT:
        return CoordinateFrameTypes::CF_GLOBAL_RELATIVE_ALT_INT;
    case GeodeticFrameTypes::CF_GLOBAL_TERRAIN_ALT:
        return CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT;
    case GeodeticFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT:
        return CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT;
    default:
        return CoordinateFrameTypes::CF_GLOBAL_UNKNOWN;
    }
}

inline std::string CoordinateFrameToString(const CoordinateFrameTypes &frame) {
    switch (frame) {
    case CoordinateFrameTypes::CF_GLOBAL_AMSL:
        return "CF_GLOBAL_AMSL";
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
    case CoordinateFrameTypes::CF_BODY_OFFSET_NED:
        return "CF_BODY_OFFSET_NED";
    case CoordinateFrameTypes::CF_LOCAL_OFFSET_NED:
        return "CF_LOCAL_OFFSET_NED";
    case CoordinateFrameTypes::CF_BODY_NED:
        return "CF_BODY_NED";
    case CoordinateFrameTypes::CF_BODY_ENU:
        return "CF_BODY_ENU";
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
    if(str == "CF_GLOBAL_AMSL")
        return CoordinateFrameTypes::CF_GLOBAL_AMSL;
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
    if(str == "CF_BODY_OFFSET_NED")
        return CoordinateFrameTypes::CF_BODY_OFFSET_NED;
    if(str == "CF_LOCAL_OFFSET_NED")
        return CoordinateFrameTypes::CF_LOCAL_OFFSET_NED;
    if(str == "CF_BODY_NED")
        return CoordinateFrameTypes::CF_BODY_NED;
    if(str == "CF_BODY_ENU")
        return CoordinateFrameTypes::CF_BODY_ENU;
    if(str == "CF_GLOBAL_TERRAIN_ALT")
        return CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT;
    if(str == "CF_GLOBAL_TERRAIN_ALT_INT")
        return CoordinateFrameTypes::CF_GLOBAL_TERRAIN_ALT_INT;
    throw std::runtime_error("Unknown coordinate system seen");
}

inline MAV_FRAME getMAVLINKCoordinateFrame(const mace::CoordinateFrameTypes &frame)
{
    MAV_FRAME currentFrame = MAV_FRAME_ENUM_END;

    if(getCoordinateSystemType(frame) == CoordinateSystemTypes::GEODETIC)
        currentFrame = getMAVLINKCoordinateFrame(mace::getGeodeticCoordinateFrame(frame));
    else if(getCoordinateSystemType(frame) == CoordinateSystemTypes::CARTESIAN)
        currentFrame = getMAVLINKCoordinateFrame(mace::getCartesianCoordinateFrame(frame));
    else {
        throw std::logic_error("There is no coordinate system type that can be parsed into a mavlink definition.");
    }

    return currentFrame;
}

} //end of namespace mace

#endif // COORDINATE_FRAME_COMPONENTS_H
