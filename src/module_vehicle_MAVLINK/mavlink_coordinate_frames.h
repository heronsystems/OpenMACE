#ifndef MAVLINK_COORDINATE_FRAMES_H
#define MAVLINK_COORDINATE_FRAMES_H

#include <mavlink.h>

#include <sstream>
#include <iostream>
#include <exception>

#include "base/misc/coordinate_frame_components.h"

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

inline MAV_FRAME getMAVLINKCoordinateFrame(const mace::CartesianFrameTypes &frame)
{
    MAV_FRAME currentFrame = MAV_FRAME_LOCAL_ENU; //we are going to default to LOCAL_ENU

    switch (frame) {
    case mace::CartesianFrameTypes::CF_LOCAL_UNKNOWN:
        break;
    case mace::CartesianFrameTypes::CF_LOCAL_NED:
        currentFrame = MAV_FRAME_LOCAL_NED;
        break;
    case mace::CartesianFrameTypes::CF_LOCAL_ENU:
        currentFrame = MAV_FRAME_LOCAL_ENU;
        break;
    case mace::CartesianFrameTypes::CF_BODY_OFFSET_NED:
        currentFrame = MAV_FRAME_BODY_OFFSET_NED;
        break;
    case mace::CartesianFrameTypes::CF_LOCAL_OFFSET_NED:
        currentFrame = MAV_FRAME_LOCAL_OFFSET_NED;
        break;
    case mace::CartesianFrameTypes::CF_BODY_NED:
        currentFrame = MAV_FRAME_BODY_FRD;
        break;
    case mace::CartesianFrameTypes::CF_BODY_ENU:
        throw std::logic_error("There is no coordinate frame that is equivalent within the mavlink definition set.");
    } //end of switch statement

    return currentFrame;
}

inline MAV_FRAME getMAVLINKCoordinateFrame(const mace::CoordinateFrameTypes &frame)
{
    MAV_FRAME currentFrame = MAV_FRAME_ENUM_END;

    if(getCoordinateSystemType(frame) == CoordinateSystemTypes::GEODETIC)
        currentFrame = getMAVLINKCoordinateFrame(mace::getGeodeticCoordinateFrame(frame));
    else if(getCoordinateSystemType(frame) == CoordinateSystemTypes::CARTESIAN)
        currentFrame = getMAVLINKCoordinateFrame(mace::getGeodeticCoordinateFrame(frame));
    else {
        throw std::logic_error("There is no coordinate system type that can be parsed into a mavlink definition.");
    }

    return currentFrame;
}

#endif // MAVLINK_COORDINATE_FRAMES_H
