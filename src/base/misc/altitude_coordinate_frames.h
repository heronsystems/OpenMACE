#ifndef ALTITUDE_COORDINATE_FRAMES_H
#define ALTITUDE_COORDINATE_FRAMES_H

#include <string>
#include <stdexcept>

namespace mace {

#define ALTITUDE_FRAMES REF_ALT_UNKNOWN, /* Current reference frame for altitude is currently undefined. | */ \
REF_ALT_MSL, /* Positive altitude over mean sea level (MSL) | */ \
REF_ALT_RELATIVE, /* Positive altitude with 0 being referenced to the home location (traditionally the starting location). | */ \
REF_ALT_OFFSET, /* Offset relative to the current altitude. | */ \
REF_ALT_TERRAIN /* Positive altitude in meters with 0 being at ground level in terrain model or as currently measured (traditionally a LiDAR or similar ranging device). | */

enum class AltitudeReferenceTypes: uint8_t{
    ALTITUDE_FRAMES
};

inline std::string AltitudeFrameToString(const AltitudeReferenceTypes &frame) {
    switch (frame) {
    case AltitudeReferenceTypes::REF_ALT_MSL:
        return "REF_ALT_MSL";
    case AltitudeReferenceTypes::REF_ALT_RELATIVE:
        return "REF_ALT_RELATIVE";
    case AltitudeReferenceTypes::REF_ALT_OFFSET:
        return "REF_ALT_OFFSET";
    case AltitudeReferenceTypes::REF_ALT_TERRAIN:
        return "REF_ALT_TERRAIN";
    case AltitudeReferenceTypes::REF_ALT_UNKNOWN:
        return "REF_ALT_UNKNOWN";
    default:
        throw std::runtime_error("Unknown altitude reference seen");
    }
}

inline AltitudeReferenceTypes AltitudeFrameFromString(const std::string &str) {
    if(str == "REF_ALT_MSL")
        return AltitudeReferenceTypes::REF_ALT_MSL;
    if(str == "REF_ALT_RELATIVE")
        return AltitudeReferenceTypes::REF_ALT_RELATIVE;
    if(str == "REF_ALT_OFFSET")
        return AltitudeReferenceTypes::REF_ALT_OFFSET;
    if(str == "REF_ALT_TERRAIN")
        return AltitudeReferenceTypes::REF_ALT_TERRAIN;
    if(str == "REF_ALT_UNKNOWN")
        return AltitudeReferenceTypes::REF_ALT_UNKNOWN;
    throw std::runtime_error("Unknown altitude reference seen");
}


} //end of namespace mace

#endif // ALTITUDE_COORDINATE_FRAMES_H
