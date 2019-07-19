#ifndef ALTITUDE_COORDINATE_FRAMES_H
#define ALTITUDE_COORDINATE_FRAMES_H

#include <string>

namespace mace {

#define ALTITUDE_FRAMES REF_ALT_UNKNOWN, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */ \
REF_ALT_MSL, /* positive altitude over mean sea level (MSL) | */ \
REF_ALT_RELATIVE, /* positive altitude with 0 being at the altitude of the home location. | */ \
REF_ALT_OFFSET, /* Offset to the current altitude. | */ \
REF_ALT_TERRAIN /* positive altitude in meters with 0 being at ground level in terrain model. | */

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
