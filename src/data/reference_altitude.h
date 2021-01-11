#ifndef REFERENCE_ALTITUDE_H
#define REFERENCE_ALTITUDE_H

#include <string>
#include <stdexcept>

namespace Data
{
enum class ReferenceAltitude{
    REF_ALT_MSL=1, /* positive altitude over mean sea level (MSL) | */
    REF_ALT_RELATIVE=2, /* positive altitude with 0 being at the altitude of the home location. | */
    REF_ALT_OFFSET=3, /* Offset to the current altitude. | */
    REF_ALT_TERRAIN=4, /* positive altitude in meters with 0 being at ground level in terrain model. | */
    REF_ALT_ENUM_END=5, /*  | */
};

inline std::string ReferenceAltitudeToString(const ReferenceAltitude &frame) {
    switch (frame) {
    case ReferenceAltitude::REF_ALT_MSL:
        return "REF_ALT_GLOBAL";
    case ReferenceAltitude::REF_ALT_RELATIVE:
        return "REF_ALT_RELATIVE";
    case ReferenceAltitude::REF_ALT_OFFSET:
        return "REF_ALT_OFFSET";
    case ReferenceAltitude::REF_ALT_TERRAIN:
        return "REF_ALT_TERRAIN";
    default:
        throw std::runtime_error("Unknown altitude reference seen");
    }
}

inline ReferenceAltitude ReferenceAltitudeFromString(const std::string &str) {
    if(str == "REF_ALT_GLOBAL")
        return ReferenceAltitude::REF_ALT_MSL;
    if(str == "REF_ALT_RELATIVE")
        return ReferenceAltitude::REF_ALT_RELATIVE;
    if(str == "REF_ALT_OFFSET")
        return ReferenceAltitude::REF_ALT_OFFSET;
    if(str == "REF_ALT_TERRAIN")
        return ReferenceAltitude::REF_ALT_TERRAIN;
    throw std::runtime_error("Unknown altitude reference seen");
}

} //end of namespace Data

#endif // REFERENCE_ALTITUDE_H
