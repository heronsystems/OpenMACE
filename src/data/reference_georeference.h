#ifndef REFERENCE_GEOREFERENCE_H
#define REFERENCE_GEOREFERENCE_H

#include <string>
#include <stdexcept>

namespace Data
{
enum class ReferenceGeoCoords{
    REF_GEO_DEG=1, /* latitude/longitude in degres | */
    REF_GEO_INT=2, /* latitude/longitude in degrees*1.0e-7 | */
    REF_GEO_ENUM_END=3, /*  | */
};

inline std::string ReferenceGeoCoordsToString(const ReferenceGeoCoords &frame) {
    switch (frame) {
    case ReferenceGeoCoords::REF_GEO_DEG:
        return "REF_GEO_DEG";
    case ReferenceGeoCoords::REF_GEO_INT:
        return "REF_GEO_INT";
    default:
        throw std::runtime_error("Unknown altitude reference seen");
    }
}

inline ReferenceGeoCoords ReferenceGeoCoordsFromString(const std::string &str) {
    if(str == "REF_GEO_DEG")
        return ReferenceGeoCoords::REF_GEO_DEG;
    if(str == "REF_GEO_INT")
        return ReferenceGeoCoords::REF_GEO_INT;
    throw std::runtime_error("Unknown altitude reference seen");
}

} //end of namespace Data

#endif // REFERENCE_GEOREFERENCE_H
