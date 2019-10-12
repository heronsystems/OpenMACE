#ifndef BOUNDARY_TYPE_H
#define BOUNDARY_TYPE_H

#include <cassert>
#include <stdexcept>
#include <string>

namespace BoundaryItem {

enum class BOUNDARYTYPE : uint8_t
{
    OPERATIONAL_FENCE = 0,
    RESOURCE_FENCE = 1,
    GENERIC_POLYGON = 2
};

inline BOUNDARYTYPE BoundaryTypeFromString(const std::string &str) {
    if(str == "OPERATIONAL FENCE")
        return BOUNDARYTYPE::OPERATIONAL_FENCE;
    if(str == "RESOURCE FENCE")
        return BOUNDARYTYPE::RESOURCE_FENCE;
    if(str == "GENERIC POLYGON")
        return BOUNDARYTYPE::GENERIC_POLYGON;
    throw std::runtime_error("Unknown string boundary type seen");
}

inline std::string BoundaryTypeToString(const BOUNDARYTYPE &type) {
    assert(type == BOUNDARYTYPE::OPERATIONAL_FENCE ||
           type == BOUNDARYTYPE::RESOURCE_FENCE ||
           type == BOUNDARYTYPE::GENERIC_POLYGON);

    std::string returnString = "";

    switch (type) {
    case BOUNDARYTYPE::OPERATIONAL_FENCE:
        returnString = "OPERATIONAL FENCE";
        break;
    case BOUNDARYTYPE::RESOURCE_FENCE:
        returnString = "RESOURCE FENCE";
        break;
    case BOUNDARYTYPE::GENERIC_POLYGON:
        returnString = "GENERIC POLYGON";
        break;
    }
    return returnString;
}

} //end of namespace MissionItem

#endif // BOUNDARY_TYPE_H
