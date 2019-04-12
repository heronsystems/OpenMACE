#ifndef BOUNDARY_TYPE_H
#define BOUNDARY_TYPE_H

namespace BoundaryItem {

enum class BOUNDARYTYPE : uint8_t
{
    OPERATIONAL_FENCE = 0,
    RESOURCE_FENCE = 1,
    GENERIC_POLYGON = 2,
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
    switch (type) {
    case BOUNDARYTYPE::OPERATIONAL_FENCE:
        return "OPERATIONAL FENCE";
    case BOUNDARYTYPE::RESOURCE_FENCE:
        return "RESOURCE FENCE";
    case BOUNDARYTYPE::GENERIC_POLYGON:
        return "GENERIC POLYGON";
    default:
        throw std::runtime_error("Unknown boundary type seen");
    }
}

} //end of namespace MissionItem

#endif // BOUNDARY_TYPE_H
