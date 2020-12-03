#ifndef MISSION_TYPE_H
#define MISSION_TYPE_H

#include <stdint.h>
#include <string>
#include <stdexcept>

namespace MissionItem {

enum class MISSIONTYPE : uint8_t
{
    AUTO = 0,
    GUIDED = 1,
    ROI = 2,
    FENCE = 3,
    RALLY = 4,
    ALL = 5
};

inline MISSIONTYPE MissionTypeFromString(const std::string &str) {
    if(str == "AUTO")
        return MISSIONTYPE::AUTO;
    if(str == "GUIDED")
        return MISSIONTYPE::GUIDED;
    if(str == "ROI")
        return MISSIONTYPE::ROI;
    if(str == "FENCE")
        return MISSIONTYPE::FENCE;
    if(str == "RALLY")
        return MISSIONTYPE::RALLY;
    if(str == "ALL")
        return MISSIONTYPE::ALL;
    throw std::runtime_error("Unknown string MissionType seen");
}

inline std::string MissionTypeToString(const MISSIONTYPE &type) {
    switch (type) {
    case MISSIONTYPE::AUTO:
        return "AUTO";
    case MISSIONTYPE::GUIDED:
        return "GUIDED";
    case MISSIONTYPE::ROI:
        return "ROI";
    case MISSIONTYPE::FENCE:
        return "FENCE";
    case MISSIONTYPE::RALLY:
        return "RALLY";
    case MISSIONTYPE::ALL:
        return "ALL";
    default:
        throw std::runtime_error("Unknown MissionType seen");
    }
}

} //end of namespace MissionItem
#endif // MISSION_TYPE_H
