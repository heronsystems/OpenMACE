#ifndef DATA_MISSION_STATE_H
#define DATA_MISSION_STATE_H

#include <stdint.h>
#include <string>
#include <stdexcept>

namespace Data {

enum class MISSIONSTATE : uint8_t
{
    CURRENT = 0,
    ONBOARD = 1,
    PROPOSED = 2,
    RECEIVED = 3,
    OUTDATED = 4
};

inline std::string MissionStateToString(const MISSIONSTATE &cmdType) {
    switch (cmdType) {
    case MISSIONSTATE::CURRENT:
        return "CURRENT";
    case MISSIONSTATE::ONBOARD:
        return "ONBOARD";
    case MISSIONSTATE::PROPOSED:
        return "PROPOSED";
    case MISSIONSTATE::RECEIVED:
        return "RECEIVED";
    case MISSIONSTATE::OUTDATED:
        return "OUTDATED";
    default:
        throw std::runtime_error("Unknown MISSIONSTATE seen");
    }
}

inline MISSIONSTATE MissionStateFromString(const std::string &str) {
    if(str == "CURRENT")
        return MISSIONSTATE::CURRENT;
    if(str == "ONBOARD")
        return MISSIONSTATE::ONBOARD;
    if(str == "PROPOSED")
        return MISSIONSTATE::PROPOSED;
    if(str == "RECEIVED")
        return MISSIONSTATE::RECEIVED;
    if(str == "OUTDATED")
        return MISSIONSTATE::OUTDATED;
    throw std::runtime_error("Unknown string MISSIONSTATE seen");
}

} //end of namespace Data

#endif // DATA_MISSION_STATE_H
