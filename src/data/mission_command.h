#ifndef MISSION_COMMAND_H
#define MISSION_COMMAND_H

#include <stdint.h>
#include <string>
#include <stdexcept>

namespace Data
{

enum class MissionCommandAction{
    MISSIONCA_START = 0,
    MISSIONCA_PAUSE = 1,
    MISSIONCA_RESUME = 2
};

inline std::string MissionCommandActionToString(const MissionCommandAction &commandType) {
    switch (commandType) {
    case MissionCommandAction::MISSIONCA_START:
        return "START";
    case MissionCommandAction::MISSIONCA_PAUSE:
        return "PAUSE";
    case MissionCommandAction::MISSIONCA_RESUME:
        return "RESUME";
    default:
        throw std::runtime_error("Unknown MissionCommandAction seen");
    }
}

inline MissionCommandAction MissionCommandActionFromString(const std::string &str) {
    if(str == "START")
        return MissionCommandAction::MISSIONCA_START;
    if(str == "PAUSE")
        return MissionCommandAction::MISSIONCA_PAUSE;
    if(str == "RESUME")
        return MissionCommandAction::MISSIONCA_RESUME;
    throw std::runtime_error("Unknown string MissionCommandAction seen");
}


}

#endif // MISSION_COMMAND_H
