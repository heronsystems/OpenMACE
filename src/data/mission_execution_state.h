#ifndef MISSION_EXECUTION_STATE_H
#define MISSION_EXECUTION_STATE_H

#include <stdint.h>
#include <string>
#include <stdexcept>

namespace Data
{

enum class MissionExecutionState{
    MESTATE_UNEXECUTED = 0,
    MESTATE_EXECUTING = 1,
    MESTATE_PAUSED = 2,
    MESTATE_COMPLETED = 3,
    MESTATE_ABORTED = 4
};

inline std::string MissionExecutionStateToString(const MissionExecutionState &commandType) {
    switch (commandType) {
    case MissionExecutionState::MESTATE_UNEXECUTED:
        return "UNEXECUTED";
    case MissionExecutionState::MESTATE_EXECUTING:
        return "EXECUTING";
    case MissionExecutionState::MESTATE_PAUSED:
        return "PAUSED";
    case MissionExecutionState::MESTATE_COMPLETED:
        return "COMPLETED";
    case MissionExecutionState::MESTATE_ABORTED:
        return "ABORTED";
    default:
        throw std::runtime_error("Unknown MissionCommandAction seen");
    }
}

inline MissionExecutionState MissionExecutionStateFromString(const std::string &str) {
    if(str == "UNEXECUTED")
        return MissionExecutionState::MESTATE_UNEXECUTED;
    if(str == "EXECUTING")
        return MissionExecutionState::MESTATE_EXECUTING;
    if(str == "PAUSED")
        return MissionExecutionState::MESTATE_PAUSED;
    if(str == "COMPLETED")
        return MissionExecutionState::MESTATE_COMPLETED;
    if(str == "ABORTED")
        return MissionExecutionState::MESTATE_ABORTED;
    throw std::runtime_error("Unknown string MissionCommandAction seen");
}


}

#endif // MISSION_EXECUTION_STATE_H
