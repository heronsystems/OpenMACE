#ifndef AI_COMMAND_TYPE_H
#define AI_COMMAND_TYPE_H

#include <mavlink.h>

#include <string>
#include <stdexcept>

namespace command_item
{

enum class COMMANDTYPE_AI : uint8_t{
    CAI_EVENT_TAG = 0,
    CAI_PROCEDURAL_ACTION = 1,
    CAI_ACT_TEST_INITIALIZATION = 3,
    CAI_UNKNOWN = 4,
    COMMANDTYPE_AIEND = 5
};

inline std::string AICommandItemToString(const COMMANDTYPE_AI &commandItemType) {
    switch (commandItemType) {
    case COMMANDTYPE_AI::CAI_EVENT_TAG:
        return "CAI_EVENT_TAG";
    case COMMANDTYPE_AI::CAI_PROCEDURAL_ACTION:
        return "CAI_PROC_ACT";
    case COMMANDTYPE_AI::CAI_ACT_TEST_INITIALIZATION:
        return "CAI_INI";
    case COMMANDTYPE_AI::CAI_UNKNOWN:
        return "CAI_UNKNOWN";
    default:
        throw std::runtime_error("Unknown ai command enum seen");
    }
}

inline COMMANDTYPE_AI AICommandItemFromString(const std::string &str) {
    if(str == "CAI_EVENT_TAG")
        return COMMANDTYPE_AI::CAI_EVENT_TAG;
    if(str == "CAI_PROC_ACT")
        return COMMANDTYPE_AI::CAI_PROCEDURAL_ACTION;
    if(str == "CAI_INI")
        return COMMANDTYPE_AI::CAI_ACT_TEST_INITIALIZATION;
    return COMMANDTYPE_AI::CAI_UNKNOWN;
}

} //end of namespace Data

#endif // AI_COMMAND_TYPES_H
