#ifndef COMMAND_ACK_H
#define COMMAND_ACK_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class CommandACKType{
    CA_RECEIVED=0, /* Command / mission item is ok. | */
    CA_ACCEPTED=1, /* Command / mission item is ok. | */
    CA_REJECTED=2, /* Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. | */
    CA_NOT_SUPPORTED=3, /* Command or mission item is not supported, other commands would be accepted. | */
    CA_FAILED = 4,
    CA_UNKNOWN = 5
};

inline std::string CommandACKToString(const CommandACKType &frame) {
    switch (frame) {
    case CommandACKType::CA_RECEIVED:
        return "RECEIVED";
    case CommandACKType::CA_ACCEPTED:
        return "ACCEPTED";
    case CommandACKType::CA_REJECTED:
        return "REJECTED";
    case CommandACKType::CA_NOT_SUPPORTED:
        return "NOT SUPPORTED";
    case CommandACKType::CA_FAILED:
        return "FAILED";
    case CommandACKType::CA_UNKNOWN:
        return "UNKNOWN";
    default:
        throw std::runtime_error("Unknown command acknowledgement seen");
    }
}

inline CommandACKType CommandACKFromString(const std::string &str) {
    if(str == "RECEIVED")
        return CommandACKType::CA_RECEIVED;
    if(str == "ACCEPTED")
        return CommandACKType::CA_ACCEPTED;
    if(str == "REJECTED")
        return CommandACKType::CA_REJECTED;
    if(str == "NOT_SUPPORTED")
        return CommandACKType::CA_NOT_SUPPORTED;
    if(str == "FAILED")
        return CommandACKType::CA_FAILED;
    if(str == "UNKNOWN")
        return CommandACKType::CA_UNKNOWN;
    throw std::runtime_error("Unknown command acknowledgment seen");
}

} //end of namespace Data

#endif // COMMAND_ACK_H
