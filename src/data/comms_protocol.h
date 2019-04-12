#ifndef COMMS_PROTOCOL_H
#define COMMS_PROTOCOL_H

#include <string>
#include <stdexcept>

namespace Data
{
enum class CommsProtocol
{
    COMMS_MACE=0, /* Generic micro air vehicle. | */
    COMMS_MAVLINK=1, /* Normal helicopter with tail rotor. | */
    COMMS_DJI=2, /* Operator control unit / ground control station | */
    COMMS_UNKNOWN=3, /* Operator control unit / ground control station | */
};


inline std::string CommsProtocolToString(const CommsProtocol &commsProtocol) {
    switch (commsProtocol) {
    case CommsProtocol::COMMS_MACE:
        return "COMMS_MACE";
    case CommsProtocol::COMMS_MAVLINK:
        return "COMMS_MAVLINK";
    case CommsProtocol::COMMS_DJI:
        return "COMMS_DJI";
    case CommsProtocol::COMMS_UNKNOWN:
        return "COMMS_UNKNOWN";
    default:
        throw std::runtime_error("Unknown communication protocol seen");
    }
}

inline CommsProtocol CommsProtocolFromString(const std::string &str) {
    if(str == "COMMS_MACE")
        return CommsProtocol::COMMS_MACE;
    if(str == "COMMS_MAVLINK")
        return CommsProtocol::COMMS_MAVLINK;
    if(str == "COMMS_DJI")
        return CommsProtocol::COMMS_DJI;
    if(str == "COMMS_UNKNOWN")
        return CommsProtocol::COMMS_UNKNOWN;
    throw std::runtime_error("Unknown communication protocol seen");
}

}

#endif // COMMS_PROTOCOL_H
