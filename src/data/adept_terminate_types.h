#ifndef ADEPTTERMINATETYPES_H
#define ADEPTTERMINATETYPES_H

#include <iostream>

namespace Data
{

enum class AdeptTerminateType: uint8_t{
    RUNTIME,
    CLOSURE,
    UNKNOWN
};

inline AdeptTerminateType AdeptTerminateFromString(const std::string &type) {
    if (type == "RUNTIME"){
        return AdeptTerminateType::RUNTIME;
    } else if (type == "CLOSURE"){
        return AdeptTerminateType::CLOSURE;
    } else {
        return AdeptTerminateType::UNKNOWN;
    }
}

inline std::string AdeptTerminateToString(const AdeptTerminateType &team) {
    switch(team){
    case AdeptTerminateType::RUNTIME:
        return "RUNTIME";
    case AdeptTerminateType::CLOSURE:
        return "CLOSURE";
    case AdeptTerminateType::UNKNOWN:
        return "UNKNOWN";
    default:
        throw std::runtime_error("Unknown team name");
    }
}

} //end of namespace Data

#endif // ADEPTTERMINATETYPES_H
