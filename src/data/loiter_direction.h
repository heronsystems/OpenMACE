#ifndef LOITER_DIRECTION_H
#define LOITER_DIRECTION_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class LoiterDirection{
    CW,
    CCW
};

inline std::string LoiterDirectionToString(const LoiterDirection &direction) {
    switch (direction) {
    case LoiterDirection::CW:
        return "CW";
    case LoiterDirection::CCW:
        return "CCW";
    default:
        throw std::runtime_error("Unknown coordinate system seen");
    }
}

inline LoiterDirection LoiterDirectionFromString(const std::string &str) {
    if(str == "CW")
        return LoiterDirection::CW;
    if(str == "CCW")
        return LoiterDirection::CCW;

    throw std::runtime_error("Unknown coordinate system seen");
}

}

#endif // LOITER_DIRECTION_H
