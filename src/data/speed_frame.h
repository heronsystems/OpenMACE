#ifndef SPEED_FRAME_H
#define SPEED_FRAME_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class SpeedFrame{
    GROUNDSPEED,
    AIRSPEED
};

inline std::string SpeedFrameToString(const SpeedFrame &frame) {
    switch (frame) {
    case SpeedFrame::GROUNDSPEED:
        return "GROUNDSPEED";
    case SpeedFrame::AIRSPEED:
        return "AIRSPEED";
    default:
        throw std::runtime_error("Unknown speed frame seen");
    }
}

inline SpeedFrame SpeedFrameFromString(const std::string &str) {
    if(str == "GROUNDSPEED")
        return SpeedFrame::GROUNDSPEED;
    if(str == "AIRSPEED")
        return SpeedFrame::AIRSPEED;

    throw std::runtime_error("Unknown speed frame seen");
}

} //end of namespace Data

#endif // SPEED_FRAME_H
