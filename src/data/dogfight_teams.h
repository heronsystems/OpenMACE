#ifndef DOGFIGHT_TEAMS_H
#define DOGFIGHT_TEAMS_H
#include <iostream>

namespace Data
{

enum class DogfightTeam: uint8_t{
    BLUE,
    RED,
    UNKNOWN
};

inline DogfightTeam DogfightTeamFromString(const std::string &team) {
    if (team == "BLUE"){
        return DogfightTeam::BLUE;
    } else if (team == "RED"){
        return DogfightTeam::RED;
    } else {
        return DogfightTeam::UNKNOWN;
    }
}

inline std::string DogfightTeamtoString(const DogfightTeam &team) {
    switch(team){
    case DogfightTeam::BLUE:
        return "BLUE";
    case DogfightTeam::RED:
        return "RED";
    case DogfightTeam::UNKNOWN:
        return "UNKNOWN";
    default:
        throw std::runtime_error("Unknown team name");
    }
}
} //end of namespace Data
#endif // DOGFIGHT_TEAMS_H
