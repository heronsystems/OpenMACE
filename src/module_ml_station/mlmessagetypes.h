#ifndef MLMESSAGETYPES_H
#define MLMESSAGETYPES_H

#include <string>
#include <stdexcept>

enum class MLMessageTypes: uint8_t{
    VEHICLE_POSITION,
    VEHICLE_ATTITUDE,
    VEHICLE_CONTROL,
    VEHICLE_AIRSPEED,
    VEHICLE_FUEL,
    ENVIRONMENT_BOUNDARY,
    VEHICLE_PARAM_LIST,
    CONNECTED_VEHICLES

};

inline std::string MLMessageString(const MLMessageTypes &type) {
    switch (type) {
    case MLMessageTypes::VEHICLE_POSITION:
        return "vehicle_position";
    case MLMessageTypes::VEHICLE_ATTITUDE:
        return "vehicle_attitude";
    case MLMessageTypes::VEHICLE_CONTROL:
        return "vehicle_control";
    case MLMessageTypes::VEHICLE_AIRSPEED:
        return "vehicle_airspeed";
    case MLMessageTypes::VEHICLE_FUEL:
        return "vehicle_fuel";
    case MLMessageTypes::ENVIRONMENT_BOUNDARY:
        return "environment_boundary";
    case MLMessageTypes::VEHICLE_PARAM_LIST:
        return "vehicle_param_list";
    case MLMessageTypes::CONNECTED_VEHICLES:
        return "connected_vehicles";

    default:
        throw std::runtime_error("Unknown gui message type");
    }
}

#endif // MLESSAGETYPES_H
