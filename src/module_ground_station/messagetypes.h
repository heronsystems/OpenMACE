#ifndef MESSAGETYPES_H
#define MESSAGETYPES_H

#include <string>
#include <stdexcept>

enum class GuiMessageTypes: uint8_t{
    VEHICLE_TARGET,
    VEHICLE_HOME,
    VEHICLE_POSITION,
    VEHICLE_ATTITUDE,
    VEHICLE_AIRSPEED,
    VEHICLE_FUEL,
    VEHICLE_MODE,
    VEHICLE_TEXT,
    VEHICLE_GPS,
    VEHICLE_ARM,
    VEHICLE_HEARTBEAT,
    VEHICLE_MISSION,
    VEHICLE_PATH,
    VEHICLE_PARAM_LIST,
    GLOBAL_ORIGIN,
    CURRENT_MISSION_ITEM,
    MISSION_ITEM_REACHED,
    SENSOR_FOOTPRINT,
    ENVIRONMENT_BOUNDARY
};

inline std::string guiMessageString(const GuiMessageTypes &type) {
    switch (type) {
    case GuiMessageTypes::VEHICLE_TARGET:
        return "vehicle_target";
    case GuiMessageTypes::VEHICLE_HOME:
        return "environment_icon";
    case GuiMessageTypes::VEHICLE_POSITION:
        return "vehicle_position";
    case GuiMessageTypes::VEHICLE_ATTITUDE:
        return "vehicle_attitude";
    case GuiMessageTypes::VEHICLE_AIRSPEED:
        return "vehicle_airspeed";
    case GuiMessageTypes::VEHICLE_FUEL:
        return "vehicle_fuel";
    case GuiMessageTypes::VEHICLE_MODE:
        return "vehicle_mode";
    case GuiMessageTypes::VEHICLE_TEXT:
        return "vehicle_text";
    case GuiMessageTypes::VEHICLE_GPS:
        return "vehicle_gps";
    case GuiMessageTypes::VEHICLE_ARM:
        return "vehicle_arm";
    case GuiMessageTypes::VEHICLE_HEARTBEAT:
        return "vehicle_heartbeat";
    case GuiMessageTypes::VEHICLE_MISSION:
        return "vehicle_mission";
    case GuiMessageTypes::VEHICLE_PATH:
        return "vehicle_path";
    case GuiMessageTypes::VEHICLE_PARAM_LIST:
        return "vehicle_param_list";
    case GuiMessageTypes::GLOBAL_ORIGIN:
        return "environment_icon";
    case GuiMessageTypes::CURRENT_MISSION_ITEM:
        return "current_mission_item";
    case GuiMessageTypes::MISSION_ITEM_REACHED:
        return "mission_item_reached";
    case GuiMessageTypes::SENSOR_FOOTPRINT:
        return "sensor_footprint";
    case GuiMessageTypes::ENVIRONMENT_BOUNDARY:
        return "environment_boundary";
    default:
        throw std::runtime_error("Unknown gui message type");
    }
}

#endif // MESSAGETYPES_H
