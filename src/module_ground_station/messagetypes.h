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
    VEHICLE_VELOCITY,
    VEHICLE_ROTATION,
    VEHICLE_CONTROL,
    VEHICLE_FUEL,
    VEHICLE_MODE,
    VEHICLE_TEXT,
    VEHICLE_GPS,
    VEHICLE_ARM,
    VEHICLE_HEARTBEAT,
    VEHICLE_MISSION,
    VEHICLE_PATH,
    VEHICLE_PARAM_LIST,
    VEHICLE_TRACKANGLE,
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
    case GuiMessageTypes::VEHICLE_VELOCITY:
        return "vehicle_translational_velocity";
    case GuiMessageTypes::VEHICLE_ROTATION:
        return "vehicle_rotational_velocity";
    case GuiMessageTypes::VEHICLE_FUEL:
        return "vehicle_fuel";
    case GuiMessageTypes::VEHICLE_CONTROL:
        return "vehicle_control";
    case GuiMessageTypes::VEHICLE_TRACKANGLE:
        return "vehicle_trackangle";
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

inline  GuiMessageTypes StringtoGuiMessage(const std::string &type) {
    if(type == "vehicle_target"){
        return GuiMessageTypes::VEHICLE_TARGET;
    } else if(type == "vehicle_position"){
        return GuiMessageTypes::VEHICLE_POSITION;
    } else if(type == "vehicle_attitude"){
        return GuiMessageTypes::VEHICLE_ATTITUDE;
    } else if(type == "vehicle_airspeed"){
        return  GuiMessageTypes::VEHICLE_AIRSPEED;
    } else if(type == "vehicle_translational_velocity"){
        return  GuiMessageTypes::VEHICLE_VELOCITY;
    } else if(type == "vehicle_rotational_velocity"){
        return  GuiMessageTypes::VEHICLE_ROTATION;
    } else if(type == "vehicle_fuel"){
        return GuiMessageTypes::VEHICLE_FUEL;
    } else if(type == "vehicle_control"){
        return GuiMessageTypes::VEHICLE_CONTROL;
    } else if(type == "vehicle_trackangle"){
        return GuiMessageTypes::VEHICLE_TRACKANGLE;
    } else if(type == "vehicle_mode"){
        return GuiMessageTypes::VEHICLE_MODE;
    } else if(type == "environment_icon"){
        return GuiMessageTypes::VEHICLE_HOME;
    } else if(type == "vehicle_text"){
        return GuiMessageTypes::VEHICLE_TEXT;
    } else if(type == "vehicle_gps"){
        return GuiMessageTypes::VEHICLE_GPS;
    } else if(type == "vehicle_arm"){
        return GuiMessageTypes::VEHICLE_ARM;
    } else if(type == "vehicle_path"){
        return GuiMessageTypes::VEHICLE_PATH;
    } else if(type == "vehicle_heartbeat"){
        return GuiMessageTypes::VEHICLE_HEARTBEAT;
    } else {
//        throw std::runtime_error("Unknown gui message type");
        return GuiMessageTypes::ENVIRONMENT_BOUNDARY;
    }
}

#endif // MESSAGETYPES_H
