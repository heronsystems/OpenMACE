#include "arduplane_component_flight_mode.h"

ARDUPLANEComponent_FlightMode::ARDUPLANEComponent_FlightMode():
    ARDUPILOTComponent_OperatingMode()
{
    _currentMode = static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_ENUM_END);
}

uint32_t ARDUPLANEComponent_FlightMode::getFlightModeFromString(const std::string &modeString) const
{
    std::map<uint32_t,std::string>::const_iterator it;
    uint32_t vehicleModeID = static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_ENUM_END);
    std::map<uint32_t, std::string> arduplaneFM = getAvailableFlightModes();
    for (it=arduplaneFM.begin(); it != arduplaneFM.end(); it++)
    {
        if(it->second == modeString)
        {
            vehicleModeID = static_cast<uint32_t>(it->first);
            return vehicleModeID;
        }
    }
    return vehicleModeID;
}
std::map<uint32_t, std::string> ARDUPLANEComponent_FlightMode::getAvailableFlightModes()
{
    std::map<uint32_t, std::string> arduplaneFM = {{static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_MANUAL),"MANUAL"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_CIRCLE),"CIRCLE"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_STABILIZE),"STABILIZE"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_TRAINING),"TRAINING"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_ACRO),"ACRO"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_FLY_BY_WIRE_A),"FBWA"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_FLY_BY_WIRE_B),"FBWB"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_CRUISE),"CRUISE"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_AUTOTUNE),"AUTOTUNE"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_TAKEOFF),"TAKEOFF"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_AUTO),"AUTO"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_RTL),"RTL"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_LOITER),"LOITER"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_AVOID_ADSB),"AVOID ADSB"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_GUIDED),"GUIDED"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_INITIALIZING),"INITIALIZING"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_QSTABILIZE),"QSTABILIZE"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_QHOVER),"QHOVER"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_QLOITER),"QLOITER"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_QLAND),"QLAND"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_QRTL),"QRTL"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_QAUTOTUNE),"QAUTOTUNE"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_QACRO),"QACRO"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_THERMAL),"THERMAL"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_AI_DEFLECTION),"AI_DEFL"},
                                               {static_cast<uint32_t>(PLANE_MODE::PLANE_MODE_INITIALIZING),"INITIALIZING"}};
    return arduplaneFM;
}

std::string ARDUPLANEComponent_FlightMode::parseMAVLINK(const mavlink_heartbeat_t &msg)
{
    std::string newFlightMode = getAvailableFlightModes().at(msg.custom_mode);
    _currentMode = msg.custom_mode;
    return newFlightMode;
}

std::string ARDUPLANEComponent_FlightMode::getFlightModeStr(const uint8_t &mode)
{
    std::string rtnStr = "UNSUPPORTED";
    try {
        rtnStr = getAvailableFlightModes().at(mode);
    }
    catch (const std::out_of_range& oor) {
        // std::cout << "Accessing unknown flight mode: " << mode << std::endl;
        rtnStr = "UNSUPPORTED";
    }

    return rtnStr;
}
