#include "arducopter_component_flight_mode.h"

ARDUCOPTERComponent_FlightMode::ARDUCOPTERComponent_FlightMode():
    ARDUPILOTComponent_OperatingMode()
{
    _currentMode = static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_ENUM_END);
}

uint32_t ARDUCOPTERComponent_FlightMode::getFlightModeFromString(const std::string &modeString) const
{
    std::map<uint32_t,std::string>::const_iterator it;
    uint32_t vehicleModeID = static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_ENUM_END);
    std::map<uint32_t, std::string> arducopterFM = getAvailableFlightModes();
    for (it=arducopterFM.begin(); it != arducopterFM.end(); it++)
    {
        if(it->second == modeString)
        {
            vehicleModeID = static_cast<uint32_t>(it->first);
            return vehicleModeID;
        }
    }
    return vehicleModeID;
}
std::map<uint32_t, std::string> ARDUCOPTERComponent_FlightMode::getAvailableFlightModes()
{
    std::map<uint32_t, std::string> arducopterFM = {{static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_STABILIZE),"STABILIZE"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_ACRO),"ACRO"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_ALT_HOLD),"ALT HOLD"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_AUTO),"AUTO"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_GUIDED),"GUIDED"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_LOITER),"LOITER"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_RTL),"RTL"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_CIRCLE),"CIRCLE"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_LAND),"LAND"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_DRIFT),"DRIFT"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_SPORT),"SPORT"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_FLIP),"FLIP"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_AUTOTUNE),"AUTOTUNE"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_POSHOLD),"POSHOLD"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_BRAKE),"BRAKE"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_THROW),"THROW"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_AVOID_ADSB),"AVOID ADSB"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_GUIDED_NOGPS),"GUIDED NO GPS"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_SMART_RTL),"SMART RTL"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_FLOWHOLD),"SMART RTL"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_FOLLOW),"FOLLOW"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_ZIGZAG),"ZIGZAG"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_SYSTEMID),"SYSTEMID"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_AUTOROTATE),"AUTORATE"},
                                               {static_cast<uint32_t>(COPTER_MODE::COPTER_MODE_ENUM_END),"UNKNOWN"}};
    return arducopterFM;
}

std::string ARDUCOPTERComponent_FlightMode::parseMAVLINK(const mavlink_heartbeat_t &msg)
{
    std::map<uint32_t, std::string> arduplaneFM = getAvailableFlightModes();
    std::string newFlightMode = arduplaneFM.at(msg.custom_mode);
    _currentMode = msg.custom_mode;
    return newFlightMode;
}

std::string ARDUCOPTERComponent_FlightMode::getFlightModeStr(const uint8_t &mode)
{
    std::map<uint32_t, std::string> arduplaneFM = getAvailableFlightModes();
    return arduplaneFM.at(mode);
}
