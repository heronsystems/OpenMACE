#include "arducopter_component_flight_mode.h"

ARDUCOPTERComponent_FlightMode::ARDUCOPTERComponent_FlightMode():
    ARDUPILOTComponent_OperatingMode()
{
    _currentMode = static_cast<uint32_t>(Arducopter_FM::ACFM_UNKNOWN);
}

uint32_t ARDUCOPTERComponent_FlightMode::getFlightModeFromString(const std::string &modeString) const
{
    std::map<uint32_t,std::string>::const_iterator it;
    uint32_t vehicleModeID = static_cast<uint32_t>(Arducopter_FM::ACFM_UNKNOWN);
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
std::map<uint32_t, std::string> ARDUCOPTERComponent_FlightMode::getAvailableFlightModes() const
{
    return arducopterFM;
}

std::string ARDUCOPTERComponent_FlightMode::parseMAVLINK(const mavlink_heartbeat_t &msg)
{
    std::string newFlightMode = arducopterFM.at(msg.custom_mode);
    _currentMode = msg.custom_mode;
    return newFlightMode;
}
