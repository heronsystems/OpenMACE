#include "arduplane_component_flight_mode.h"

ARDUPLANEComponent_FlightMode::ARDUPLANEComponent_FlightMode():
    ARDUPILOTComponent_OperatingMode()
{
    _currentMode = static_cast<uint32_t>(Arduplane_FM::APFM_UNKNOWN);
}

uint32_t ARDUPLANEComponent_FlightMode::getFlightModeFromString(const std::string &modeString) const
{
    std::map<uint32_t,std::string>::const_iterator it;
    uint32_t vehicleModeID = static_cast<uint32_t>(Arduplane_FM::APFM_UNKNOWN);
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
std::map<uint32_t, std::string> ARDUPLANEComponent_FlightMode::getAvailableFlightModes() const
{
    return arduplaneFM;
}

std::string ARDUPLANEComponent_FlightMode::parseMAVLINK(const mavlink_heartbeat_t &msg)
{
    std::string newFlightMode = arduplaneFM.at(msg.custom_mode);
    _currentMode = msg.custom_mode;
    return newFlightMode;
}
