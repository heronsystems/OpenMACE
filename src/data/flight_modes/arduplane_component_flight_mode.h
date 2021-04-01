#ifndef ARDUPLANE_COMPONENT_FLIGHT_MODE_H
#define ARDUPLANE_COMPONENT_FLIGHT_MODE_H

#include  "ardupilot_component_operating_mode.h"

class ARDUPLANEComponent_FlightMode : public ARDUPILOTComponent_OperatingMode
{
public:
    ARDUPLANEComponent_FlightMode();

    std::string parseMAVLINK(const mavlink_heartbeat_t &msg) override;

    uint32_t getFlightModeFromString(const std::string &modeString) const override;

    static std::map<uint32_t, std::string> getAvailableFlightModes();

    static std::string getFlightModeStr(const uint8_t &mode);


private:

};

#endif // ARDUPLANE_COMPONENT_FLIGHT_MODE_H
