#ifndef ARDUCOPTER_COMPONENT_FLIGHT_MODE_H
#define ARDUCOPTER_COMPONENT_FLIGHT_MODE_H

#include  "module_vehicle_ardupilot/vehicle_object/ardupilot_component_operating_mode.h"

class ARDUCOPTERComponent_FlightMode : public ARDUPILOTComponent_OperatingMode
{
public:

public:
    ARDUCOPTERComponent_FlightMode();

    std::string parseMAVLINK(const mavlink_heartbeat_t &msg) override;

    uint32_t getFlightModeFromString(const std::string &modeString) const override;

    std::map<uint32_t, std::string> getAvailableFlightModes() const override;

private:
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
};

#endif ///< ARDUPILOT_COMPONENT_FLIGHT_MODE_H
