#ifndef ARDUPLANE_COMPONENT_FLIGHT_MODE_H
#define ARDUPLANE_COMPONENT_FLIGHT_MODE_H

#include  "module_vehicle_ardupilot/vehicle_object/ardupilot_component_operating_mode.h"

class ARDUPLANEComponent_FlightMode : public ARDUPILOTComponent_OperatingMode
{
private:
    enum class Arduplane_FM : uint32_t{
        APFM_MANUAL        = 0,
        APFM_CIRCLE        = 1,
        APFM_STABILIZE     = 2,
        APFM_TRAINING      = 3,
        APFM_ACRO          = 4,
        APFM_FLY_BY_WIRE_A = 5,
        APFM_FLY_BY_WIRE_B = 6,
        APFM_CRUISE        = 7,
        APFM_AUTOTUNE      = 8,
        APFM_AUTO          = 10,
        APFM_RTL           = 11,
        APFM_LOITER        = 12,
        APFM_AVOID_ADSB    = 14,
        APFM_TAKEOFF       = 13,
        APFM_GUIDED        = 15,
        APFM_INITIALISING  = 16,
        APFM_QSTABILIZE    = 17,
        APFM_QHOVER        = 18,
        APFM_QLOITER       = 19,
        APFM_QLAND         = 20,
        APFM_QRTL          = 21,
        APFM_UNKNOWN = 22,
        APFM_NR = 23,
    };

public:
    ARDUPLANEComponent_FlightMode();

    std::string parseMAVLINK(const mavlink_heartbeat_t &msg) override;

    uint32_t getFlightModeFromString(const std::string &modeString) const override;

    std::map<uint32_t, std::string> getAvailableFlightModes() const override;


private:
    std::map<uint32_t, std::string> arduplaneFM = {{static_cast<uint32_t>(Arduplane_FM::APFM_MANUAL),"MANUAL"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_CIRCLE),"CIRCLE"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_STABILIZE),"STABILIZE"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_TRAINING),"TRAINING"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_ACRO),"ACRO"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_FLY_BY_WIRE_A),"FBWA"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_FLY_BY_WIRE_B),"FBWB"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_CRUISE),"CRUISE"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_AUTOTUNE),"AUTOTUNE"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_TAKEOFF),"TAKEOFF"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_AUTO),"AUTO"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_RTL),"RTL"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_LOITER),"LOITER"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_AVOID_ADSB),"AVOID ADSB"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_GUIDED),"GUIDED"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_INITIALISING),"INITIALIZING"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_QSTABILIZE),"QSTABILIZE"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_QHOVER),"QHOVER"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_QLOITER),"QLOITER"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_QLAND),"QLAND"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_QRTL),"QRTL"},
                                               {static_cast<uint32_t>(Arduplane_FM::APFM_UNKNOWN),"UNKNOWN"}};
};

#endif // ARDUPLANE_COMPONENT_FLIGHT_MODE_H
