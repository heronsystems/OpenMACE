#ifndef ARDUCOPTER_COMPONENT_FLIGHT_MODE_H
#define ARDUCOPTER_COMPONENT_FLIGHT_MODE_H

#include  "module_vehicle_ardupilot/vehicle_object/ardupilot_component_operating_mode.h"

class ARDUCOPTERComponent_FlightMode : public ARDUPILOTComponent_OperatingMode
{
public:

    /**
     * @brief The Arducopter_FM enum values are used to determine the explicit flight mode of the arducopter.
     */

    enum class Arducopter_FM : uint32_t {
        ACFM_STABILIZE =     0,  ///< manual airframe angle with manual throttle
        ACFM_ACRO =          1,  ///< manual body-frame angular rate with manual throttle
        ACFM_ALT_HOLD =      2,  ///< manual airframe angle with automatic throttle
        ACFM_AUTO =          3,  ///< fully automatic waypoint control using mission commands
        ACFM_GUIDED =        4,  ///< fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        ACFM_LOITER =        5,  ///< automatic horizontal acceleration with automatic throttle
        ACFM_RTL =           6,  ///< automatic return to launching point
        ACFM_CIRCLE =        7,  ///< automatic circular flight with automatic throttle
        ACFM_LAND =          9,  ///< automatic landing with horizontal position control
        ACFM_DRIFT =        11,  ///< semi-automous position, yaw and throttle control
        ACFM_SPORT =        13,  ///< manual earth-frame angular rate control with manual throttle
        ACFM_FLIP =         14,  ///< automatically flip the vehicle on the roll axis
        ACFM_AUTOTUNE =     15,  ///< automatically tune the vehicle's roll and pitch gains
        ACFM_POSHOLD =      16,  ///< automatic position hold with manual override, with automatic throttle
        ACFM_BRAKE =        17,  ///< full-brake using inertial/GPS system, no copter input
        ACFM_THROW =        18,  ///< throw to launch mode using inertial/GPS system, no copter input
        ACFM_AVOID_ADSB =   19,  ///< automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        ACFM_GUIDED_NOGPS = 20,  ///< guided mode but only accepts attitude and altitude
        ACFM_UNKNOWN = 21,
        ACFM_NR = 22
    };

public:
    ARDUCOPTERComponent_FlightMode();

    std::string parseMAVLINK(const mavlink_heartbeat_t &msg) override;

    uint32_t getFlightModeFromString(const std::string &modeString) const override;

    std::map<uint32_t, std::string> getAvailableFlightModes() const override;


private:
    std::map<uint32_t, std::string> arducopterFM = {{static_cast<uint32_t>(Arducopter_FM::ACFM_STABILIZE),"STABILIZE"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_ACRO),"ACRO"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_ALT_HOLD),"ALT HOLD"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_AUTO),"AUTO"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_GUIDED),"GUIDED"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_LOITER),"LOITER"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_RTL),"RTL"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_CIRCLE),"CIRCLE"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_LAND),"LAND"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_DRIFT),"DRIFT"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_SPORT),"SPORT"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_FLIP),"FLIP"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_AUTOTUNE),"AUTOTUNE"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_POSHOLD),"POSHOLD"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_BRAKE),"BRAKE"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_THROW),"THROW"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_AVOID_ADSB),"AVOID ADSB"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_GUIDED_NOGPS),"GUIDED NO GPS"},
                                               {static_cast<uint32_t>(Arducopter_FM::ACFM_UNKNOWN),"UNKNOWN"}};
};

#endif ///< ARDUPILOT_COMPONENT_FLIGHT_MODE_H
