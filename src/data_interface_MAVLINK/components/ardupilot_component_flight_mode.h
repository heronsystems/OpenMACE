#ifndef ARDUPILOT_COMPONENT_FLIGHT_MODE_OLD_H
#define ARDUPILOT_COMPONENT_FLIGHT_MODE_OLD_H

#include <iostream>
#include <string>
#include <map>

#include "mavlink.h"
#include "common/common.h"

#include "data_generic_item_topic/data_generic_item_topic_flightmode.h"

namespace DataARDUPILOT
{

class ARDUPILOTComponent_FlightMode : public DataGenericItemTopic::DataGenericItemTopic_FlightMode
{
    enum class Arducopter_FM : uint8_t {
        ACFM_STABILIZE =     0,  // manual airframe angle with manual throttle
        ACFM_ACRO =          1,  // manual body-frame angular rate with manual throttle
        ACFM_ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        ACFM_AUTO =          3,  // fully automatic waypoint control using mission commands
        ACFM_GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        ACFM_LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        ACFM_RTL =           6,  // automatic return to launching point
        ACFM_CIRCLE =        7,  // automatic circular flight with automatic throttle
        ACFM_LAND =          9,  // automatic landing with horizontal position control
        ACFM_DRIFT =        11,  // semi-automous position, yaw and throttle control
        ACFM_SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        ACFM_FLIP =         14,  // automatically flip the vehicle on the roll axis
        ACFM_AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        ACFM_POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        ACFM_BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        ACFM_THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        ACFM_AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        ACFM_GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        ACFM_UNKNOWN = 21,
        ACFM_NR = 22
    };

    enum class Arduplane_FM : uint8_t{
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
    ARDUPILOTComponent_FlightMode();

    void parseMAVLINK(const mavlink_heartbeat_t &msg);

    void setVehicleTypeFromMAVLINK(const int &vehicleType);

    int getFlightModeFromString(const std::string &modeString);

    void getAvailableFlightModes(const Data::VehicleTypes &vehicleType, std::map<int, std::string> &availableFM);

    bool vehicleArmable();

private:
    std::map<Arducopter_FM, std::string> arducopterFM = {{Arducopter_FM::ACFM_STABILIZE,"STABILIZE"},
                                               {Arducopter_FM::ACFM_ACRO,"ACRO"},
                                               {Arducopter_FM::ACFM_ALT_HOLD,"ALT HOLD"},
                                               {Arducopter_FM::ACFM_AUTO,"AUTO"},
                                               {Arducopter_FM::ACFM_GUIDED,"GUIDED"},
                                               {Arducopter_FM::ACFM_LOITER,"LOITER"},
                                               {Arducopter_FM::ACFM_RTL,"RTL"},
                                               {Arducopter_FM::ACFM_CIRCLE,"CIRCLE"},
                                               {Arducopter_FM::ACFM_LAND,"LAND"},
                                               {Arducopter_FM::ACFM_DRIFT,"DRIFT"},
                                               {Arducopter_FM::ACFM_SPORT,"SPORT"},
                                               {Arducopter_FM::ACFM_FLIP,"FLIP"},
                                               {Arducopter_FM::ACFM_AUTOTUNE,"AUTOTUNE"},
                                               {Arducopter_FM::ACFM_POSHOLD,"POSHOLD"},
                                               {Arducopter_FM::ACFM_BRAKE,"BRAKE"},
                                               {Arducopter_FM::ACFM_THROW,"THROW"},
                                               {Arducopter_FM::ACFM_AVOID_ADSB,"AVOID ADSB"},
                                               {Arducopter_FM::ACFM_GUIDED_NOGPS,"GUIDED NO GPS"},
                                               {Arducopter_FM::ACFM_UNKNOWN,"UNKNOWN"}};

    std::map<Arduplane_FM, std::string> arduplaneFM = {{Arduplane_FM::APFM_MANUAL,"MANUAL"},
                                               {Arduplane_FM::APFM_CIRCLE,"CIRCLE"},
                                               {Arduplane_FM::APFM_STABILIZE,"STABILIZE"},
                                               {Arduplane_FM::APFM_TRAINING,"TRAINING"},
                                               {Arduplane_FM::APFM_ACRO,"ACRO"},
                                               {Arduplane_FM::APFM_FLY_BY_WIRE_A,"FBWA"},
                                               {Arduplane_FM::APFM_FLY_BY_WIRE_B,"FBWB"},
                                               {Arduplane_FM::APFM_CRUISE,"CRUISE"},
                                               {Arduplane_FM::APFM_AUTOTUNE,"AUTOTUNE"},
                                               {Arduplane_FM::APFM_AUTO,"AUTO"},
                                               {Arduplane_FM::APFM_RTL,"RTL"},
                                               {Arduplane_FM::APFM_LOITER,"LOITER"},
                                               {Arduplane_FM::APFM_AVOID_ADSB,"AVOID ADSB"},
                                               {Arduplane_FM::APFM_GUIDED,"GUIDED"},
                                               {Arduplane_FM::APFM_INITIALISING,"INITIALIZING"},
                                               {Arduplane_FM::APFM_QSTABILIZE,"QSTABILIZE"},
                                               {Arduplane_FM::APFM_QHOVER,"QHOVER"},
                                               {Arduplane_FM::APFM_QLOITER,"QLOITER"},
                                               {Arduplane_FM::APFM_QLAND,"QLAND"},
                                               {Arduplane_FM::APFM_QRTL,"QRTL"},
                                               {Arduplane_FM::APFM_UNKNOWN,"UNKNOWN"}};
private:
    std::map<Arducopter_FM,std::string> availableFM;
};

} //end of namespace DataArdupilot

#endif // ARDUPILOT_COMPONENT_FLIGHT_MODE_OLD_H
