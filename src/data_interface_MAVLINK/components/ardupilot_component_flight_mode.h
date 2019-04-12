#ifndef ARDUPILOT_COMPONENT_FLIGHT_MODE_OLD_H
#define ARDUPILOT_COMPONENT_FLIGHT_MODE_OLD_H


#include <iostream>
#include <map>
#include <list>
#include <string>
#include "common/common.h"

#include "mavlink.h"

#include "data_generic_item_topic/data_generic_item_topic_flightmode.h"

namespace DataARDUPILOT
{

class ARDUPILOTComponent_FlightMode : public DataGenericItemTopic::DataGenericItemTopic_FlightMode
{
    enum class Arducopter_FM {
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

    enum class Arduplane_FM {
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
    std::map<int, std::string> arducopterFM = {{(int)Arducopter_FM::ACFM_STABILIZE,"STABILIZE"},
                                               {(int)Arducopter_FM::ACFM_ACRO,"ACRO"},
                                               {(int)Arducopter_FM::ACFM_ALT_HOLD,"ALT HOLD"},
                                               {(int)Arducopter_FM::ACFM_AUTO,"AUTO"},
                                               {(int)Arducopter_FM::ACFM_GUIDED,"GUIDED"},
                                               {(int)Arducopter_FM::ACFM_LOITER,"LOITER"},
                                               {(int)Arducopter_FM::ACFM_RTL,"RTL"},
                                               {(int)Arducopter_FM::ACFM_CIRCLE,"CIRCLE"},
                                               {(int)Arducopter_FM::ACFM_LAND,"LAND"},
                                               {(int)Arducopter_FM::ACFM_DRIFT,"DRIFT"},
                                               {(int)Arducopter_FM::ACFM_SPORT,"SPORT"},
                                               {(int)Arducopter_FM::ACFM_FLIP,"FLIP"},
                                               {(int)Arducopter_FM::ACFM_AUTOTUNE,"AUTOTUNE"},
                                               {(int)Arducopter_FM::ACFM_POSHOLD,"POSHOLD"},
                                               {(int)Arducopter_FM::ACFM_BRAKE,"BRAKE"},
                                               {(int)Arducopter_FM::ACFM_THROW,"THROW"},
                                               {(int)Arducopter_FM::ACFM_AVOID_ADSB,"AVOID ADSB"},
                                               {(int)Arducopter_FM::ACFM_GUIDED_NOGPS,"GUIDED NO GPS"},
                                               {(int)Arducopter_FM::ACFM_UNKNOWN,"UNKNOWN"}};

    std::map<int, std::string> arduplaneFM = {{(int)Arduplane_FM::APFM_MANUAL,"MANUAL"},
                                               {(int)Arduplane_FM::APFM_CIRCLE,"CIRCLE"},
                                               {(int)Arduplane_FM::APFM_STABILIZE,"STABILIZE"},
                                               {(int)Arduplane_FM::APFM_TRAINING,"TRAINING"},
                                               {(int)Arduplane_FM::APFM_ACRO,"ACRO"},
                                               {(int)Arduplane_FM::APFM_FLY_BY_WIRE_A,"FBWA"},
                                               {(int)Arduplane_FM::APFM_FLY_BY_WIRE_B,"FBWB"},
                                               {(int)Arduplane_FM::APFM_CRUISE,"CRUISE"},
                                               {(int)Arduplane_FM::APFM_AUTOTUNE,"AUTOTUNE"},
                                               {(int)Arduplane_FM::APFM_AUTO,"AUTO"},
                                               {(int)Arduplane_FM::APFM_RTL,"RTL"},
                                               {(int)Arduplane_FM::APFM_LOITER,"LOITER"},
                                               {(int)Arduplane_FM::APFM_AVOID_ADSB,"AVOID ADSB"},
                                               {(int)Arduplane_FM::APFM_GUIDED,"GUIDED"},
                                               {(int)Arduplane_FM::APFM_INITIALISING,"INITIALIZING"},
                                               {(int)Arduplane_FM::APFM_QSTABILIZE,"QSTABILIZE"},
                                               {(int)Arduplane_FM::APFM_QHOVER,"QHOVER"},
                                               {(int)Arduplane_FM::APFM_QLOITER,"QLOITER"},
                                               {(int)Arduplane_FM::APFM_QLAND,"QLAND"},
                                               {(int)Arduplane_FM::APFM_QRTL,"QRTL"},
                                               {(int)Arduplane_FM::APFM_UNKNOWN,"UNKNOWN"}};
private:
    std::map<int,std::string> availableFM;
};

} //end of namespace DataArdupilot

#endif // ARDUPILOT_COMPONENT_FLIGHT_MODE_OLD_H
