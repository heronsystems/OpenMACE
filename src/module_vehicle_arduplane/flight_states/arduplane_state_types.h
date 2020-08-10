#ifndef ARDUPLANE_STATE_TYPES_H
#define ARDUPLANE_STATE_TYPES_H

#include <string>
#include <vector>
#include <stdexcept>


namespace arduplane{
namespace state {

enum class ArduplaneFlightState{
    STATE_GROUNDED = 0, /**< */
    STATE_GROUNDED_IDLE = 1, /**< */
    STATE_GROUNDED_ARMING = 2, /**< */
    STATE_GROUNDED_ARMED = 3, /**< */
    STATE_GROUNDED_DISARMING = 4, /**< */
    STATE_GROUNDED_DISARMED = 5, /**< */
    STATE_TAKEOFF = 6, /**< */
    STATE_TAKEOFF_CLIMBING = 7, /**< */
    STATE_TAKEOFF_TRANSITIONING = 8, /**< */
    STATE_TAKEOFF_COMPLETE = 9, /**< */
    STATE_FLIGHT = 10, /**< */
    STATE_FLIGHT_LOITER = 11, /**< */
    STATE_FLIGHT_MANUAL = 12, /**< */
    STATE_FLIGHT_GUIDED = 13, /**< */
    STATE_FLIGHT_GUIDED_IDLE = 14, /**< */
    STATE_FLIGHT_GUIDED_SPATIALITEM = 15, /**< */
    STATE_FLIGHT_GUIDED_QUEUE = 16, /**< */
    STATE_FLIGHT_GUIDED_ATTTARGET = 17, /**< */
    STATE_FLIGHT_GUIDED_GEOTARGET = 18, /**< */
    STATE_FLIGHT_GUIDED_CARTARGET = 19, /**< */
    STATE_FLIGHT_AUTO = 20, /**< */
    STATE_FLIGHT_BRAKE= 21, /**< */
    STATE_FLIGHT_RTL = 22, /**< */
    STATE_FLIGHT_LAND = 23,
    STATE_FLIGHT_UNKNOWN = 24,
    STATE_LANDING= 25, /**< */
    STATE_LANDING_TRANSITIONING = 26, /**< */
    STATE_LANDING_DESCENDING = 27, /**< */
    STATE_LANDING_COMPLETE = 28, /**< */
    STATE_UNKNOWN = 29 /**< */
};

//!
//! \brief CommandToString
//! \param type
//! \return
//!
inline std::string ArduplaneStateToString(const ArduplaneFlightState &type) {
    switch (type) {
    case ArduplaneFlightState::STATE_GROUNDED:
        return "Grounded";
    case ArduplaneFlightState::STATE_GROUNDED_IDLE:
        return "Grounded Idle";
    case ArduplaneFlightState::STATE_GROUNDED_ARMING:
        return "Grounded Arming";
    case ArduplaneFlightState::STATE_GROUNDED_ARMED:
        return "Grounded Armed";
    case ArduplaneFlightState::STATE_GROUNDED_DISARMING:
        return "Grounded Disarming";
    case ArduplaneFlightState::STATE_GROUNDED_DISARMED:
        return "Grounded Disarmed";
    case ArduplaneFlightState::STATE_FLIGHT:
        return "Flight";
    case ArduplaneFlightState::STATE_TAKEOFF:
        return "Flight Takeoff";
    case ArduplaneFlightState::STATE_TAKEOFF_CLIMBING:
        return "Flight Takeoff Climbing";
    case ArduplaneFlightState::STATE_TAKEOFF_TRANSITIONING:
        return "Flight Takeoff Transitioning";
    case ArduplaneFlightState::STATE_FLIGHT_MANUAL:
        return "Flight Manual";
    case ArduplaneFlightState::STATE_FLIGHT_GUIDED:
        return "Flight Guided";
    case ArduplaneFlightState::STATE_FLIGHT_AUTO:
        return "Flight Auto";
    case ArduplaneFlightState::STATE_FLIGHT_BRAKE:
        return "Flight Brake";
    case ArduplaneFlightState::STATE_FLIGHT_RTL:
        return "Flight RTL";
    case ArduplaneFlightState::STATE_LANDING:
        return "Flight Landing";
    case ArduplaneFlightState::STATE_LANDING_TRANSITIONING:
        return "Flight Landing Transitioning";
    case ArduplaneFlightState::STATE_LANDING_DESCENDING:
        return "Flight Landing Descent";
    default:
        throw std::runtime_error("Unknown arduplane state type seen");
    }
}

} //end of namespace arduplane
} //end of namespace state


#endif // ARDUPLANE_STATE_TYPES_H

