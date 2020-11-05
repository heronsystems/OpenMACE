#ifndef MACE_HSM_STATE_H
#define MACE_HSM_STATE_H

#include <string>
#include <vector>
#include <stdexcept>


namespace Data
{

enum class MACEHSMState {
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


inline std::string MACEHSMStateToString(const MACEHSMState &type) {
    switch (type) {
    case MACEHSMState::STATE_GROUNDED:
        return "Grounded";
    case MACEHSMState::STATE_GROUNDED_IDLE:
        return "Grounded Idle";
    case MACEHSMState::STATE_GROUNDED_ARMING:
        return "Grounded Arming";
    case MACEHSMState::STATE_GROUNDED_ARMED:
        return "Grounded Armed";
    case MACEHSMState::STATE_GROUNDED_DISARMING:
        return "Grounded Disarming";
    case MACEHSMState::STATE_GROUNDED_DISARMED:
        return "Grounded Disarmed";
    case MACEHSMState::STATE_FLIGHT:
        return "Flight";
    case MACEHSMState::STATE_TAKEOFF:
        return "Flight Takeoff";
    case MACEHSMState::STATE_TAKEOFF_CLIMBING:
        return "Flight Takeoff Climbing";
    case MACEHSMState::STATE_TAKEOFF_TRANSITIONING:
        return "Flight Takeoff Transitioning";
    case MACEHSMState::STATE_TAKEOFF_COMPLETE:
        return "Flight Takeoff Complete";
    case MACEHSMState::STATE_FLIGHT_MANUAL:
        return "Flight Manual";
    case MACEHSMState::STATE_FLIGHT_GUIDED:
        return "Flight Guided";
    case MACEHSMState::STATE_FLIGHT_GUIDED_IDLE:
        return "Flight Guided Idle";
    case MACEHSMState::STATE_FLIGHT_GUIDED_SPATIALITEM:
        return "Flight Guided Spatial Item";
    case MACEHSMState::STATE_FLIGHT_GUIDED_QUEUE:
        return "Flight Guided Queue";
    case MACEHSMState::STATE_FLIGHT_GUIDED_ATTTARGET:
        return "Flight Guided AttTarget";
    case MACEHSMState::STATE_FLIGHT_GUIDED_GEOTARGET:
        return "Flight Guided GeoTarget";
    case MACEHSMState::STATE_FLIGHT_GUIDED_CARTARGET:
        return "Flight Guided CartTarget";
    case MACEHSMState::STATE_FLIGHT_AUTO:
        return "Flight Auto";
    case MACEHSMState::STATE_FLIGHT_BRAKE:
        return "Flight Brake";
    case MACEHSMState::STATE_FLIGHT_RTL:
        return "Flight RTL";
    case MACEHSMState::STATE_FLIGHT_LOITER:
        return "Flight Loiter";
    case MACEHSMState::STATE_FLIGHT_LAND:
        return "Flight Land";
    case MACEHSMState::STATE_LANDING:
        return "Flight Landing";
    case MACEHSMState::STATE_LANDING_TRANSITIONING:
        return "Flight Landing Transitioning";
    case MACEHSMState::STATE_LANDING_DESCENDING:
        return "Flight Landing Descent";
    case MACEHSMState::STATE_LANDING_COMPLETE:
        return "Flight Landing Complete";
    case MACEHSMState::STATE_FLIGHT_UNKNOWN:
        return "Flight Unknown";
    case MACEHSMState::STATE_UNKNOWN:
        return "Unknown";
    default:
        throw std::runtime_error("Unknown MACE HSM state type seen");
    }
}

} //end of namespace Data

#endif // MACE_HSM_STATE_H
