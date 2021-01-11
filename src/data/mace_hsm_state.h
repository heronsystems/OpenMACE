#ifndef MACE_HSM_STATE_H
#define MACE_HSM_STATE_H

#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>


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
    STATE_FLIGHT_AUTO = 12, /**< */
    STATE_FLIGHT_BRAKE= 13, /**< */
    STATE_FLIGHT_LAND = 14, /**< */
    STATE_FLIGHT_LOITER = 15, /**< */
    STATE_FLIGHT_MANUAL = 16, /**< */
    STATE_FLIGHT_RTL = 17, /**< */
    STATE_FLIGHT_UNKNOWN = 18, /**< */

    STATE_LANDING= 20, /**< */
    STATE_LANDING_TRANSITIONING = 21, /**< */
    STATE_LANDING_DESCENDING = 22, /**< */
    STATE_LANDING_COMPLETE = 23, /**< */

    STATE_FLIGHT_GUIDED = 30, /**< */
    STATE_FLIGHT_GUIDED_IDLE = 31, /**< */
    STATE_FLIGHT_GUIDED_SPATIALITEM = 32, /**< */
    STATE_FLIGHT_GUIDED_QUEUE = 33, /**< */
    STATE_FLIGHT_GUIDED_ATTTARGET = 34, /**< */
    STATE_FLIGHT_GUIDED_GEOTARGET = 35, /**< */
    STATE_FLIGHT_GUIDED_CARTARGET = 36, /**< */

    STATE_FLIGHT_AI = 40, /**< */
    STATE_FLIGHT_AI_ABORT = 41, /**< */
    STATE_FLIGHT_AI_EXECUTE= 42, /**< */
    STATE_FLIGHT_AI_EXECUTE_ABORT= 43, /**< */
    STATE_FLIGHT_AI_EXECUTE_DEFLECTION = 44, /**< */
    STATE_FLIGHT_AI_EXECUTE_END= 45, /**< */
    STATE_FLIGHT_AI_INITIALIZE = 46, /**< */
    STATE_FLIGHT_AI_INITIALIZE_ABORT = 47, /**< */
    STATE_FLIGHT_AI_INITIALIZE_ROUTE = 48, /**< */

    STATE_UNKNOWN = 254, /**< */
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
    case MACEHSMState::STATE_TAKEOFF:
        return "Flight Takeoff";
    case MACEHSMState::STATE_TAKEOFF_CLIMBING:
        return "Flight Takeoff Climbing";
    case MACEHSMState::STATE_TAKEOFF_TRANSITIONING:
        return "Flight Takeoff Transitioning";
    case MACEHSMState::STATE_TAKEOFF_COMPLETE:
        return "Flight Takeoff Complete";

    case MACEHSMState::STATE_FLIGHT:
        return "Flight";
    case MACEHSMState::STATE_FLIGHT_AUTO:
        return "Flight Auto";
    case MACEHSMState::STATE_FLIGHT_BRAKE:
        return "Flight Brake";
    case MACEHSMState::STATE_FLIGHT_LAND:
        return "Flight Land";
    case MACEHSMState::STATE_FLIGHT_LOITER:
        return "Flight Loiter";
    case MACEHSMState::STATE_FLIGHT_MANUAL:
        return "Flight Manual";
    case MACEHSMState::STATE_FLIGHT_RTL:
        return "Flight RTL";
    case MACEHSMState::STATE_FLIGHT_UNKNOWN:
        return "Flight Unknown";

    case MACEHSMState::STATE_LANDING:
        return "Flight Landing";
    case MACEHSMState::STATE_LANDING_TRANSITIONING:
        return "Flight Landing Transitioning";
    case MACEHSMState::STATE_LANDING_DESCENDING:
        return "Flight Landing Descent";
    case MACEHSMState::STATE_LANDING_COMPLETE:
        return "Flight Landing Complete";

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

    case MACEHSMState::STATE_FLIGHT_AI:
        return "Flight AI";
    case MACEHSMState::STATE_FLIGHT_AI_ABORT:
        return "Flight AI Abort";
    case MACEHSMState::STATE_FLIGHT_AI_EXECUTE:
        return "Flight AI Execute";
    case MACEHSMState::STATE_FLIGHT_AI_EXECUTE_ABORT:
        return "Flight AI Execute Abort";
    case MACEHSMState::STATE_FLIGHT_AI_EXECUTE_DEFLECTION:
        return "Flight AI Execute Deflection";
    case MACEHSMState::STATE_FLIGHT_AI_EXECUTE_END:
        return "Flight AI Execute End";
    case MACEHSMState::STATE_FLIGHT_AI_INITIALIZE:
        return "Flight AI Initialize";
    case MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ABORT:
        return "Flight AI Initialize Abort";
    case MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ROUTE:
        return "Flight AI Initialize Route";
        
    case MACEHSMState::STATE_UNKNOWN:
        return "Unknown";
    default:
        throw std::runtime_error("Unknown MACE HSM state type seen");
        break;
    }
}

inline MACEHSMState MACEHSMStateFromString(const std::string &str) {
    if(str == "Grounded")
        return MACEHSMState::STATE_GROUNDED;
    if(str == "Grounded Idle")
        return MACEHSMState::STATE_GROUNDED_IDLE;
    if(str == "Grounded Arming")
        return MACEHSMState::STATE_GROUNDED_ARMING;
    if(str == "Grounded Armed")
        return MACEHSMState::STATE_GROUNDED_ARMED;
    if(str == "Grounded Disarming")
        return MACEHSMState::STATE_GROUNDED_DISARMING;
    if(str == "Grounded Disarmed")
        return MACEHSMState::STATE_GROUNDED_DISARMED;
    if(str == "Flight")
        return MACEHSMState::STATE_FLIGHT;
    if(str == "Flight Takeoff")
        return MACEHSMState::STATE_TAKEOFF;
    if(str == "Flight Takeoff Climbing")
        return MACEHSMState::STATE_TAKEOFF_CLIMBING;
    if(str == "Flight Takeoff Transitioning")
        return MACEHSMState::STATE_TAKEOFF_TRANSITIONING;
    if(str == "Flight Takeoff Complete")
        return MACEHSMState::STATE_TAKEOFF_COMPLETE;
    if(str == "Flight Manual")
        return MACEHSMState::STATE_FLIGHT_MANUAL;
    if(str == "Flight Guided")
        return MACEHSMState::STATE_FLIGHT_GUIDED;
    if(str == "Flight Guided Idle")
        return MACEHSMState::STATE_FLIGHT_GUIDED_IDLE;
    if(str == "Flight Guided Spatial Item")
        return MACEHSMState::STATE_FLIGHT_GUIDED_SPATIALITEM;
    if(str == "Flight Guided Queue")
        return MACEHSMState::STATE_FLIGHT_GUIDED_QUEUE;
    if(str == "Flight Guided AttTarget")
        return MACEHSMState::STATE_FLIGHT_GUIDED_ATTTARGET;
    if(str == "Flight Guided GeoTarget")
        return MACEHSMState::STATE_FLIGHT_GUIDED_GEOTARGET;
    if(str == "Flight Guided CartTarget")
        return MACEHSMState::STATE_FLIGHT_GUIDED_CARTARGET;
    if(str == "Flight Auto")
        return MACEHSMState::STATE_FLIGHT_AUTO;
    if(str == "Flight Brake")
        return MACEHSMState::STATE_FLIGHT_BRAKE;
    if(str == "Flight RTL")
        return MACEHSMState::STATE_FLIGHT_RTL;
    if(str == "Flight Loiter")
        return MACEHSMState::STATE_FLIGHT_LOITER;
    if(str == "Flight Brake")
        return MACEHSMState::STATE_FLIGHT_BRAKE;
    if(str == "Flight Land")
        return MACEHSMState::STATE_FLIGHT_LAND;
    if(str == "Flight Landing")
        return MACEHSMState::STATE_LANDING;
    if(str == "Flight Landing Transitioning")
        return MACEHSMState::STATE_LANDING_TRANSITIONING;
    if(str == "Flight Landing Descent")
        return MACEHSMState::STATE_LANDING_DESCENDING;
    if(str == "Flight Landing Complete")
        return MACEHSMState::STATE_LANDING_COMPLETE;
    if(str == "Flight AI")
        return MACEHSMState::STATE_FLIGHT_AI;
    if(str == "Flight AI Abort")
        return MACEHSMState::STATE_FLIGHT_AI_ABORT;
    if(str == "Flight AI Execute")
        return MACEHSMState::STATE_FLIGHT_AI_EXECUTE;
    if(str == "Flight AI Execute Abort")
        return MACEHSMState::STATE_FLIGHT_AI_EXECUTE_ABORT;
    if(str == "Flight AI Execute Deflection")
        return MACEHSMState::STATE_FLIGHT_AI_EXECUTE_DEFLECTION;
    if(str == "Flight AI Execute End")
        return MACEHSMState::STATE_FLIGHT_AI_EXECUTE_END;
    if(str == "Flight AI Initialize")
        return MACEHSMState::STATE_FLIGHT_AI_INITIALIZE;
    if(str == "Flight AI Initialize Abort")
        return MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ABORT;
    if(str == "Flight AI Initialize Route")
        return MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ROUTE;
    if(str == "Flight Unknown")
        return MACEHSMState::STATE_FLIGHT_UNKNOWN;
    if(str == "Unknown")
        return MACEHSMState::STATE_UNKNOWN;
    throw std::runtime_error("Unknown MACE HSM state string seen");

}
} //end of namespace Data

#endif // MACE_HSM_STATE_H
