#include "module_vehicle_ardupilot.h"

void ModuleVehicleArdupilot::MissionAcknowledgement(const MAV_MISSION_RESULT &missionResult, const bool &publishResult)
{
    UNUSED(publishResult);
    switch(missionResult) {
    case MAV_MISSION_ACCEPTED:
    {
        break;
    }
    case MAV_MISSION_ERROR:
    {
        break;
    }
    case MAV_MISSION_UNSUPPORTED_FRAME:
    {
        break;
    }
    case MAV_MISSION_UNSUPPORTED:
    {
        break;
    }
    case MAV_MISSION_NO_SPACE:
    {
        break;
    }
    case MAV_MISSION_INVALID:
    case MAV_MISSION_INVALID_PARAM1:
    case MAV_MISSION_INVALID_PARAM2:
    case MAV_MISSION_INVALID_PARAM3:
    case MAV_MISSION_INVALID_PARAM4:
    case MAV_MISSION_INVALID_PARAM5_X:
    case MAV_MISSION_INVALID_PARAM6_Y:
    case MAV_MISSION_INVALID_PARAM7:
    {
        std::cout<<"One of the parameters has an invalid value"<<std::endl;
        break;
    }
    case MAV_MISSION_DENIED:
    {
        break;
    }
    default:
    {
        break;
    }
    }
}
