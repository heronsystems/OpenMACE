#include "ardupilot_component_flight_mode.h"

namespace DataARDUPILOT
{

ARDUPILOTComponent_FlightMode::ARDUPILOTComponent_FlightMode()
{
    availableFM = arducopterFM;
}

int ARDUPILOTComponent_FlightMode::getFlightModeFromString(const std::string &modeString)
{
    std::map<Arducopter_FM,std::string>::iterator it;
    int vehicleModeID = 0;
    for (it=availableFM.begin(); it != availableFM.end(); it++)
    {
        if(it->second == modeString)
        {
            vehicleModeID = static_cast<uint8_t>(it->first);
            break;
        }
    }
    return vehicleModeID;
}

void ARDUPILOTComponent_FlightMode::getAvailableFlightModes(const Data::VehicleTypes &vehicleType, std::map<int, std::string> &availableFM)
{
    UNUSED(vehicleType);
    UNUSED(availableFM);
}

bool ARDUPILOTComponent_FlightMode::vehicleArmable()
{
    if((flightModeString == "STABILIZE") || (flightModeString == "LOITER"))
        return true;
    return false;
}

void ARDUPILOTComponent_FlightMode::parseMAVLINK(const mavlink_heartbeat_t &msg)
{
    this->setVehicleTypeFromMAVLINK(msg.type);
    std::string newFlightMode = availableFM.at(static_cast<Arducopter_FM>(msg.custom_mode));
    this->setFlightMode(newFlightMode);
}

void ARDUPILOTComponent_FlightMode::setVehicleTypeFromMAVLINK(const int &vehicleType)
{
        switch (vehicleType) {
        case MAV_TYPE_FIXED_WING:
        {
            //this->availableFM = arduplaneFM;
            break;
        }
        case MAV_TYPE_TRICOPTER:
        case MAV_TYPE_QUADROTOR:
        case MAV_TYPE_HEXAROTOR:
        case MAV_TYPE_OCTOROTOR:
        {
            this->availableFM = arducopterFM;
            break;
        }
        default:
            this->availableFM = arducopterFM;
            break;
        }
}
} //end of namespace DataARDUPILOT
