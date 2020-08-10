#include "arduplane_component_flight_mode.h"

ARDUPLANEComponent_FlightMode::ARDUPLANEComponent_FlightMode()
{
    availableFM = arduplaneFM;
}

int ARDUPLANEComponent_FlightMode::getFlightModeFromString(const std::string &modeString)
{
    std::map<uint8_t,std::string>::iterator it;
    int vehicleModeID = 0;
    for (it=availableFM.begin(); it != availableFM.end(); it++)
    {
        if(it->second == modeString)
        {
            vehicleModeID = it->first;
            return vehicleModeID;
        }
    }
}

void ARDUPLANEComponent_FlightMode::getAvailableFlightModes(const Data::VehicleTypes &vehicleType, std::map<int, std::string> &availableFM)
{
    UNUSED(vehicleType);
    UNUSED(availableFM);
}

std::string ARDUPLANEComponent_FlightMode::parseMAVLINK(const mavlink_heartbeat_t &msg)
{
    this->setVehicleTypeFromMAVLINK(msg.type);
    std::string newFlightMode = availableFM.at(msg.custom_mode);
    currentFM = msg.custom_mode;
    return newFlightMode;
}

void ARDUPLANEComponent_FlightMode::setVehicleTypeFromMAVLINK(const int &vehicleType)
{
        switch (vehicleType) {
        case MAV_TYPE_FIXED_WING:
        {
            this->availableFM = arduplaneFM;
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
