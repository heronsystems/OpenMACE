#include "ardupilot_general_controller.h"

Ardupilot_GeneralController::Ardupilot_GeneralController(std::shared_ptr<DataInterface_MAVLINK::VehicleObject_MAVLINK> vehicleData):
    vehicleDataObject(vehicleData), mToExit(false)
{
    std::cout << "Constructor on general controller" << std::endl;
}

void Ardupilot_GeneralController::terminateObject()
{
    mToExit = true;
}
