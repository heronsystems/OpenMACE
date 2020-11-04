#include "vehicle_object_ardupilot.h"

VehicleObject_Ardupilot::VehicleObject_Ardupilot(CommsMAVLINK* commsObj, const MaceCore::ModuleCharacteristic &module, const int &mavlinkID):
    MavlinkVehicleObject(commsObj, module, mavlinkID)
{

}
