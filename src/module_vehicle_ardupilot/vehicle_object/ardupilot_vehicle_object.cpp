#include "ardupilot_vehicle_object.h"

ArdupilotVehicleObject::ArdupilotVehicleObject(CommsMAVLINK* commsObj, const MaceCore::ModuleCharacteristic &module, const int &mavlinkID):
    MavlinkVehicleObject(commsObj, module, mavlinkID)
{

}
