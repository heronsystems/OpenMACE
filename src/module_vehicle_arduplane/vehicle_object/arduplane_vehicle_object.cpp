#include "arduplane_vehicle_object.h"

ArduplaneVehicleObject::ArduplaneVehicleObject(CommsMAVLINK* commsObj, const MaceCore::ModuleCharacteristic &module, const int &mavlinkID):
    MavlinkVehicleObject(commsObj, module, mavlinkID)
{

}
