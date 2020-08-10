#include "arducopter_vehicle_object.h"

ArducopterVehicleObject::ArducopterVehicleObject(CommsMAVLINK* commsObj, const MaceCore::ModuleCharacteristic &module, const int &mavlinkID):
    MavlinkVehicleObject(commsObj, module, mavlinkID)
{

}
