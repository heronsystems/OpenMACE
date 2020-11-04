#include "vehicle_object_arducopter.h"

VehicleObject_Arducopter::VehicleObject_Arducopter(CommsMAVLINK* commsObj, const MaceCore::ModuleCharacteristic &module, const int &mavlinkID):
    VehicleObject_Ardupilot(commsObj, module, mavlinkID)
{
    m_ArdupilotMode = new ARDUCOPTERComponent_FlightMode();
}

VehicleObject_Arducopter::~VehicleObject_Arducopter()
{
    if(m_ArdupilotMode != nullptr)
        delete m_ArdupilotMode;
}
