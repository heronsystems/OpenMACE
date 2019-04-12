#ifndef I_MODULE_EVENTS_SENSORS_H
#define I_MODULE_EVENTS_SENSORS_H

#include "i_module_events_general.h"

namespace MaceCore
{

class IModuleEventsSensors : public IModuleEventsGeneral
{
public:
    //virtual void Event_ArmVehicle(const void* sender, const MissionItem::ActionArm &arm) = 0;
    //virtual void Event_ChangeVehicleMode(const void* sender, const MissionItem::ActionChangeMode &changeMode) = 0;


    virtual void Sensors_UpdatedMapLayer(const std::string& layerName, mace::maps::BaseGridMap* mapLayer) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_SENSORS_H
