#ifndef I_MODULE_COMAND_GENERIC_VEHICLE_H
#define I_MODULE_COMAND_GENERIC_VEHICLE_H

#include "mace_core_global.h"

#include "abstract_module_event_listeners.h"

namespace MaceCore
{

#define GENERIC_MODULE_VEHICLE_LISTENER_ENUMS NEWLY_AVAILABLE_VEHICLE \


//!
//! \brief Defines shared behavior for commanding modules to listen for vehicle events
//!
class MACE_CORESHARED_EXPORT IModuleGenericVehicleListener
{
    friend class MaceCore;
public:

    IModuleGenericVehicleListener()
    {

    }

    template<typename T, typename TT, typename CT>
    void SetUp(AbstractModule_EventListeners<T, TT, CT> *ptr)
    {

        ptr->template AddCommandLogic<int>(CT::NEWLY_AVAILABLE_VEHICLE, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableVehicle(vehicleID, sender);
        });
    }

public:

    virtual void NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>()) = 0;

};

}

#endif // I_MODULE_COMAND_GENERIC_VEHICLE_H
