#ifndef I_MODULE_COMMAND_GENERIC_BOUNDARIES_H
#define I_MODULE_COMMAND_GENERIC_BOUNDARIES_H

#include "mace_core_global.h"

#include "abstract_module_event_listeners.h"

namespace MaceCore
{

#define GENERIC_MODULE_BOUNDARY_LISTENER_ENUMS NEWLY_AVAILABLE_BOUNDARY \


class MACE_CORESHARED_EXPORT IModuleGenericBoundaries
{
    friend class MaceCore;
public:

    IModuleGenericBoundaries()
    {

    }

    template<typename T, typename TT, typename CT>
    void SetUp(AbstractModule_EventListeners<T, TT, CT> *ptr)
    {
        ptr->template AddCommandLogic<uint8_t>(CT::NEWLY_AVAILABLE_BOUNDARY, [this](const uint8_t &key, const OptionalParameter<ModuleCharacteristic> &sender)
        {
            NewlyAvailableBoundary(key, sender);
        });
    }

public:

    virtual void NewlyAvailableBoundary(const uint8_t &key, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>()) = 0;

};

}


#endif // I_MODULE_COMMAND_GENERIC_BOUNDARIES_H
