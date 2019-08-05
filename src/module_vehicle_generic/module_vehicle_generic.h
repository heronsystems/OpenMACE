#ifndef MODULE_VEHICLE_GENERIC_H
#define MODULE_VEHICLE_GENERIC_H

#include "module_vehicle_generic_global.h"

#include <functional>

#include "common/common.h"

#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_vehicle.h"
#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_generic_item_topic/data_generic_item_topic_components.h"

template <typename ...VehicleTopicAdditionalComponents>
class MODULE_VEHICLE_GENERICSHARED_EXPORT ModuleVehicleGeneric : public MaceCore::IModuleCommandVehicle
{
public:

    typedef Data::TopicDataObjectCollection<
    VehicleTopicAdditionalComponents...,
    DATA_GENERIC_VEHICLE_ITEM_TOPICS> VehicleDataTopicType;

    ModuleVehicleGeneric() :
        MaceCore::IModuleCommandVehicle(),
        m_VehicleDataTopic("vehicleData")
    {

    }

public:

    VehicleDataTopicType m_VehicleDataTopic;

};



#endif // MODULE_VEHICLE_GENERIC_H
