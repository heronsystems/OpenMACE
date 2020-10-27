#ifndef I_ML_STATION_H
#define I_ML_STATION_H

#include "abstract_module_event_listeners.h"
#include "metadata_ml_station.h"

#include "i_module_topic_events.h"
#include "i_module_events_ml_station.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_item/data_generic_item_components.h"

#include "base/pose/cartesian_position_3D.h"


#include "i_module_command_generic_boundaries.h"
#include "i_module_command_generic_vehicle_listener.h"

namespace MaceCore
{

enum class MLStationCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    GENERIC_MODULE_BOUNDARY_LISTENER_ENUMS,
    GENERIC_MODULE_VEHICLE_LISTENER_ENUMS,
    NEWLY_AVAILABLE_VEHICLE_PARAMETERS
};

class MACE_CORESHARED_EXPORT IModuleCommandMLStation :
        public AbstractModule_EventListeners<Metadata_MLStation, IModuleEventsMLStation, MLStationCommands>,
        public IModuleGenericBoundaries,
        public IModuleGenericVehicleListener
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandMLStation():
        AbstractModule_EventListeners()
    {
        IModuleGenericBoundaries::SetUp<Metadata_MLStation, IModuleEventsMLStation, MLStationCommands>(this);
        IModuleGenericVehicleListener::SetUp<Metadata_MLStation, IModuleEventsMLStation, MLStationCommands>(this);



        AddCommandLogic<std::map<std::string, DataGenericItem::DataGenericItem_ParamValue>>(MLStationCommands::NEWLY_AVAILABLE_VEHICLE_PARAMETERS, [this](const std::map<std::string, DataGenericItem::DataGenericItem_ParamValue> &paramValues, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableParameterList(paramValues, sender);
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:

    //!
    //! \brief NewlyAvailableParameterList
    //! \param params
    //!
    virtual void NewlyAvailableParameterList(const std::map<std::string, DataGenericItem::DataGenericItem_ParamValue> &params, const OptionalParameter<ModuleCharacteristic> &sender) = 0;

    //!
    //! \brief StartTCPServer Start module TCP server for GUI communications
    //! \return
    //!
    virtual bool StartTCPServer() = 0;


};


} //End MaceCore Namespace

#endif // I_ML_STATION_H
