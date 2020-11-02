#ifndef MODULE_VEHICLE_ADEPT_H
#define MODULE_VEHICLE_ADEPT_H

#include <iostream>
#include <cmath>
//#include <pybind11/pybind11.h>
#include "module_vehicle_adept_global.h"


#include "mace_core/i_module_topic_events.h"


#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "mace_core/i_module_command_adept.h"


#include "base/pose/dynamics_aid.h"
#include "base_topic/base_topic_components.h"



//namespace py = pybind11;

class MODULE_VEHICLE_ADEPTSHARED_EXPORT ModuleVehicleAdept: public MaceCore::IModuleCommandAdept
{
public:
    ModuleVehicleAdept();

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr);

    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);


    //!
    //! \brief New non-spooled topic given
    //!
    //! NonSpooled topics send their data immediatly.
    //! \param topicName Name of stopic
    //! \param sender Module that sent topic
    //! \param data Data for topic
    //! \param target Target module (or broadcasted)
    //!
    virtual void NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target);


    //!
    //! \brief New Spooled topic given
    //!
    //! Spooled topics are stored on the core's datafusion.
    //! This method is used to notify other modules that there exists new data for the given components on the given module.
    //! \param topicName Name of topic given
    //! \param sender Module that sent topic
    //! \param componentsUpdated Components in topic that where updated
    //! \param target Target moudle (or broadcast)
    //!
    virtual void NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>());


public:

    //!
    //! \brief receiveFrame Gets new frame data from Adept agent
    //!
    void receiveFrame();

    //!
    //! \brief NewlyAvailableVehicle Subscriber to a newly available vehicle topic
    //! \param vehicleID Vehilce ID of the newly available vehicle
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief NewlyAvailableFrame Subscriber to a new frame
    //!
    void NewlyAvailableFrame(const uint8_t &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>());

    //!
    //! \brief createLog Creates a new log to record flight data passing to/from adept agent
    //!
    void createLog();

private:


};


#endif // MODULE_VEHICLE_ADEPT_H
