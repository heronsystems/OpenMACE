#ifndef MODULE_TASK_GENERATION_H
#define MODULE_TASK_GENERATION_H

#include "module_task_generation_global.h"

#include "common/common.h"

#include "mace_core/i_module_command_task_generation.h"

#include "data/topic_data_object_collection.h"
#include "base_topic/base_topic_components.h"
#include "task_generation_UMD/frontiergeneration.h"
#include "task_generation_UMD/random_generation.h"
#include "data_generic_state_item_topic/state_topic_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"


class MODULE_TASK_GENERATIONSHARED_EXPORT ModuleTaskGeneration : public MaceCore::IModuleCommandTaskGeneration
{

public:
    ModuleTaskGeneration();

    virtual ~ModuleTaskGeneration() = default;

    //!
    //! \brief OnModulesStarted Fired when all modules have started, indicates the core is ready to marshal data
    //!
    virtual void OnModulesStarted() override
    {
       std::cout<<"MODULE TASK GENERATION STARTED" <<std::endl;

      // TestFunction();
    }


    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr) override;

    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const override;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params) override;

    //!
    //! \brief New non-spooled topic given
    //!
    //! NonSpooled topics send their data immediatly.
    //! \param topicName Name of stopic
    //! \param sender Module that sent topic
    //! \param data Data for topic
    //! \param target Target module (or broadcasted)
    //!
    void NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target) override;


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
    void NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;


    //!
    //! \brief TestFunction
    //! \param
    //!
    void TestFunction(const int &vehicleID) override;


    void LayeredMapLayerUpdated(const std::string &layerName) override;

    //!
    //! \brief updateGlobalPositionData Update the position of the corresponding vehicle and convert to a local position (from Geodetic 3D)
    //! \param vehicleID ID of the vehicle to update
    //! \param component Position (in a global, Geodetic frame)
    //!
    void updateGlobalPositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> &component);

    //!
    //! \brief NewlyAvailableVehicle Subscriber to a newly available vehicle topic
    //! \param vehicleID Vehilce ID of the newly available vehicle
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

private:

    bool frontierBlobAlgorithm;
    bool frontierRoadAlgorithm;
    bool randomAlgorithm;


    std::vector<std::shared_ptr<Abstract_TaskGeneration>> m_taskGenerationList;

    std::map<int, mace::maps::CartesianPosition_2D*> m_currentVehiclePoseMap;
    int m_vehicleID;

private:

    //Subscribing to generic vehicle state data that is published throughout the MACE network
    Data::TopicDataObjectCollection<DATA_GENERIC_VEHICLE_ITEM_TOPICS, DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
};

#endif // MODULE_TASK_GENERATION_H
