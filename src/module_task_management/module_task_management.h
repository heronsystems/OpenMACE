#ifndef MODULE_TASK_MANAGEMENT_H
#define MODULE_TASK_MANAGEMENT_H

#include <mutex>
#include "unordered_set"

#include "module_task_management_global.h"

#include "mace_core/i_module_command_task_management.h"

#include "mace_core/i_module_topic_events.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"

#include "tasking_state_machine.h"

#include "task_management_topic/task_management_components.h"

#include "data_generic_item_topic/data_generic_item_topic_components.h"


#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "data_tasks/task_key.h"

class MODULE_TASK_MANAGEMENTSHARED_EXPORT ModuleTaskManagement : public MaceCore::IModuleCommandTaskManagement
{

public:
    ModuleTaskManagement();

    virtual void start();

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
    //! \brief Get all topics that are to be emited by this module
    //! \return List of topics
    //!
    virtual std::vector<MaceCore::TopicCharacteristic> GetEmittedTopics() override;

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

    //!
    //! \brief NewlyAvailableVehicle Subscriber to a newly available vehicle topic
    //! \param vehicleID Vehilce ID of the newly available vehicle
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    int getVehicleID() const;

private:
    Data::TopicDataObjectCollection<DATA_STATE_GENERIC_TOPICS, DATA_GENERIC_VEHICLE_ITEM_TOPICS> m_VehicleDataTopic;

    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_VehicleMissionTopic;

    Data::TopicDataObjectCollection<TASK_ASSIGNMENT_TOPIC> m_TaskAssignmentTopic;
    Data::TopicDataObjectCollection<VEHICLE_TASK_STATUS_TOPIC> m_VehicleTaskStatusTopic;

    int m_vehicleID;


    MaceCore::ModuleCharacteristic m_auctionModuleCharacteristic;
    std::vector<TaskDescriptorPtr> m_assignedTasks;
    std::vector<TaskDescriptorPtr> m_stableAssignedTasks;
    std::unordered_map<TaskKey, Data::EnvironmentTime> m_assignmentTime;
    std::mutex m_assignmentMutex;

    std::mutex m_vehicleMutex;

    TaskingStateMachine m_stateMachine;
    VehicleState m_vehicleState;

    bool m_hasNewAssignments = false;

    int m_assignmentCommandDelay = 5000000;

    double m_takeoffAltitude = 2;
    double m_defaultAltitude = 10;

    bool m_canFly = true;
    bool m_canBeStationary = true;


    std::mutex m_takeoffMutex;
    std::string m_flightMode = "UNKNOWN";

    bool m_waitingOnAuction = false;

    bool m_auctionHasSignaled = false;
    bool m_auctionSignalOK;


    void UpdateStableTaskAssignments();

    void TaskManagementLoop();

    void UpdateAssignmentQueue(const std::vector<string> &componentsUpdated, const MaceCore::ModuleCharacteristic &sender);

    void UpdateTaskStatus();

    void UpdateVehicleState(const std::vector<string> &componentsUpdated, const MaceCore::ModuleCharacteristic &sender);
};

#endif // MODULE_TASK_MANAGEMENT_H
