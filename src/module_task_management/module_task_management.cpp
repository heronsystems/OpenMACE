#include "module_task_management.h"

#include "data_generic_item_topic/data_generic_item_topic_battery.h"
#include "data_generic_item_topic/data_generic_item_topic_system_arm.h"

#include "data_generic_command_item/spatial_items/spatial_takeoff.h"

ModuleTaskManagement::ModuleTaskManagement() :
    m_VehicleDataTopic("vehicleData"),
    m_TaskAssignmentTopic("TaskAssignment"),
    m_VehicleTaskStatusTopic("VehicleTaskStatus"),
    m_VehicleMissionTopic("vehicleMission"),
    m_stateMachine(this)
{

}

void ModuleTaskManagement::start()
{
    std::thread thread([&]()
    {
        TaskManagementLoop();
    });
    thread.detach();

    AbstractModule_EventListeners::start();
}

void ModuleTaskManagement::AttachedAsModule(MaceCore::IModuleTopicEvents *ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_TaskAssignmentTopic.Name());
    ptr->Subscribe(this, m_VehicleTaskStatusTopic.Name());
    ptr->Subscribe(this, m_VehicleMissionTopic.Name());
}

std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleTaskManagement::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;

    structure.AddTerminalParameters("VehicleID", MaceCore::ModuleParameterTerminalTypes::INT, true);
    structure.AddTerminalParameters("AssignmentCommandDelay", MaceCore::ModuleParameterTerminalTypes::INT, false);

    structure.AddTerminalParameters("RequiredDistanceToTarget", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    structure.AddTerminalParameters("MinTurnRadius", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    structure.AddTerminalParameters("NominalSpeed", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    structure.AddTerminalParameters("NominalClimbRate", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    structure.AddTerminalParameters("NominalAcceleration", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    structure.AddTerminalParameters("DefaultAltitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    structure.AddTerminalParameters("TakeoffAltitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    structure.AddTerminalParameters("CanFly", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, false);
    structure.AddTerminalParameters("CanBeStationary", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, false);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}

void ModuleTaskManagement::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    m_vehicleID = params->GetTerminalValue<int>("VehicleID");

    if (params->HasTerminal("AssignmentCommandDelay"))
        m_assignmentCommandDelay = 1000 * params->GetTerminalValue<int>("AssignmentCommandDelay");

    if (params->HasTerminal("RequiredDistanceToTarget"))
        m_stateMachine.setRequiredDistanceToTarget(params->GetTerminalValue<double>("RequiredDistanceToTarget"));

    if (params->HasTerminal("MinTurnRadius"))
        m_stateMachine.setMinTurnRadius(params->GetTerminalValue<double>("MinTurnRadius"));

    if (params->HasTerminal("NominalSpeed"))
        m_stateMachine.setNominalSpeed(params->GetTerminalValue<double>("NominalSpeed"));

    if (params->HasTerminal("NominalClimbRate"))
        m_stateMachine.setNominalClimbRate(params->GetTerminalValue<double>("NominalClimbRate"));

    if (params->HasTerminal("NominalAcceleration"))
        m_stateMachine.setNominalAcceleration(params->GetTerminalValue<double>("NominalAcceleration"));

    if (params->HasTerminal("DefaultAltitude"))
    {
        m_defaultAltitude = params->GetTerminalValue<double>("DefaultAltitude");
    }
    m_stateMachine.setDefaultAltitude(m_defaultAltitude);

    if (params->HasTerminal("TakeoffAltitude"))
    {
        m_takeoffAltitude = params->GetTerminalValue<double>("TakeoffAltitude");
    }
    m_stateMachine.setTakeoffAltitude(m_takeoffAltitude);

    if (params->HasTerminal("CanFly"))
        m_stateMachine.setCanFly(params->GetTerminalValue<bool>("CanFly"));

    if (params->HasTerminal("CanBeStationary"))
        m_stateMachine.setCanBeStationary(params->GetTerminalValue<bool>("CanBeStationary"));
}

std::vector<MaceCore::TopicCharacteristic> ModuleTaskManagement::GetEmittedTopics()
{
    std::vector<MaceCore::TopicCharacteristic> topicCharacteristics;
    MaceCore::TopicCharacteristic assignmentTopicCharacteristic(true, m_TaskAssignmentTopic.Name());
    MaceCore::TopicCharacteristic vehicleTaskStatusTopicCharacteristic(false, m_VehicleTaskStatusTopic.Name());

    topicCharacteristics.push_back(assignmentTopicCharacteristic);
    topicCharacteristics.push_back(vehicleTaskStatusTopicCharacteristic);
    return topicCharacteristics;
}

void ModuleTaskManagement::NewTopicData(const string &topicName,
                                        const MaceCore::ModuleCharacteristic &sender,
                                        const MaceCore::TopicDatagram &data,
                                        const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    if (target.IsSet() && target.Value() != this->GetCharacteristic())
        return; // Not for us

    if (topicName == m_VehicleTaskStatusTopic.Name())
    {
        std::shared_ptr<VehicleTaskStatusTopic> vehicleStatusTopic = std::make_shared<VehicleTaskStatusTopic>();
        if (m_VehicleTaskStatusTopic.GetComponent(vehicleStatusTopic, data))
        {

            if (!target.IsSet() && vehicleStatusTopic->getVehicleID() != m_vehicleID)
                return;
            m_auctionSignalOK = vehicleStatusTopic->getSuccess();
            m_auctionHasSignaled = true;
        }
    }
}

void ModuleTaskManagement::NewTopicSpooled(const string &topicName,
                                           const MaceCore::ModuleCharacteristic &sender,
                                           const std::vector<string> &componentsUpdated,
                                           const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    if (target.IsSet() && target.Value() != this->GetCharacteristic())
        return; // Not for us

    if (topicName == m_VehicleDataTopic.Name())
        UpdateVehicleState(componentsUpdated, sender);
    else if (topicName == m_TaskAssignmentTopic.Name())
        UpdateAssignmentQueue(componentsUpdated, sender);

}

void ModuleTaskManagement::NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(vehicleID);
    UNUSED(sender);
}

int ModuleTaskManagement::getVehicleID() const
{
    return m_vehicleID;
}

/*!
 * \brief Updates the stable task assignments
 * \details A task is considered stably assigned if it has been seen as assigned to the agent
 * for a sufficient amount of time, which is m_assignmentCommandDelay microseconds.
 */
void ModuleTaskManagement::UpdateStableTaskAssignments()
{
    std::lock_guard<std::mutex> guard(m_assignmentMutex);
    std::unordered_set<TaskKey> currentTaskSet;
    m_stableAssignedTasks.clear();

    Data::EnvironmentTime now;
    Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
    TaskKey key;

    for (const auto &task : m_assignedTasks)
    {
        key = task->getTaskKey();
        auto it = m_assignmentTime.find(key);
        if (it == m_assignmentTime.end()) // Was not previously assigned.
        {
            m_assignmentTime.insert({key, now});
            currentTaskSet.insert(key);
        }
        else // Was previously assigned
        {
            currentTaskSet.insert(key);
            if (now - m_assignmentTime.at(key) > m_assignmentCommandDelay) // Task has been assigned for at least 5 seconds. Assume stable
                m_stableAssignedTasks.push_back(task);
        }
    }

    // Remove any tasks which were lost from having assignment time tracked
    auto it = m_assignmentTime.begin();
    while (it != m_assignmentTime.end())
    {
        if (currentTaskSet.find(it->first) == currentTaskSet.cend())
            it = m_assignmentTime.erase(it);
        else
            ++it;
    }

    m_hasNewAssignments = false;
}

/*!
 * \brief Main Task Management Loop
 */
void ModuleTaskManagement::TaskManagementLoop()
{
    bool newState, newCommand;
    TaskingState state = m_stateMachine.getCurrentState();
    while (true)
    {
        if (m_hasNewAssignments)
        {
            UpdateStableTaskAssignments();
            m_stateMachine.NewAssignmentQueue(m_stableAssignedTasks);
        }

        if (m_auctionHasSignaled)
        {
            m_stateMachine.ReceiveAuctionSignal();
            m_auctionHasSignaled = false;
        }

        m_vehicleMutex.lock();
        if (m_vehicleState.hasPosition()) // Otherwise we haven't received vehicle info yet
            m_stateMachine.AdvanceStateMachine(m_vehicleState, newState, newCommand);
        m_vehicleMutex.unlock();

        state = m_stateMachine.getCurrentState();

        if (newState)
            UpdateTaskStatus();


        // Arm vehicle and takeoff if needed


        if ((!m_vehicleState.hasSystemArm() || !m_vehicleState.getSystemArm().getSystemArm())
                && !(state == TaskingState::Idle || state == TaskingState::Refuel))
        {
            // If the vehicle needs to take off, we know it has some new command to run after.
            newCommand = true;
            while (!m_vehicleState.hasSystemArm() || !m_vehicleState.getSystemArm().getSystemArm())
            {
                auto armCommand = CommandItem::ActionArm(this->getParentMaceInstanceID(), m_vehicleID);
                armCommand.setVehicleArm(true);
                this->NotifyListeners([&](MaceCore::IModuleEventsTaskManagement* ptr)
                {
                    ptr->Event_IssueCommandSystemArm(this, armCommand);
                });

                CommandItem::SpatialTakeoff takeoffCommand;
                takeoffCommand.setTargetSystem(m_vehicleID);
                takeoffCommand.position->setZ(m_takeoffAltitude);

                this->NotifyListeners([&](MaceCore::IModuleEventsTaskManagement* ptr)
                {
                    ptr->Event_IssueCommandTakeoff(this, takeoffCommand);
                });
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // Delay is long enough for vehicle to take off and transition into the guided flight idle state,
            // which it needs to be in for the goto commands being used by tasks. This is a workaround due to
            // a lack of sufficient feedback available on when this transition occurs.
            //
            // Flight mode transitions to GUIDED before goto commands can be issued, so the flight mode
            // is not sufficient to rely on for this transition.
            //
            // TODO Update this when the Ardupilot module gives feedback on whether a command can be issued
            // and/or if a command has been completed.
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }

        if (newCommand)
        {
            const auto &nextCommand = m_stateMachine.GetNextVehicleCommand();
            if (nextCommand != nullptr)
            {
                nextCommand->setOriginatingSystem(this->getParentMaceInstanceID());
                nextCommand->setTargetSystem(m_vehicleID);

                if (nextCommand->getCommandType() == CommandItem::COMMANDITEM::CI_ACT_GOTO)
                {
                    this->NotifyListeners([&](MaceCore::IModuleEventsTaskManagement* ptr)
                    {
                        ptr->Event_IssueCommandGoTo(this, static_cast<CommandItem::CommandGoTo &>(*nextCommand));
                    });
                }
                else
                {
                    this->NotifyListeners([&](MaceCore::IModuleEventsTaskManagement* ptr)
                    {
                        ptr->Event_IssueGeneralCommand(this, *nextCommand);
                    });
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/*!
 * \brief Updates the assignment queue
 * \details Called when receiving the appropriate topic.
 * \param componentsUpdated Updated components
 * \param sender Sender
 */
void ModuleTaskManagement::UpdateAssignmentQueue(const std::vector<string> &componentsUpdated, const MaceCore::ModuleCharacteristic &sender)
{
    std::lock_guard<std::mutex> guard(m_assignmentMutex);

    auto assignmentTopic = this->getDataObject()->GetCurrentTopicDatagram(m_TaskAssignmentTopic.Name(), sender);

    for(size_t i = 0 ; i < componentsUpdated.size() ; ++i)
    {
        const std::string &componentName = componentsUpdated.at(i);
        if(componentName == TaskAssignmentTopic::Name())
        {
            std::shared_ptr<TaskAssignmentTopic> assignmentData = std::make_shared<TaskAssignmentTopic>();
            m_TaskAssignmentTopic.GetComponent(assignmentData, assignmentTopic);

            if (assignmentData->getVehicleID() != m_vehicleID)
                return;

            m_auctionModuleCharacteristic = sender;

            m_assignedTasks = assignmentData->getAssignedTasks();
        }
    }
    m_hasNewAssignments = true;
}

/*!
 * \brief Sends a task status update to the auction process
 */
void ModuleTaskManagement::UpdateTaskStatus()
{
    TaskingState state = m_stateMachine.getCurrentState();

    std::shared_ptr<VehicleTaskStatusTopic> statusTopic = std::make_shared<VehicleTaskStatusTopic>();

    switch (state)
    {
        case TaskingState::WaitForAuction_Start:
            statusTopic->setStatusType(TaskStatusType::Started);
            break;
        case TaskingState::WaitForAuction_Complete:
            statusTopic->setStatusType(TaskStatusType::Completed);
            break;
        case TaskingState::WaitForAuction_Abort:
            statusTopic->setStatusType(TaskStatusType::Aborted);
            break;
        case TaskingState::Idle:
        case TaskingState::Refuel:
        case TaskingState::Transition:
        case TaskingState::PerformingTask:
        case TaskingState::Complete:
        case TaskingState::Abort:
            return;
    }
    TaskKey key = m_stateMachine.getCurrentTask()->getTaskKey();
    std::cout << "Updating status on task " << key << std::endl;
    statusTopic->setTaskKey(key);
    statusTopic->setVehicleID(m_vehicleID);

    MaceCore::TopicDatagram topic;

    m_VehicleTaskStatusTopic.SetComponent(statusTopic, topic);

    MaceCore::ModuleCharacteristic characteristic = GetCharacteristic();

    ModuleTaskManagement::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_VehicleTaskStatusTopic.Name(), characteristic, MaceCore::TIME(), topic,
                                OptionalParameter<MaceCore::ModuleCharacteristic>(m_auctionModuleCharacteristic));
    });
}


/*!
 * \brief Updates the state of the connected vehicle
 * \details Called when receiving the appropriate topic.
 * \param componentsUpdated Updated components
 * \param sender Sender
 */
void ModuleTaskManagement::UpdateVehicleState(const std::vector<string> &componentsUpdated, const MaceCore::ModuleCharacteristic &sender)
{
    auto dataObject = this->getDataObject();
    if (!dataObject->HasMavlinkID(m_vehicleID))
        return;
    std::lock_guard<std::mutex> guard(m_vehicleMutex);

    MaceCore::TopicDatagram read_topicDatagram = dataObject->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), dataObject->GetVehicleFromMAVLINKID(m_vehicleID));


    for(size_t i = 0 ; i < componentsUpdated.size() ; ++i)
    {
        const std::string &componentName = componentsUpdated.at(i);
        if(componentName == DataStateTopic::StateLocalPositionTopic::Name())
        {
            continue;

            std::shared_ptr<DataStateTopic::StateLocalPositionTopic> localPositionData = std::make_shared<DataStateTopic::StateLocalPositionTopic>();
            m_VehicleDataTopic.GetComponent(localPositionData, read_topicDatagram);

            m_vehicleState.setPosition(*localPositionData);
        }
        else if(componentName == DataStateTopic::StateLocalPositionExTopic::Name())
        {
            continue;

            std::shared_ptr<DataStateTopic::StateLocalPositionExTopic> localPositionData = std::make_shared<DataStateTopic::StateLocalPositionExTopic>();
            m_VehicleDataTopic.GetComponent(localPositionData, read_topicDatagram);

            m_vehicleState.setPosition(*localPositionData);
        }
        else if(componentName == DataStateTopic::StateGlobalPositionTopic::Name())
        {
            std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> globalPositionData = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
            m_VehicleDataTopic.GetComponent(globalPositionData, read_topicDatagram);

            DataState::StateLocalPosition tempLocal;
            DataState::StateGlobalPosition tempOrigin;
            Position<GeodeticPosition_3D> globalOrigin = this->getDataObject()->GetGlobalOrigin();
            tempOrigin.setPosition(globalOrigin.getLatitude(), globalOrigin.getLongitude(), globalOrigin.getAltitude());

            DataState::PositionalAid::GlobalPositionToLocal(tempOrigin, *globalPositionData, tempLocal);
            if (!globalPositionData->getPosZFlag())
                tempLocal.setPositionZ(m_defaultAltitude);

            m_vehicleState.setPosition(tempLocal);
        }
        else if(componentName == DataStateTopic::StateGlobalPositionExTopic::Name())
        {
            std::shared_ptr<DataStateTopic::StateGlobalPositionExTopic> globalPositionData = std::make_shared<DataStateTopic::StateGlobalPositionExTopic>();
            m_VehicleDataTopic.GetComponent(globalPositionData, read_topicDatagram);

            DataState::StateLocalPosition tempLocal;
            DataState::StateGlobalPosition tempOrigin;
            Position<GeodeticPosition_3D> globalOrigin = this->getDataObject()->GetGlobalOrigin();
            tempOrigin.setPosition(globalOrigin.getLatitude(), globalOrigin.getLongitude(), globalOrigin.getAltitude());

            DataState::PositionalAid::GlobalPositionToLocal(tempOrigin, *globalPositionData, tempLocal);
            if (!globalPositionData->getPosZFlag())
                tempLocal.setPositionZ(m_defaultAltitude);

            m_vehicleState.setPosition(tempLocal);
        }
        else if(componentName == DataStateTopic::StateLocalVelocityTopic::Name())
        {
            std::shared_ptr<DataStateTopic::StateLocalVelocityTopic> localVelocityData = std::make_shared<DataStateTopic::StateLocalVelocityTopic>();
            m_VehicleDataTopic.GetComponent(localVelocityData, read_topicDatagram);

            m_vehicleState.setVelocity(*localVelocityData);
        }
        else if (componentName == DataStateTopic::StateGlobalVelocityTopic::Name())
        {
            std::shared_ptr<DataStateTopic::StateGlobalVelocityTopic> globalVelocityData = std::make_shared<DataStateTopic::StateGlobalVelocityTopic>();
            m_VehicleDataTopic.GetComponent(globalVelocityData, read_topicDatagram);

            // TODO transform global velocity to local velocity

        }
        else if (componentName == DataStateTopic::StateAttitudeTopic::Name())
        {
            std::shared_ptr<DataStateTopic::StateAttitudeTopic> attitudeData = std::make_shared<DataStateTopic::StateAttitudeTopic>();
            m_VehicleDataTopic.GetComponent(attitudeData, read_topicDatagram);

            m_vehicleState.setAttitude(*attitudeData);
        }
        else if (componentName == DataGenericItemTopic::DataGenericItemTopic_Battery::Name())
        {
            std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> batteryData = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>();
            m_VehicleDataTopic.GetComponent(batteryData, read_topicDatagram);

            m_vehicleState.setBattery(*batteryData);
        }
        else if (componentName == DataGenericItemTopic::DataGenericItemTopic_SystemArm::Name())
        {
            std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> armData = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>();
            m_VehicleDataTopic.GetComponent(armData, read_topicDatagram);

            m_vehicleState.setSystemArm(*armData);
        }
        else if (componentName == DataGenericItemTopic::DataGenericItemTopic_FlightMode::Name())
        {
            std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> modeData = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>();
            m_VehicleDataTopic.GetComponent(modeData, read_topicDatagram);

            m_flightMode = modeData->getFlightModeString();
        }

        Data::EnvironmentTime now;
        Data::EnvironmentTime::CurrentTime(Data::SYSTEMCLOCK, now);
        m_vehicleState.setTimestamp(now);
    }
}


