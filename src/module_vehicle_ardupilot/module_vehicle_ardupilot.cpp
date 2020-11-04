#include "module_vehicle_ardupilot.h"
#include <functional>

#include "mace_core/i_module_events_general.h"

template <typename T>
T CopyCommandAndInsertTarget(const command_item::AbstractCommandItem &item, int targetSystem)
{
    T cpy((T&)item);
    cpy.setTargetSystem(targetSystem);
    return cpy;
}


//ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
//    ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPLANE_TYPES>(),
//    m_VehicleMissionTopic("vehicleMission"), m_AircraftController(nullptr), vehicleData(nullptr)
ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
    ModuleVehicleMAVLINK<>(),
    stateMachine(nullptr)
{

    m_TransmissionQueue = new TransmitQueue(2000, 3);

}

ModuleVehicleArdupilot::~ModuleVehicleArdupilot()
{
    if(stateMachine)
    {
        delete stateMachine;
    }

    m_ControllersCollection.ForAll([this](Controllers::IController<mavlink_message_t, int>* controller){
        controller->RemoveHost(this);
    });

    delete m_TransmissionQueue;
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleVehicleArdupilot::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    ModuleVehicleMAVLINK::ConfigureModule(params);
}

//!
//! \brief createLog Create log file
//! \param systemID ID of vehicle being logged
//!
void ModuleVehicleArdupilot::createLog(const int &systemID)
{
    std::string logname = this->loggingPath + "/VehicleModule_" + std::to_string(systemID) + ".txt";
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleVehicleArdupilot::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleMissionTopic.Name());
}

////////////////////////////////////////////////////////////////////////////
/// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
/// command and action sequence that accompanies the vheicle. Expect an
/// acknowledgement or an event to take place when calling these items.
////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::Command_SetGlobalOrigin(const Action_SetGlobalOrigin &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    mace::pose::GeodeticPosition_3D* originPosition = command.getGlobalOrigin()->positionAs<mace::pose::GeodeticPosition_3D>();
    m_SystemData->environment->swarmGlobalOrigin.set(*originPosition);

//    handleGlobalOriginController(command);
}

void ModuleVehicleArdupilot::Command_ExecuteSpatialItem(const Action_ExecuteSpatialItem &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    UNUSED(command);
}

//!
//! \brief Request_FullDataSync Request all data from all systems
//! \param targetSystem Destination of the data dump receiver
//!
void ModuleVehicleArdupilot::Request_FullDataSync(const int &targetSystem, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> objectData = m_SystemData->state->GetTopicData();
    this->PublishVehicleData(targetSystem,objectData);
    this->prepareMissionController();

    MAVLINKUXVControllers::ControllerMission* missionController = static_cast<MAVLINKUXVControllers::ControllerMission*>(m_ControllersCollection.At("missionController"));
    if(missionController)
    {
        missionController->AddLambda_Finished(this, [missionController](const bool completed, const uint8_t code){
            UNUSED(code);
            if (completed)
                printf("Mission Download Completed\n");
            else
                printf("Mission Download Failed\n");

            missionController->Shutdown();
        });

        missionController->GetMissions(m_SystemData->getMAVLINKID());
    }
}

//!
//! \brief Command_SystemArm Command an ARM/DISARM action
//! \param command ARM/DISARM command
//! \param sender Generating system
//!
void ModuleVehicleArdupilot::Command_SystemArm(const command_item::ActionArm &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    UNUSED(command);
}

//!
//! \brief Command_VehicleTakeoff Command a takeoff action
//! \param command Takeoff altitude and location
//! \param sender Generating system
//!
void ModuleVehicleArdupilot::Command_VehicleTakeoff(const command_item::SpatialTakeoff &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    UNUSED(command);
}

//!
//! \brief Command_Land Command a LAND action
//! \param command Land command
//! \param sender Generating system
//!
void ModuleVehicleArdupilot::Command_Land(const command_item::SpatialLand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    UNUSED(command);
}

//!
//! \brief Command_ReturnToLaunch command a return to launch action
//! \param command RTL command
//! \param sender Generating system
//!
void ModuleVehicleArdupilot::Command_ReturnToLaunch(const command_item::SpatialRTL &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    UNUSED(command);
}

//!
//! \brief Command_MissionState Command a mission state request
//! \param command Mission state request command
//! \param sender Generating system
//!
void ModuleVehicleArdupilot::Command_MissionState(const command_item::ActionMissionCommand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    UNUSED(command);
}

//!
//! \brief Command_ChangeSystemMode Command a system mode change
//! \param command Change mode command
//! \param sender Generating system
//!
void ModuleVehicleArdupilot::Command_ChangeSystemMode(const command_item::ActionChangeMode &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    UNUSED(command);
}

void ModuleVehicleArdupilot::Command_ExecuteDynamicTarget(const command_item::Action_DynamicTarget &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    UNUSED(command);
}

//!
//! \brief Command_IssueGeneralCommand Command a general command
//! \param command General command
//!
void ModuleVehicleArdupilot::Command_IssueGeneralCommand(const std::shared_ptr<command_item::AbstractCommandItem> &command)
{
    UNUSED(command);
    std::cout<<"Ken: we have seen the issuing of a general command!!!! Pay attention to this."<<std::endl;
}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way ardupilot works.
/////////////////////////////////////////////////////////////////////////////

//!
//! \brief Command_GetHomePosition Request a vehicle's home position
//! \param vehicleID Vehicle ID corresponding to the home position
//!
void ModuleVehicleArdupilot::Command_GetHomePosition(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(vehicleID); UNUSED(sender);
    //    if((vehicleData) && (vehicleData->getSystemID() == vehicleID))
    //        vehicleData->command->getSystemHome();
}

//!
//! \brief Command_SetHomePosition Set a vehicle's home position
//! \param vehicleHome Vehicle home data
//!
void ModuleVehicleArdupilot::Command_SetHomePosition(const command_item::SpatialHome &vehicleHome, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    UNUSED(vehicleHome);
}


/////////////////////////////////////////////////////////////////////////
/// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
/// This functionality may be pertinent for vehicles not containing a
/// direct MACE hardware module.
/////////////////////////////////////////////////////////////////////////

//!
//! \brief UpdateMissionKey Update the current mission's key
//! \param key New mission key
//!
void ModuleVehicleArdupilot::UpdateMissionKey(const MissionItem::MissionKeyChange &key)
{
    UNUSED(key);
}

//!
//! \brief Command_UploadMission Command a mission upload
//! \param missionList Mission list to upload
//!
void ModuleVehicleArdupilot::Command_UploadMission(const MissionItem::MissionList &missionList)
{
    UNUSED(missionList);
}

//!
//! \brief Command_SetCurrentMission Issue a set current mission command
//! \param key Mission key to set as current mission
//!
void ModuleVehicleArdupilot::Command_SetCurrentMission(const MissionItem::MissionKey &key)
{
    UNUSED(key);
}

//!
//! \brief Command_GetCurrentMission Issue a get current mission command
//! \param targetSystem System asking for the current mission
//!
void ModuleVehicleArdupilot::Command_GetCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);
}

//!
//! \brief Command_GetMission Request a mission by mission key
//! \param key Mission key
//! \param sender System asking for the mission
//!
void ModuleVehicleArdupilot::Command_GetMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(key);
    UNUSED(sender);
}

//!
//! \brief Command_ClearCurrentMission Clear the current mission
//! \param targetSystem System asking for mission cleared
//!
void ModuleVehicleArdupilot::Command_ClearCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);
}

////////////////////////////////////////////////////////////////////////////
/// GENERAL AUTO EVENTS: This is implying for auto mode of the vehicle.
/// This functionality is pertinent for vehicles that may contain a
/// MACE HW module, or, vehicles that have timely or ever updating changes.
////////////////////////////////////////////////////////////////////////////

//!
//! \brief Command_GetOnboardAuto Request the current onboard auto mission and state
//! \param targetSystem System asking for auto info
//!
void ModuleVehicleArdupilot::Command_GetOnboardAuto(const int &targetSystem)
{
    UNUSED(targetSystem);
}

//!
//! \brief Command_ClearOnboardAuto Clear the current onboard auto mission and state
//! \param targetSystem System to clear auto info
//!
void ModuleVehicleArdupilot::Command_ClearOnboardAuto(const int &targetSystem)
{
    UNUSED(targetSystem);
}

/////////////////////////////////////////////////////////////////////////
/// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
/// This functionality is pertinent for vehicles that may contain a
/// MACE HW module, or, vehicles that have timely or ever updating changes.
/////////////////////////////////////////////////////////////////////////

//!
//! \brief Command_GetOnboardGuided Request the current onboard guided state
//! \param targetSystem System asking for guided info
//!
void ModuleVehicleArdupilot::Command_GetOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

//!
//! \brief Command_ClearOnboardGuided Clear the current onboard guided state
//! \param targetSystem System to clear guided state
//!
void ModuleVehicleArdupilot::Command_ClearOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
bool ModuleVehicleArdupilot::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    bool consumed = false;

    //this is necessary because if we have yet to have received a vehicle heartbeat,
    //we have yet to form the vehicle object and accompanying state machine
    if(m_SystemData)
    {
//        MavlinkEntityKey sender = message.sysid;
        //consumed = m_MissionController->ReceiveMessage(&message, sender);

        if(!consumed)
            consumed = ModuleVehicleMAVLINK::MavlinkMessage(linkName, message);

        if(!consumed)
            consumed = m_SystemData->handleMAVLINKMessage(message);

        if(!consumed)
            consumed = m_SystemData->parseMessage(&message);

        ProgressStateMachineStates();
    }

    return consumed;
}

//!
//! \brief VehicleHeartbeatInfo Heartbeat message from vehicle
//! \param linkName Comms link name
//! \param systemID Vehicle ID generating heartbeat
//! \param heartbeatMSG Heartbeat message
//!
void ModuleVehicleArdupilot::VehicleHeartbeatInfo(const std::string &linkName, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG)
{
    //If module hasn't started yet, then ignore this message
    if(this->ModuleStarted() == false)
    {
        return;
    }

    UNUSED(linkName);
    UNUSED(systemID);
    UNUSED(heartbeatMSG);
}

//!
//! \brief PublishVehicleData Parent publisher for vehicle data
//! \param systemID Vehicle ID generating vehicle data
//! \param components Data components to publish
//!
void ModuleVehicleArdupilot::PublishVehicleData(const int &systemID, const std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> &components)
{
    if(components.size() > 0)
    {
        //construct datagram
        MaceCore::TopicDatagram topicDatagram;
        for(size_t i = 0 ; i < components.size() ; i++)
        {
            m_VehicleDataTopic.SetComponent(components.at(i), topicDatagram);
            //notify listeners of topic
            ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
            });

        }
    } //if there is information available
}

//!
//! \brief New non-spooled topic given
//!
//! NonSpooled topics send their data immediatly.
//! \param topicName Name of stopic
//! \param sender Module that sent topic
//! \param data Data for topic
//! \param target Target module (or broadcasted)
//!
void ModuleVehicleArdupilot::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(sender);
    UNUSED(target);
    if(this->m_TopicToControllers.find(topicName) == m_TopicToControllers.cend())
    {
        throw std::runtime_error("Attempting to send a topic that the vehicle module link has no knowledge of");
    }

    Controllers::IController<mavlink_message_t, int> *controller = m_TopicToControllers.at(topicName);

    if(controller->ContainsAction(Controllers::Actions::SEND) == false)
    {
        throw std::runtime_error("Attempting to send a topic to a controller that has no send action");
    }
    Controllers::IActionSend<MaceCore::TopicDatagram, int> *sendAction = dynamic_cast<Controllers::IActionSend<MaceCore::TopicDatagram, int>*>(controller);
    sendAction->Send(data, 255, this->GetAttachedMavlinkEntity());
}


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
void ModuleVehicleArdupilot::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(target);
    if(topicName == m_VehicleMissionTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleMissionTopic.Name(), sender);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++){
            if(componentsUpdated.at(i) == MissionTopic::MissionItemTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionItemTopic> component = std::make_shared<MissionTopic::MissionItemTopic>();
                m_VehicleMissionTopic.GetComponent(component, read_topicDatagram);
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionListTopic::Name()){
                std::shared_ptr<MissionTopic::MissionListTopic> component = std::make_shared<MissionTopic::MissionListTopic>();
                m_VehicleMissionTopic.GetComponent(component, read_topicDatagram);
            }
        }
    }
}


//!
//! \brief Cause the state machine to update it's states
//!
void ModuleVehicleArdupilot::ProgressStateMachineStates()
{
    m_Mutex_StateMachine.lock();
    stateMachine->ProcessStateTransitions();
    stateMachine->UpdateStates();
    m_Mutex_StateMachine.unlock();
}

void ModuleVehicleArdupilot::TransmitVisionPoseEstimate(const mace::pose::Pose &pose)
{
    UNUSED(pose);
}


void ModuleVehicleArdupilot::UpdateDynamicMissionQueue(const command_target::DynamicMissionQueue &queue)
{
    UNUSED(queue);
}


