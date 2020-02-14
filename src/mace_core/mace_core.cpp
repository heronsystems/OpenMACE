#include "mace_core.h"

#include <stdexcept>
#include <iostream>

#include "module_characteristics.h"

namespace MaceCore
{

MaceCore::MaceCore() :
    m_GroundStation(nullptr),
    m_PathPlanning(nullptr),
    m_GlobalRTA(nullptr),
    m_MaceInstanceIDSet(false)
{
}


/////////////////////////////////////////////////////////////////////////
/// CONFIGURE CORE
/////////////////////////////////////////////////////////////////////////

//!
//! \brief AddDataFusion Add MACE data pointer
//! \param dataFusion Data fusion to add
//!
void MaceCore::AddDataFusion(const std::shared_ptr<MaceData> dataFusion)
{
    m_DataFusion = dataFusion;
}


void MaceCore::AddLocalModule(const std::shared_ptr<ModuleBase> &module)
{
    uint8_t moduleID = 1;

    /// If module already has an ID, use it.
    /// If not then assign an ID
    if(module->HasID())
    {
        moduleID = module->GetID();
    }
    else{

        // start search for a suitable module ID by setting to size
        if(m_Modules.size() > 0)
        {
            moduleID = m_Modules.size()+1;
        }

        while(true)
        {
            // check for overflow
            if(moduleID == 0)
            {
                throw std::runtime_error("Trying to find a module ID has overflowed. Have too many modules been added");
            }


            // Don't use if proposed ID is reserved
            if (std::find(m_ReservedModuleIDs.begin(), m_ReservedModuleIDs.end(), moduleID) == m_ReservedModuleIDs.end())
            {
                // Don't use if proposed ID has already been used
                if(m_Modules.find(moduleID) == m_Modules.cend())
                {
                    // Proposed ID is good, add and exit
                    m_Modules.insert({moduleID, module});
                    break;
                }
            }
            moduleID++;
        }
    }

    module->setPararentMaceInstanceID(getMaceInstanceID());
    module->SetID(moduleID);

    std::vector<TopicCharacteristic> topics = module->GetEmittedTopics();
    for(auto it = topics.cbegin() ; it != topics.cend() ; ++it) {
        this->AddTopicCharacteristic(module.get(), *it);
    }
}


//!
//! \brief AddVehicle Add vehicle to data fusion
//! \param ID Vehicle ID to add
//! \param vehicle Vehicle module pointer
//!
void MaceCore::AddLocalModule_Vehicle(const std::string &ID, const std::shared_ptr<IModuleCommandVehicle> &vehicle)
{
    std::lock_guard<std::mutex> guard(m_VehicleMutex);

    if(m_VehicleIDToPtr.find(ID) != m_VehicleIDToPtr.cend())
        throw std::runtime_error("Vehicle ID already exists");

    m_VehicleIDToPtr.insert({ID, vehicle.get()});
    m_VehiclePTRToID.insert({vehicle.get(), ID});

    AddLocalModule(vehicle);

    vehicle->addListener(this);
    vehicle->addTopicListener(this);


    //notify all existing external links about new RTA module
    if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            (*it)->MarshalCommandTwoParameter(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, vehicle->GetCharacteristic(), ModuleClasses::VEHICLE_COMMS);
        }
    }
}


//!
//! \brief RemoveVehicle Remove vehicle from data fusion
//! \param ID Vehicle ID to remove
//!
void MaceCore::RemoveLocalModule_Vehicle(const std::string &ID)
{
    if(m_VehicleIDToPtr.find(ID) == m_VehicleIDToPtr.cend())
        throw std::runtime_error("Vehicle does not exists");

    m_VehiclePTRToID.erase(m_VehicleIDToPtr.at(ID));
    m_VehicleIDToPtr.erase(m_VehicleIDToPtr.find(ID));


    m_DataFusion->RemoveVehicle(ID);
}


//!
//! \brief AddExternalLink Add external link module
//! \param externalLink External link module setup
//!
void MaceCore::AddLocalModule_ExternalLink(const std::shared_ptr<IModuleCommandExternalLink> &externalLink)
{
    AddLocalModule(externalLink);

    externalLink->addListener(this);
    externalLink->addTopicListener(this);
    m_ExternalLink.push_back(externalLink);

    //if there is a ground station, notify this new external link about the existance of GS
    if(m_GroundStation != nullptr)
    {
        externalLink->MarshalCommandTwoParameter(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, m_GroundStation->GetCharacteristic(), ModuleClasses::GROUND_STATION);
    }

    //if there is an RTA module, notify this new external link about the existance of the rta module
    if(m_GlobalRTA != nullptr)
    {
        externalLink->MarshalCommandTwoParameter(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, m_GlobalRTA->GetCharacteristic(), ModuleClasses::RTA);
    }

    //if there are specialized RTA modules, notify this new external link about the existance of the rta modules
    for (std::shared_ptr<IModuleCommandRTA> specializedRTA: m_SpecailizedRTA)
        externalLink->MarshalCommandTwoParameter(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, specializedRTA->GetCharacteristic(), ModuleClasses::RTA);


    //If there are vehiclemodules, notify this new external link about each ones existance
    for(auto it = m_VehicleIDToPtr.cbegin() ; it != m_VehicleIDToPtr.cend() ; ++it)
    {
        externalLink->MarshalCommandTwoParameter(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, it->second->GetCharacteristic(), ModuleClasses::VEHICLE_COMMS);
    }
}

//!
//! \brief AddGroundStationModule Add ground station module
//! \param groundStation Ground station module setup
//!
void MaceCore::AddGroundStationModule(const std::shared_ptr<IModuleCommandGroundStation> &groundStation)
{
    AddLocalModule(groundStation);

    groundStation->addListener(this);
    groundStation->addTopicListener(this);
    groundStation->StartTCPServer();
    m_GroundStation = groundStation;


    //notify all existing external links about new ground station.
    if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            (*it)->MarshalCommandTwoParameter(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, m_GroundStation->GetCharacteristic(), ModuleClasses::GROUND_STATION);
        }
    }


}

//!
//! \brief AddPathPlanningModule Add path planning module
//! \param pathPlanning Path planning module setup
//!
void MaceCore::AddPathPlanningModule(const std::shared_ptr<IModuleCommandPathPlanning> &pathPlanning)
{
    pathPlanning->addListener(this);
    pathPlanning->addTopicListener(this);
    m_PathPlanning = pathPlanning;

    AddLocalModule(pathPlanning);
}

//!
//! \brief AddROSModule Add ROS module
//! \param ros ROS module setup
//!
void MaceCore::AddROSModule(const std::shared_ptr<IModuleCommandROS> &ros)
{
    ros->addListener(this);
    ros->addTopicListener(this);
    m_ROS = ros;

    AddLocalModule(ros);
}


//!
//! \brief Add a generic RTA module to the MACE instance
//!
//! This function simpy consults the module and calls either AddLocalModule_GlobalRTA or AddLocalModule_SpecializedRTA
//! \param rta Module to add
//!
void MaceCore::AddLocalModule_GenericRTA(const std::shared_ptr<IModuleCommandRTA> &rta)
{
    if(rta->getModuleMetaData().IsGlobal())
    {
        AddLocalModule_GlobalRTA(rta);
    }
    else
    {
        AddLocalModule_SpecializedRTA(rta);
    }
}


//!
//! \brief Add a global RTA module
//!
//! A single global RTA module only exists on one MACE instance in the entire MACE network.
//! \param rta RTA module
//!
void MaceCore::AddLocalModule_GlobalRTA(const std::shared_ptr<IModuleCommandRTA> &rta)
{
    AddLocalModule(rta);

    rta->addListener(this);
    rta->addTopicListener(this);
    m_GlobalRTA = rta;

    //notify all existing external links about new RTA module
    if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            (*it)->MarshalCommandTwoParameter(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, m_GlobalRTA->GetCharacteristic(), ModuleClasses::RTA);
        }
    }
}


//!
//! \brief Add a specialized RTA module
//!
//! A specialized RTA module coordianates with a specific resource (vehicle).
//! There may be multiple specalized RTA modules per MACE instance.
//! \param rta RTA module
//!
void MaceCore::AddLocalModule_SpecializedRTA(const std::shared_ptr<IModuleCommandRTA> &rta)
{
    AddLocalModule(rta);

    rta->addListener(this);
    rta->addTopicListener(this);
    m_SpecailizedRTA.push_back(rta);

    //notify all existing external links about new RTA module
    if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            (*it)->MarshalCommandTwoParameter(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, rta->GetCharacteristic(), ModuleClasses::RTA);
        }
    }
}

//!
//! \brief AddSensorsModule Add sensors module
//! \param sensors Sensors module setup
//!
void MaceCore::AddSensorsModule(const std::shared_ptr<IModuleCommandSensors> &sensors)
{
    sensors->addListener(this);
    sensors->addTopicListener(this);
    m_Sensors = sensors;

    AddLocalModule(sensors);
}

//This ends the functions adding appropriate modules

//!
//! \brief Add a module's topic chacteristic to the mace core
//! \param sender Module that will be using topic
//! \param topic Characteristic of topic
//!
void MaceCore::AddTopicCharacteristic(const ModuleBase *sender, const TopicCharacteristic &topic) {

    if(m_TopicsToReceive.find(sender) == m_TopicsToReceive.cend())
    {
        m_TopicsToReceive.insert({sender, {}});
    }

    if(m_TopicsToReceive.at(sender).find(topic.Name()) == m_TopicsToReceive.at(sender).cend())
    {
        m_TopicsToReceive.at(sender).insert({topic.Name(), topic});
    }
}

//!
//! \brief Subscribe Subscribe to a topic
//! \param sender Module that will be using the topic
//! \param topicName Topic name to subscribe to
//! \param senderIDs Vector of IDs to subscribe
//! \param components Vector of component names to subscribe
//!
void MaceCore::Subscribe(ModuleBase* sender, const std::string &topicName, const std::vector<int> &senderIDs, const std::vector<std::string> &components)
{
    UNUSED(senderIDs);
    UNUSED(components);

    m_TopicNotifierMutex.lock();

    if(m_TopicNotifier.find(topicName) == m_TopicNotifier.cend()) {
        m_TopicNotifier.insert({topicName, {}});
    }
    m_TopicNotifier[topicName].push_back(sender);

    m_TopicNotifierMutex.unlock();
}

//!
//! \brief NewTopicDataValues Subscriber to new topic data values
//! \param moduleFrom Module sending the new data values
//! \param topicName Topic over which new values are present
//! \param sender Sender module characteristic
//! \param time Timestamp
//! \param value Value of the new topic datagram
//! \param target Target of the new topic datagram
//!
void MaceCore::NewTopicDataValues(const ModuleBase* moduleFrom, const std::string &topicName, const ModuleCharacteristic &sender, const TIME &time, const TopicDatagram &value, const OptionalParameter<ModuleCharacteristic> &target)
{
    if(this->m_TopicsToReceive.at(moduleFrom).find(topicName) == m_TopicsToReceive.at(moduleFrom).cend())
    {
        throw std::runtime_error("Topic emitted to MaceCore was not expected");
    }

    //if vehicle hasn't been added yet then return. This can happen if a topic is sent before heartbeat is seen
    if(value.HasNonTerminal("system_operating_mode") && m_DataFusion->HasModuleAsMavlinkID(sender) == false) {
        std::string modeStr = value.GetNonTerminal("system_operating_mode").get()->GetTerminal<std::string>("systemModeString");
        uint8_t vehicleID;
        if(m_DataFusion->getMavlinkIDFromModule(sender, vehicleID)) {
            MaceLog::Alert("Mode string for vehicle ID " + std::to_string(vehicleID) + " is: " + modeStr);
            m_DataFusion->updateVehicleFlightMode(vehicleID, modeStr);
        }
    }


    if(this->m_TopicsToReceive.at(moduleFrom).at(topicName).Spooled() == true)
    {
        std::vector<std::string> components = value.ListNonTerminals();

        m_DataFusion->setTopicDatagram(topicName, sender, time, value);

        //list through all interested parties and notify of new topic data
        m_TopicNotifierMutex.lock();
        if(m_TopicNotifier.find(topicName) != m_TopicNotifier.cend())
        {
            for(std::vector<ModuleBase*>::const_iterator it = m_TopicNotifier.at(topicName).cbegin() ; it != m_TopicNotifier.at(topicName).cend() ; ++it) {
                if((*it) == moduleFrom) continue;
                (*it)->NewTopicSpooled(topicName, sender, components, target);
            }
        }
        m_TopicNotifierMutex.unlock();
    }
    else
    {
        m_TopicNotifierMutex.lock();
        //list through all interested parties and notify of new topic data
        if(m_TopicNotifier.find(topicName) != m_TopicNotifier.cend())
        {
            for(std::vector<ModuleBase*>::const_iterator it = m_TopicNotifier.at(topicName).cbegin() ; it != m_TopicNotifier.at(topicName).cend() ; ++it) {
                if((*it) == moduleFrom) continue;
                (*it)->NewTopicData(topicName, sender, value, target);
            }
        }
        m_TopicNotifierMutex.unlock();
    }
}

//!
//! \brief NewTopicDataValues Subscriber to new topic data values
//! \param moduleFrom Module sending the new data values
//! \param topicName Topic over which the new values are present
//! \param senderID Sender module ID
//! \param time Timestamp
//! \param value Value of the new topic datagram
//!
void MaceCore::NewTopicDataValues(const ModuleBase* moduleFrom, const std::string &topicName, const int senderID, const TIME &time, const TopicDatagram &value) {

    //printf("Deprecated!\n");
    //throw std::runtime_error("Deprecated");

    std::vector<std::string> components = value.ListNonTerminals();

    ModuleCharacteristic senderModule = m_DataFusion->GetVehicleFromMAVLINKID(senderID);

    m_DataFusion->setTopicDatagram(topicName, senderModule, time, value);

    ModuleCharacteristic sender = m_DataFusion->GetVehicleFromMAVLINKID(senderID);

    if(value.HasNonTerminal("system_operating_mode")) {
        std::string modeStr = value.GetNonTerminal("system_operating_mode").get()->GetTerminal<std::string>("systemModeString");

        MaceLog::Alert("Mode string: " + modeStr);
        m_DataFusion->updateVehicleFlightMode(senderID, modeStr);
    }

    m_TopicNotifierMutex.lock();
    //list through all interested parties and notify of new topic data
    if(m_TopicNotifier.find(topicName) != m_TopicNotifier.cend())
    {
        for(std::vector<ModuleBase*>::const_iterator it = m_TopicNotifier.at(topicName).cbegin() ; it != m_TopicNotifier.at(topicName).cend() ; ++it) {
            ModuleBase *caller = *it;
            if(caller == moduleFrom) continue;
            caller->NewTopicSpooled(topicName, sender, components);
        }
    }
    m_TopicNotifierMutex.unlock();
}


/////////////////////////////////////////////////////////////////////////
/// GENERAL MODULE EVENTS
/////////////////////////////////////////////////////////////////////////


//!
//! \brief RequestDummyFunction Test function
//! \param sender Sender module
//! \param vehicleID Vehicle ID
//!
void MaceCore::RequestDummyFunction(const void *sender, const int &vehicleID)
{

}


//!
//! \brief Event to notify the core that a new module has been attached to some MACE instance.
//! \param sender Module that generated the new module. May be different than the module itself (Like and External Link)
//! \param characterstic Characterstic that uniquly identifies the module on the MACE topology
//! \param type Type of module
//!
void MaceCore::Event_NewModule(const ModuleBase* sender, const ModuleCharacteristic &characterstic, const ModuleClasses &type)
{
    //if event came from external link then its a remote module, otherwise assume local
    if(characterstic.MaceInstance != this->getMaceInstanceID())
    {
        if(this->m_DataFusion->AddRemoteModule(characterstic, type) == true)
        {
            printf("New Remote Module. Mace: %d ID: %d Type: %s\n", characterstic.MaceInstance, characterstic.ModuleID, ModuleBase::ModuleTypeToString(type).c_str());
        }
    }
    else {
        this->m_DataFusion->AddLocalModule(characterstic, type);
    }
}


//!
//! \brief MaceCore::Event_IssueCommandGoTo
//! \param sender
//! \param gotTo
//!
void MaceCore::Event_IssueCommandGoTo(const ModuleBase* sender, const command_item::Action_ExecuteSpatialItem &gotTo)
{
    int vehicleID = gotTo.getTargetSystem();
    MarshalCommandToVehicle<command_item::Action_ExecuteSpatialItem>(vehicleID, VehicleCommands::EXECUTE_ACTION_SPATIALITEM, ExternalLinkCommands::EXECUTE_ACTION_SPATIALITEM, gotTo, sender->GetCharacteristic());
}

//!
//! \brief Event to signify that a new vehicle is available
//! \param sender Module that generated the vehicle. May be different than the vehicle itself if the vehicle came in over External Link
//! \param publicID Public ID of vehicle, may be different of MACE's internal address
//! \param vehicleModule MACE internal address of vehicle Module
//!
void MaceCore::Events_NewVehicle(const ModuleBase *sender, const uint8_t publicID, const ModuleCharacteristic &vehicleModule)
{
    std::lock_guard<std::mutex> guard(m_VehicleMutex);

    if(sender->ModuleClass() == ModuleClasses::EXTERNAL_LINK)
    {
        IModuleCommandExternalLink* externalLink = (IModuleCommandExternalLink*)sender;
        m_ExternalLinkIDToPort.insert({publicID, externalLink});
        m_DataFusion->AddAvailableVehicle(publicID, false, vehicleModule);
    }
    else if(sender->ModuleClass() == ModuleClasses::VEHICLE_COMMS)
    {
        IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
        m_VehicleIDToPort.insert({publicID, vehicle});
        m_DataFusion->AddAvailableVehicle(publicID, true, vehicleModule);
    }
    else {
        throw std::runtime_error("Currently only an External Link or Vehicle can generate knowledge of a new vehicle");
    }


    if(m_GroundStation)
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_VEHICLE, publicID, vehicleModule);

    if(m_GlobalRTA)
        m_GlobalRTA->MarshalCommand(RTACommands::NEWLY_AVAILABLE_VEHICLE, publicID, vehicleModule);

    if(m_ROS)
        m_ROS->MarshalCommand(ROSCommands::NEWLY_AVAILABLE_VEHICLE, publicID, vehicleModule);

    if(m_GroundStation.get() != nullptr)
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_VEHICLE, publicID, vehicleModule);

    //No mace_mavlink message exists to notify remote instances about the new vehicle. Relly on heartbeat instead.
}


//!
//! \brief Event_ForceVehicleDataSync Event to force a data dump of everything available to a vehicle
//! \param sender Sender module
//! \param targetSystemID Target ID of event
//!
void MaceCore::Event_ForceVehicleDataSync(const ModuleBase *sender, const int &targetSystemID)
{
    //MarshalCommandToVehicle<int>(targetSystemID, VehicleCommands::REQUEST_DATA_SYNC, ExternalLinkCommands::REQUEST_DATA_SYNC, targetSystemID, sender->GetCharacteristic());
}

//!
//! \brief Event_IssueCommandSystemArm Event to trigger and ARM command
//! \param sender Sender module
//! \param command Arm/Disarm action
//!
void MaceCore::Event_IssueCommandSystemArm(const ModuleBase* sender, const command_item::ActionArm &command)
{
    UNUSED(sender);
    int vehicleID = command.getTargetSystem();
    MarshalCommandToVehicle<command_item::ActionArm>(vehicleID, VehicleCommands::CHANGE_VEHICLE_ARM, ExternalLinkCommands::CHANGE_VEHICLE_ARM, command, sender->GetCharacteristic());
}

//!
//! \brief Event_IssueCommandTakeoff Event to trigger a TAKEOFF command
//! \param sender Sender module
//! \param command Takeoff command
//!
void MaceCore::Event_IssueCommandTakeoff(const ModuleBase* sender, const command_item::SpatialTakeoff &command)
{
    UNUSED(sender);
    int vehicleID = command.getTargetSystem();
    MarshalCommandToVehicle<command_item::SpatialTakeoff>(vehicleID, VehicleCommands::REQUEST_VEHICLE_TAKEOFF, ExternalLinkCommands::REQUEST_VEHICLE_TAKEOFF, command, sender->GetCharacteristic());
}

//!
//! \brief Event_IssueCommandLand Event to trigger a LAND command
//! \param sender Sender module
//! \param command Land command
//!
void MaceCore::Event_IssueCommandLand(const ModuleBase* sender, const command_item::SpatialLand &command)
{
    UNUSED(sender);
    int vehicleID = command.getTargetSystem();
    MarshalCommandToVehicle<command_item::SpatialLand>(vehicleID, VehicleCommands::REQUEST_VEHICLE_LAND, ExternalLinkCommands::REQUEST_VEHICLE_LAND, command, sender->GetCharacteristic());
}

//!
//! \brief Event_IssueCommandRTL Event to trigger an RTL command
//! \param sender Sender module
//! \param command RTL command
//!
void MaceCore::Event_IssueCommandRTL(const ModuleBase* sender, const command_item::SpatialRTL &command)
{
    UNUSED(sender);
    int vehicleID = command.getTargetSystem();
    MarshalCommandToVehicle<command_item::SpatialRTL>(vehicleID, VehicleCommands::REQUEST_VEHICLE_RTL, ExternalLinkCommands::REQUEST_VEHICLE_RTL, command, sender->GetCharacteristic());
}

//!
//! \brief Event_IssueMissionCommand Event to trigger a mission command
//! \param sender Sender module
//! \param command Mission command
//!
void MaceCore::Event_IssueMissionCommand(const ModuleBase* sender, const command_item::ActionMissionCommand &command)
{
    UNUSED(sender);
    int vehicleID = command.getTargetSystem();
    MarshalCommandToVehicle<command_item::ActionMissionCommand>(vehicleID, VehicleCommands::SET_MISSION_STATE, ExternalLinkCommands::SET_MISSION_STATE, command, sender->GetCharacteristic());
}

//!
//! \brief Event_ChangeSystemMode Event to trigger a mode change
//! \param sender Sender module
//! \param command Mode change command
//!
void MaceCore::Event_ChangeSystemMode(const ModuleBase *sender, const command_item::ActionChangeMode &command)
{
    UNUSED(sender);
    int vehicleID = command.getTargetSystem();

    //Update the underalying data object
//    m_DataFusion->updateVehicleFlightMode(command.getTargetSystem(), command.getRequestMode());

    MarshalCommandToVehicle<command_item::ActionChangeMode>(vehicleID, VehicleCommands::CHANGE_VEHICLE_MODE, ExternalLinkCommands::CHANGE_VEHICLE_MODE, command, sender->GetCharacteristic());
}

//!
//! \brief Event_IssueGeneralCommand Event to trigger a general command
//! \param sender Sender module
//! \param command General command
//!
void MaceCore::Event_IssueGeneralCommand(const ModuleBase* sender, const command_item::AbstractCommandItem &command)
{
    switch(command.getCommandType())
    {
    case command_item::COMMANDTYPE::CI_ACT_ARM:
    {
        return Event_IssueCommandSystemArm(sender, (command_item::ActionArm const&)command);
    }
    case command_item::COMMANDTYPE::CI_NAV_TAKEOFF:
    {
        return Event_IssueCommandTakeoff(sender, (command_item::SpatialTakeoff const&)command);
    }
    case command_item::COMMANDTYPE::CI_NAV_LAND:
    {
        return Event_IssueCommandLand(sender, (command_item::SpatialLand const&)command);
    }
    case command_item::COMMANDTYPE::CI_NAV_RETURN_TO_LAUNCH:
    {
        return Event_IssueCommandRTL(sender, (command_item::SpatialRTL const&)command);
    }
    case command_item::COMMANDTYPE::CI_ACT_EXECUTE_SPATIAL_ITEM:
    {
        return Event_IssueMissionCommand(sender, (command_item::ActionMissionCommand const&)command);
    }
    case command_item::COMMANDTYPE::CI_ACT_CHANGEMODE:
    {
        return Event_ChangeSystemMode(sender, (command_item::ActionChangeMode const&)command);
    }
    default:
    {
        std::cout << "General command not handled" << std::endl;
    }
    }
}

//!
//! \brief Event_GetMission Event to trigger a "get mission" action
//! \param sender Sender module
//! \param key Mission key to fetch
//!
void MaceCore::Event_GetMission(const void *sender, const MissionItem::MissionKey &key)
{
    UNUSED(sender);
    int systemID = key.m_systemID;

    if(systemID == 0)
    {

    }else{
        try{
            MarshalCommandToVehicle<int>(systemID, VehicleCommands::REQUEST_MISSION, ExternalLinkCommands::REQUEST_MISSION, systemID);
        }catch(const std::out_of_range &oor){

        }
    }
}

//!
//! \brief Event_GetOnboardMission Event to trigger a fetch of the current onboard mission for a vehicle
//! \param sender Sender module
//! \param systemID Vehicle ID whose mission we are asking for
//! \param type Mission type
//!
void MaceCore::Event_GetOnboardMission(const void *sender, const int &systemID, const MissionItem::MISSIONTYPE &type)
{
    UNUSED(sender);

    VehicleCommands cmd = VehicleCommands::REQUEST_ONBOARD_AUTO_MISSION;
    ExternalLinkCommands cmd2 = ExternalLinkCommands::REQUEST_ONBOARD_AUTO_MISSION;

    if(type == MissionItem::MISSIONTYPE::AUTO)
    {
        //nothing to change since this is the default
    }else if(type == MissionItem::MISSIONTYPE::GUIDED){
        cmd = VehicleCommands::REQUEST_ONBOARD_GUIDED_MISSION;
        cmd2 = ExternalLinkCommands::REQUEST_ONBOARD_GUIDED_MISSION;
    }else{
        //we should throw some type of error
    }

    if(systemID == 0)
    {
        //how should we handle the case to transmit this to all vehicle instances
    }else{
        try{
            MarshalCommandToVehicle<int>(systemID, cmd, cmd2, systemID);
        }catch(const std::out_of_range &oor){

        }
    }
}

//!
//! \brief Event_GetCurrentMission Event to trigger a "get current mission" action
//! \param sender Sender module
//! \param systemID Vehicle ID whose mission we are asking for
//!
void MaceCore::Event_GetCurrentMission(const void *sender, const int &systemID)
{
    UNUSED(sender);
    MarshalCommandToVehicle<int>(systemID, VehicleCommands::REQUEST_CURRENT_MISSION, ExternalLinkCommands::REQUEST_CURRENT_MISSION, systemID);
}

//!
//! \brief RequestClearVehicleMission Request a vehicle clear its onboard mission
//! \param sender Sender module
//! \param systemID Vehicle ID whose mission we want to clear
//!
void MaceCore::RequestClearVehicleMission(const void* sender, const Data::SystemDescription &systemID)
{
    UNUSED(sender);
    MarshalCommandToVehicle<Data::SystemDescription>(systemID.getSystemID(), VehicleCommands::CLEAR_CURRENT_MISSION, ExternalLinkCommands::CLEAR_CURRENT_MISSION, systemID.getSystemID());
}

//!
//! \brief Event_GetHomePosition Event to trigger a fetch of a vehicle's home position
//! \param sender Sender module
//! \param vehicleID Vehicle ID whose home position we want
//!
void MaceCore::Event_GetHomePosition(const void* sender, const int &vehicleID)
{
    UNUSED(sender);
    MarshalCommandToVehicle<int>(vehicleID, VehicleCommands::REQUEST_VEHICLE_HOME, ExternalLinkCommands::REQUEST_VEHICLE_HOME, vehicleID);
}

//!
//! \brief Event_SetHomePosition Event to trigger a set home position action
//! \param sender Sender module
//! \param vehicleHome New vehicle home position
//!
void MaceCore::Event_SetHomePosition(const ModuleBase *sender, const command_item::SpatialHome &vehicleHome)
{
    UNUSED(sender);
    int vehicleID = vehicleHome.getTargetSystem();
    MarshalCommandToVehicle<command_item::SpatialHome>(vehicleID, VehicleCommands::SET_VEHICLE_HOME, ExternalLinkCommands::SET_VEHICLE_HOME, vehicleHome, sender->GetCharacteristic());
}

//!
//! \brief RequestVehicleClearGuidedMission Request to clear a vehicle's guided mission
//! \param sender Sender module
//! \param vehicleID Vehicle ID whose guided mission we want to clear
//!
void MaceCore::RequestVehicleClearGuidedMission(const void* sender, const int &vehicleID)
{
    UNUSED(sender);

    MarshalCommandToVehicle<int>(vehicleID, VehicleCommands::CLEAR_ONBOARD_GUIDED_MISSION, ExternalLinkCommands::CLEAR_ONBOARD_GUIDED_MISSION, vehicleID);

}

//!
//! \brief Event_SetGridSpacing Event to set a new grid spacing
//! \param sender Sender module
//! \param gridSpacing New grid spacing
//!
void MaceCore::Event_SetGridSpacing(const void *sender, const double &gridSpacing)
{
    UNUSED(sender);
    m_DataFusion->UpdateGridSpacing(gridSpacing);
}

//!
//! \brief Event_SetGlobalOrigin Event to set a new global origin
//! \param sender Sender module
//! \param globalHome New global origin position
//!
void MaceCore::Event_SetGlobalOrigin(const ModuleBase* sender, const GeodeticPosition_3D &position)
{
    m_DataFusion->UpdateGlobalOrigin(position);

    command_item::Action_SetGlobalOrigin swarmOrigin;
    swarmOrigin.setGlobalOrigin(&position);
    MarshalCommandToVehicle<command_item::Action_SetGlobalOrigin>(swarmOrigin.getTargetSystem(), VehicleCommands::UPDATE_GLOBAL_ORIGIN, ExternalLinkCommands::UPDATE_GLOBAL_ORIGIN, swarmOrigin);

    if(m_PathPlanning && m_PathPlanning.get() != sender) {
        m_PathPlanning->MarshalCommand(PathPlanningCommands::UPDATE_GLOBAL_ORIGIN, position);
    }
    if(m_GroundStation && m_GroundStation.get() != sender) {
        m_GroundStation->MarshalCommand(GroundStationCommands::UPDATE_GLOBAL_ORIGIN, position);
    }
    if(m_GlobalRTA) {
        m_GlobalRTA->MarshalCommand(RTACommands::UPDATE_GLOBAL_ORIGIN, position);
    }
    if(m_Sensors) {
        m_Sensors->MarshalCommand(SensorCommands::UPDATE_GLOBAL_ORIGIN, position);
    }
}

void MaceCore::Event_ProcessGoalState(const ModuleBase *sender, const mace::state_space::GoalState &state)
{
    if(m_PathPlanning && m_PathPlanning.get() != sender) {
        m_PathPlanning->MarshalCommand(PathPlanningCommands::PROCESS_TARGET_STATE, state);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/// SPECIFIC VEHICLE EVENTS: These events are associated from IModuleEventsVehicleVehicle
/////////////////////////////////////////////////////////////////////////////////////////

//!
//! \brief EventVehicle_NewOnboardVehicleMission New onboard mission
//! \param sender Sender module
//! \param missionList New missionlist
//!
void MaceCore::EventVehicle_NewOnboardVehicleMission(const ModuleBase *sender, const MissionItem::MissionList &missionList)
{
    UNUSED(sender);
    //Update the core about the information
    MissionItem::MissionKey key = missionList.getMissionKey();
    m_DataFusion->receivedNewMission(missionList);
    bool isMissionCurrent = m_DataFusion->checkForCurrentMission(key);

    //Now update all potential listeners based on the type
    if(m_GroundStation)
    {
        if(m_DataFusion->getCurrentMissionValidity(missionList.getVehicleID()))
        {
            m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION,missionList.getMissionKey(), sender->GetCharacteristic());
        }
    }else if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            (*it)->MarshalCommand(ExternalLinkCommands::NEWLY_AVAILABLE_ONBOARD_MISSION,missionList.getMissionKey(), sender->GetCharacteristic());
        }
    }
}

//!
//! \brief EventVehicle_MissionACK New mission ACK event
//! \param sender Sender module
//! \param ack Mission ack
//!
void MaceCore::EventVehicle_MissionACK(const void *sender, const MissionItem::MissionACK &ack)
{
    UNUSED(sender);

    //first we should update the core based on the acknowledgment information we had recieved
    //this will update the approriate keys as necessary
    MissionItem::MissionKey key = m_DataFusion->receivedMissionACKKey(ack.getMissionKey(), ack.getNewMissionState());

    if(m_ExternalLinkIDToPort.count(key.m_creatorID) > 0)
    {
        //this implies we can talk to the creator of this mission
        //let us send the acknowledgement to them
        m_ExternalLinkIDToPort.at(key.m_creatorID)->MarshalCommand(ExternalLinkCommands::RECEIVED_MISSION_ACK, ack);
    }
    //This may not be the place to do this
    bool isMissionCurrent = m_DataFusion->checkForCurrentMission(key);
    if((isMissionCurrent) && (m_GroundStation))
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION,key);
}

//!
//! \brief EventVehicle_REJECTProposedMission Event to trigger a rejected mission action
//! \param sender Sender module
//! \param key Rejected mission key
//!
void MaceCore::EventVehicle_REJECTProposedMission(const void *sender, const MissionItem::MissionKey &key)
{
    UNUSED(sender);
    UNUSED(key);
}

//!
//! \brief ExternalEvent_UpdateRemoteID Update remote ID external event
//! \param sender Sender module
//! \param remoteID New remote ID
//!
void MaceCore::ExternalEvent_UpdateRemoteID(const void *sender, const unsigned int &remoteID)
{
    //KEN FIX THIS
    IModuleCommandExternalLink* externalLink = (IModuleCommandExternalLink*)sender;
    m_ExternalLinkIDToPort.insert({remoteID,externalLink});
}


/////////////////////////////////////////////////////////////////////////
/// VEHICLE EVENTS
/////////////////////////////////////////////////////////////////////////


//!
//! \brief GVEvents_NewHomePosition New home position
//! \param sender Sender module
//! \param vehicleHome New vehicle home
//!
void MaceCore::GVEvents_NewHomePosition(const ModuleBase *sender, const command_item::SpatialHome &vehicleHome)
{    
    UNUSED(sender);
    //TODO FIX KEN: We should incorporate a method that shall exist to understand who wants to receive
    //specific methods and information. Otherwise we may be blasting to an unknown world.
    //This is also bad as we are assuming that the only item calling this would be a vehicle instance

    uint8_t vehicleID = vehicleHome.getOriginatingSystem();
    m_DataFusion->UpdateVehicleHomePosition(vehicleID, vehicleHome);

    //If there is a ground station, and it didn't generate the home; send the home position
    if(m_GroundStation && m_GroundStation.get() != sender)
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_HOME_POSITION,vehicleHome, sender->GetCharacteristic());

    if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            if(it->get() == sender)
            {
                continue;
            }
            (*it)->MarshalCommand(ExternalLinkCommands::NEWLY_AVAILABLE_HOME_POSITION,vehicleHome, sender->GetCharacteristic());
        }
    }

    //If targeting a vehicle on this module and not comming from self, set the home.
    if(m_VehicleIDToPort.find(vehicleHome.getTargetSystem()) != m_VehicleIDToPort.cend() && m_VehicleIDToPort.at(vehicleHome.getTargetSystem()) != sender)
    {
        m_VehicleIDToPort.at(vehicleHome.getTargetSystem())->MarshalCommand(VehicleCommands::SET_VEHICLE_HOME, vehicleHome, sender->GetCharacteristic());
    }
}

//!
//! \brief GVEvents_MissionExeStateUpdated New mission EXE state event
//! \param sender Sender module
//! \param missionKey Mission key corresponding to the new EXE state
//! \param missionExeState New EXE state
//!
void MaceCore::GVEvents_MissionExeStateUpdated(const void *sender, const MissionItem::MissionKey &missionKey, const Data::MissionExecutionState &missionExeState)
{
    UNUSED(sender);
    //TODO FIX KEN: We should incorporate a method that shall exist to understand who wants to receive
    //specific methods and information. Otherwise we may be blasting to an unknown world.
    //This is also bad as we are assuming that the only item calling this would be a vehicle instance
    m_DataFusion->updateMissionExeState(missionKey,missionExeState);
    if(m_GroundStation)
        m_GroundStation->MarshalCommand(GroundStationCommands::NEW_MISSION_EXE_STATE,missionKey);
    else if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            (*it)->MarshalCommand(ExternalLinkCommands::NEW_MISSION_EXE_STATE,missionKey);
        }
    }
}

//!
//! \brief GVEvents_MissionItemAchieved Mission item achieved event
//! \param sender Sender module
//! \param achieved Mission item achieved data
//!
void MaceCore::GVEvents_MissionItemAchieved(const void *sender, const MissionItem::MissionItemAchieved &achieved)
{
    UNUSED(sender);
    UNUSED(achieved);
    //I dont know if we need to do anything with this?
}

//!
//! \brief GVEvents_MissionItemCurrent New current mission item event
//! \param sender Sender module
//! \param current New current mission item
//!
void MaceCore::GVEvents_MissionItemCurrent(const void *sender, const MissionItem::MissionItemCurrent &current)
{
    UNUSED(sender);
    m_DataFusion->updateCurrentMissionItem(current);
}

//!
//! \brief GVEvents_NewSystemTime Emitted to alert the core that a module connected to a vehicle has an updated system time
//! \param sender Sender module
//! \param systemTime New system time
//!
void MaceCore::GVEvents_NewSystemTime(const ModuleBase *sender, const DataGenericItem::DataGenericItem_SystemTime &systemTime)
{
    m_DataFusion->updateCurrentSystemTimeDelta(systemTime.getUsecSinceEpoch());
}



//!
//! \brief ConfirmedOnboardVehicleMission Confirm onboard mission event
//! \param sender Sender module
//! \param missionKey Mission key to confirm
//!
void MaceCore::ConfirmedOnboardVehicleMission(const void *sender, const MissionItem::MissionKey &missionKey)
{
    UNUSED(sender);
    UNUSED(missionKey);
    //m_DataFusion->updateOnboardMissions(missionKey);
}

//!
//! \brief NewCurrentVehicleMission New current mission event
//! \param sender Sender module
//! \param missionKey New mission key
//!
void MaceCore::NewCurrentVehicleMission(const void *sender, const MissionItem::MissionKey &missionKey)
{
    UNUSED(sender);
    m_DataFusion->checkForCurrentMission(missionKey);

    if(m_GroundStation)
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION,missionKey);
}

/////////////////////////////////////////////////////////////////////////
/// EXTERNAL LINK EVENTS
/////////////////////////////////////////////////////////////////////////

//!
//! \brief ExternalEvent_MissionACK Event signaling a mission ACK
//! \param sender Sender module
//! \param missionACK Mission ACK
//!
void MaceCore::ExternalEvent_MissionACK(const void* sender, const MissionItem::MissionACK &missionACK)
{
    UNUSED(sender);
    std::cout<<"The core has seen an event from the external link confirming the mission"<<std::endl;
    //first we should update the core based on the acknowledgment information we had recieved
    //this will update the approriate keys as necessary
    MissionItem::MissionKey key = m_DataFusion->receivedMissionACKKey(missionACK.getMissionKey(), missionACK.getNewMissionState());

    //This may not be the place to do this
    bool isMissionCurrent = m_DataFusion->checkForCurrentMission(key);
    if((isMissionCurrent) && (m_GroundStation))
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION,key);
}

//!
//! \brief ExternalEvent_NewOnboardMission New onboard mission event
//! \param sender Sender module
//! \param mission New mission key
//!
void MaceCore::ExternalEvent_NewOnboardMission(const ModuleBase *sender, const MissionItem::MissionKey &mission)
{
    if(m_DataFusion->HasMavlinkID(mission.m_systemID) == false)
    {
        printf("A new mission was received, but it belongs to an unknown module. IGNORING\n");
        return;
    }

    ///////////////////////
    ///MTB Logic goes here to decide if this mace instance is interested in the mission
    ///    For now I am going to only assume interest if there is a GS present
    ///    Other things could be done here, the RTA or PP module can be consulted and asked if interested.
    ///////////////////////
    ///
    bool interest = false;

    //If we have an GS module, assume it is interested in downloading mission and request external link to download mission from aircraft
    if(m_GroundStation != nullptr)
    {
        interest = true;
    }

    ///////////////////////
    /// END interst determination
    ///////////////////////

    //if there is interest ask for download from module that sent the mission notification.
    if(interest == true)
    {
        ModuleCharacteristic requestFrom;
        requestFrom.MaceInstance = this->getMaceInstanceID();
        requestFrom.ModuleID = 0;

        if(sender->ModuleClass() == ModuleClasses::EXTERNAL_LINK)
        {
            ((IModuleCommandExternalLink*)sender)->MarshalCommand(ExternalLinkCommands::REQUEST_MISSION, mission, requestFrom);
        }
    }
}

//!
//! \brief MaceCore has been notified that a new boundary exists on a remote module.
//!
//!
//! \param sender Module that is communicating with remote module
//! \param data Data about boundary
//!
void MaceCore::ExternalEvent_NewBoundary(const ModuleBase *sender, const NewBoundaryData &data)
{
    std::cout << "A Remote boundary was detected" << std::endl;

    ///////////////////////
    ///MTB Logic goes here to decide if this mace instance is interested in the boundary recevied.
    ///    For now I am going to consult if it contains a vehicle attached to this instance.
    ///    Other things could be done here, the RTA or PP module can be consulted and asked if interested.
    ///////////////////////

    //! Set to true if this instance is interested in module
    bool interestedInBoundary = false;

    /// pull list of all vehicles in the boundary
    std::vector<int> vehicles = data.Characteistic.List();

    /// If global boundary and at least has a module attached then make interested
    if(vehicles.size() == 0)
    {
        if(m_GlobalRTA != nullptr)
        {
            interestedInBoundary = true;
        }
        if(m_PathPlanning != nullptr)
        {
            interestedInBoundary = true;
        }
        if(m_VehicleIDToPort.size() > 0)
        {
            interestedInBoundary = true;
        }
    }

    /// Check if an attached vehicle is part of vehicles in the boundary
    for(auto it = m_VehicleIDToPort.cbegin() ; it != m_VehicleIDToPort.cend() ; ++it)
    {
        int attachedVehicleID = it->first;
        if(std::find(vehicles.begin(), vehicles.end(), attachedVehicleID) != vehicles.end())
        {
            interestedInBoundary = true;
        }
    }

    ///////////////////////
    /// END interst determination
    ///////////////////////


    /// if interested issue a download request
    if(interestedInBoundary == true)
    {
        if(sender->ModuleClass() == ModuleClasses::EXTERNAL_LINK)
        {
            ModuleCharacteristic maceInstance;
            maceInstance.MaceInstance = this->getMaceInstanceID();
            maceInstance.ModuleID = 0;
            ((IModuleCommandExternalLink*)sender)->MarshalCommand(ExternalLinkCommands::REQUEST_REMOTE_BOUNDARY, std::make_tuple(data.Sender, data.RemoteIdentifier), maceInstance);
        }
    }

    /*
    if(m_RTA != nullptr)
    {
        //TEMPORARY
        //pull from key, this should probably be the key to the RTA module. i.e. m_RTA->GetCharacterisic()
        //In this case I know that the RTA module that is interested in the same computer with this vehicle.
        ModuleCharacteristic requestFrom;
//        requestFrom.ID = 1;
//        requestFrom.Class = ModuleClasses::VEHICLE_COMMS;
        requestFrom.ID = key.m_systemID;
        requestFrom.Class = ModuleClasses::RTA;

        if(sender->ModuleClass() == ModuleClasses::EXTERNAL_LINK)
        {
            ((IModuleCommandExternalLink*)sender)->MarshalCommand(ExternalLinkCommands::REQUEST_BOUNDARY, key, requestFrom);
        }
    }
    */
}


//!
//! \brief ExternalEvent_RequestingDataSync Request data sync of target's data
//! \param sender Sender module
//! \param targetID Target ID
//!
void MaceCore::ExternalEvent_RequestingDataSync(const void *sender, const ModuleCharacteristic &module)
{
    std::unordered_map<std::string, TopicDatagram> topicMap = m_DataFusion->getAllLatestTopics(module);
    for(auto it = topicMap.cbegin() ; it != topicMap.cend() ; ++it) {
        std::vector<std::string> components = it->second.ListNonTerminals();
        ModuleBase* base = (ModuleBase*)sender;
        //base->NewTopic(it->first,targetID,components);
//        throw std::runtime_error("Requesting Data Sync Not Implemented");
        MaceLog::Debug("Requesting Data Sync not implemented");
    }
}

//!
//! \brief ExternalEvent_FinishedRXMissionList Event signaling the receipt of a mission list
//! \param sender Sender module
//! \param missionList Received mission list
//!
void MaceCore::ExternalEvent_FinishedRXMissionList(const void *sender, const MissionItem::MissionList &missionList)
{
    UNUSED(sender);
    MissionItem::MISSIONSTATE state = missionList.getMissionTXState();
    MissionItem::MissionKey key = missionList.getMissionKey();

    m_DataFusion->receivedNewMission(missionList);
    bool isMissionCurrent = m_DataFusion->checkForCurrentMission(key);

    //This may not be the place to do this
    if((isMissionCurrent) && (m_GroundStation))
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION,key);
    else if(state ==  MissionItem::MISSIONSTATE::RECEIVED)//This implies that the mission state has just moved from proposed to received
    {
        int vehicleID = missionList.getVehicleID();
        MarshalCommandToVehicle<MissionItem::MissionList>(vehicleID, VehicleCommands::UPLOAD_MISSION, ExternalLinkCommands::UPLOAD_MISSION, missionList);
    }
}

//!
//! \brief ExternalEvent_FinishedRXBoundaryList Event signaling the receipt of a boundary list
//! \param sender Sender module
//! \param boundaryList New boundary list
//!
void MaceCore::ExternalEvent_FinishedRXBoundaryList(const void *sender, const BoundaryItem::BoundaryList &boundaryList)
{
    UNUSED(sender);

    throw std::runtime_error("External Link Finished Receiving boundary list not implimented");

    /*

    BoundaryItem::BOUNDARYTYPE type = boundaryList.getBoundaryType();
    BoundaryItem::BoundaryKey key = boundaryList.getBoundaryKey();

    std::cout << "External event finished RX boundary list" << std::endl;

    //    m_DataFusion->receivedNewBoundaryList(boundaryList);

    if(m_PathPlanning) {
        // Marshal command for new boundary list
        if(type == BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE) {
            m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_UPDATED_OPERATIONAL_FENCE, boundaryList);
        }
    }

    if(m_RTA) {
        // Marshal command for new boundary list
        if(type == BoundaryItem::BOUNDARYTYPE::RESOURCE_FENCE) {
            m_RTA->MarshalCommand(RTACommands::NEWLY_UPDATED_RESOURCE_FENCE, boundaryList);
        }
        if(type == BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE) {
            m_RTA->MarshalCommand(RTACommands::NEWLY_UPDATED_OPERATIONAL_FENCE, boundaryList);
        }
    }
    */
}

/////////////////////////////////////////////////////////////////////////
/// RTA EVENTS
/////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////
/// GROUND STATION EVENTS
/////////////////////////////////////////////////////////////////////////


//!
//! \brief Event fired when a new list of targets are produced for a specific vehicle
//! \param vehicleID Vechile new targets are to be applied to
//! \param target List of positional targets
//!
void MaceCore::GroundStationEvent()
{
}

//!
//! \brief CommandNewVehicleMode Command a new vehicle mode change
//! \param vehicleMode New vehicle mode string
//!
void MaceCore::CommandNewVehicleMode(const std::string &vehicleMode)
{
    UNUSED(vehicleMode);
}

//!
//! \brief Event_UploadMission method calls the appropriate operation methods to migrate the proposed
//! mission list to the appropriate vehicle module for handling.
//! \param sender
//! \param missionList
//!
void MaceCore::GSEvent_UploadMission(const void *sender, const MissionItem::MissionList &missionList)
{
    UNUSED(sender);

    MissionItem::MissionList::MissionListStatus status = missionList.getMissionListStatus();

    if(status.state == MissionItem::MissionList::INCOMPLETE) //this checks to make sure the list is fully populated
        return;

    int vehicleID = missionList.getVehicleID();
    if(vehicleID == 0) //transmit this mission to all vehicles
    {
        for (std::map<int, IModuleCommandVehicle*>::iterator it=m_VehicleIDToPort.begin(); it!=m_VehicleIDToPort.end(); ++it){
            int nextSystemID = it->first;
            MissionItem::MissionKey key = m_DataFusion->appendAssociatedMissionMap(nextSystemID,missionList);
            MissionItem::MissionList correctedMission = missionList;
            correctedMission.setMissionKey(key);
            MarshalCommandToVehicle<MissionItem::MissionList>(vehicleID, VehicleCommands::UPLOAD_MISSION, ExternalLinkCommands::UPLOAD_MISSION, correctedMission);
        }
    }else{ //transmit the mission to a specific vehicle
        MissionItem::MissionKey key = m_DataFusion->appendAssociatedMissionMap(missionList);
        MissionItem::MissionList correctedMission = missionList;
        correctedMission.setMissionKey(key);

        if(m_PathPlanning)
        {
            m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_AVAILABLE_MISSION,correctedMission);
        }else
        {
            try{
                MarshalCommandToVehicle<MissionItem::MissionList>(vehicleID, VehicleCommands::UPLOAD_MISSION, ExternalLinkCommands::UPLOAD_MISSION, correctedMission);
            }catch(const std::out_of_range &oor){

            }


        }

    }
}

/////////////////////////////////////////////////////////////////////////
/// PATH PLANNING EVENTS
/////////////////////////////////////////////////////////////////////////

void MaceCore::EventPP_ExecuteDynamicTarget(const ModuleBase* sender, const command_item::Action_DynamicTarget &obj)
{
    MarshalCommandToVehicle<command_item::Action_DynamicTarget>(obj.getTargetSystem(), VehicleCommands::EXECUTE_DYNAMIC_TARGET, ExternalLinkCommands::EXECUTE_DYNAMIC_TARGET, obj, sender->GetCharacteristic());

}

//!
//! \brief EventPP_LoadOccupancyEnvironment Load a new occupancy map
//! \param sender Sender module
//! \param filePath Occupancy map file path
//!
void MaceCore::EventPP_LoadOccupancyEnvironment(const ModuleBase *sender, const string &filePath)
{
    std::cout<<"Somehow load occupancy environment is being called"<<std::endl;
    if(m_DataFusion->loadOccupancyEnvironment(filePath))
    {
        //we have loaded a new map which means we need to notify everyone

        //we dont have to check if PP exists here because we know it has to as it is the caller
        m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_LOADED_OCCUPANCY_MAP,0);
    }
}

//!
//! \brief EventPP_LoadOctomapProperties Load octomap properties
//! \param sender Sender module
//! \param properties Octomap properties
//!
void MaceCore::EventPP_LoadOctomapProperties(const ModuleBase *sender, const maps::OctomapSensorDefinition &properties)
{
    std::cout<<"Somehow load octomap properties is being called"<<std::endl;
    if(m_DataFusion->updateOctomapProperties(properties))
    {
        //we have loaded a new map which means we need to notify everyone

        //we dont have to check if PP exists here because we know it has to as it is the caller
        m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_LOADED_OCCUPANCY_MAP,0);

        if(m_ROS)
        {
            //m_ROS->MarshalCommand(ROSCommands::NEWLY_COMPRESSED_OCCUPANCY_MAP, map);
            m_ROS->MarshalCommand(ROSCommands::NEWLY_UPDATED_3D_OCCUPANCY_MAP, 0); // TODO: Parse for vehicle ID

        }
    }
}

//!
//! \brief EventPP_LoadMappingProjectionProperties Load map projection properties
//! \param sender Sender module
//! \param properties Map projection properties
//!
void MaceCore::EventPP_LoadMappingProjectionProperties(const ModuleBase *sender, const maps::Octomap2DProjectionDefinition &properties)
{

}

//!
//! \brief Function to fire when a new boundary of some kind was generated by a module
//! \param sender Module that generated the boundary
//! \param key Key indicating the characteristics of the boundary
//! \param boundary Data for the boundary
//!
void MaceCore::Event_SetBoundary(const ModuleBase *sender, const BoundaryItem::BoundaryCharacterisic &characterstic, const BoundaryItem::BoundaryList &boundary)
{
    std::string list_str = "";
    if(characterstic.List().size() == 0)
    {
        list_str = "global";
    }
    if(characterstic.List().size() > 0)
    {
        list_str += "[ ";
        std::vector<int> list = characterstic.List();
        for(auto it = list.cbegin() ; it != list.cend() ; ++it)
        {
            list_str += std::to_string(*it) + " ";
        }
        list_str += "]";
    }
    printf("Mace Core: Received a new Boundary\n  verticies: %d\n  Type: %s\n  Vehicles: %s\n", boundary.getQueueSize(), BoundaryItem::BoundaryTypeToString(characterstic.Type()).c_str(), list_str.c_str());

    //Update the underalying data object
    uint8_t rtnKey = m_DataFusion->setBoundaryByKey(characterstic, boundary);

    if(m_GroundStation.get() == sender)
    {
        printf("!!!!!!MADISON TESTING!!!!! - RTA module isn't full developed, so restricting transmission of boundary to global RTA instance only when boundary came from GS\n");
        if(m_GlobalRTA && sender != m_GlobalRTA.get()) {
            m_GlobalRTA->MarshalCommand(RTACommands::NEWLY_AVAILABLE_BOUNDARY, rtnKey);
        }
    }


    // Iterate over all specialized RTA modules and notify of new boundary
    for(std::vector<std::shared_ptr<IModuleCommandRTA>>::iterator it = m_SpecailizedRTA.begin() ; it != m_SpecailizedRTA.end() ; ++it)
    {
        if(it->get() == sender) continue;

        (*it)->MarshalCommand(RTACommands::NEWLY_AVAILABLE_BOUNDARY, rtnKey, sender->GetCharacteristic());
    }

    // TODO-@Ken: Does the PP module need the operational fence?
//    if(m_PathPlanning && sender != m_PathPlanning) {
//        m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_UPDATED_OPERATIONAL_FENCE, boundary);
//    }

    if(m_ROS && sender != m_ROS.get())
    {
        m_ROS->MarshalCommand(ROSCommands::NEWLY_AVAILABLE_BOUNDARY, rtnKey);
    }

    if(m_GroundStation && sender != m_GroundStation.get()) {
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_BOUNDARY, rtnKey);
    }

    if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it = m_ExternalLink.begin(); it != m_ExternalLink.end(); ++it)
        {
            if(it->get() == sender)
            {
                continue;
            }

            (*it)->MarshalCommand(ExternalLinkCommands::NEWLY_AVAILABLE_BOUNDARY, rtnKey, sender->GetCharacteristic());
        }
    }
}


//!
//! \brief Event fired to indicate what planning horizon is being utilized by the path planning module
//! \param horizon ID of the horizon being utilized
//!
void MaceCore::PlanningHorizon(const std::string &horizon)
{
    UNUSED(horizon);
    throw std::runtime_error("Not Implemented");
}

//!
//! \brief ReplaceVehicleCommands Replace vehicle commands with new commands
//! \param vehicleID Target vehicle ID
//! \param movementCommands New commands
//!
void MaceCore::ReplaceVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands)
{
    m_DataFusion->setVehicleDynamicsCommands(vehicleID, movementCommands);

    m_VehicleIDToPtr.at(vehicleID)->MarshalCommand(VehicleCommands::FOLLOW_NEW_COMMANDS);
}

//!
//! \brief ReplaceAfterCurrentVehicleCommands Append vehicle commands after current vehicle commands
//! \param vehicleID Target vehicle ID
//! \param movementCommands New commands
//!
void MaceCore::ReplaceAfterCurrentVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands)
{
    m_DataFusion->setVehicleDynamicsCommands(vehicleID, movementCommands);

    m_VehicleIDToPtr.at(vehicleID)->MarshalCommand(VehicleCommands::FINISH_AND_FOLLOW_COMMANDS);
}

//!
//! \brief AppendVehicleCommands Append vehicle commands to current vehicle commands
//! \param vehicleID Target vehicle ID
//! \param movementCommands New commands
//!
void MaceCore::AppendVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands)
{
    std::vector<FullVehicleDynamics> commands = m_DataFusion->getVehicleDynamicsCommands(vehicleID);
    commands.insert(commands.end(), movementCommands.begin(), movementCommands.end());
    m_DataFusion->setVehicleDynamicsCommands(vehicleID, commands);

    m_VehicleIDToPtr.at(vehicleID)->MarshalCommand(VehicleCommands::COMMANDS_APPENDED);
}

//!
//! \brief EventPP_New2DOccupancyMap New compressed 2D occupancy map event
//! \param sender Sender module
//! \param map Occupancy map
//!
void MaceCore::EventPP_New2DOccupancyMap(const void* sender, const mace::maps::Data2DGrid<mace::maps::OccupiedResult> &map)
{
    if(m_ROS)
        m_ROS->MarshalCommand(ROSCommands::NEWLY_COMPRESSED_OCCUPANCY_MAP, map);
}

//!
//! \brief EventPP_NewDynamicMissionQueue New dynamic mission queue event
//! \param sender Sender module
//! \param queue New mission queue
//!
void MaceCore::EventPP_NewDynamicMissionQueue(const ModuleBase *sender, const command_target::DynamicMissionQueue &queue)
{
    UNUSED(sender);

    //Find who the queue is intended for
    int vehicleID = queue.getAssociatedMissionKey().m_systemID;
    try{
        //Marshal the command to that vehicle
        m_VehicleIDToPtr.at(std::to_string(vehicleID))->MarshalCommand(VehicleCommands::UPDATED_DYNAMIC_MISSION_QUEUE, queue);
    }catch(const std::out_of_range &oor)
    {

    }
}

//!
//! \brief EventPP_NewPathFound New path found event
//! \param sender Sender module
//! \param path Path vector
//!
void MaceCore::EventPP_NewPathFound(const void* sender, const std::vector<mace::state_space::StatePtr> &path)
{
    UNUSED(sender);
    if(m_ROS)
        m_ROS->MarshalCommand(ROSCommands::NEWLY_FOUND_PATH, path);
}

//!
//! \brief Event fired when a new occupancy map to be invoked when PathPlanning module generates a new occupancy map.
//! \param occupancyMap New occupancy map
//!
void MaceCore::NewOccupancyMap(const Eigen::MatrixXd &occupancyMap)
{
    m_DataFusion->OccupancyMap_ReplaceMatrix(occupancyMap);
}


//!
//! \brief Event fired when the PathPlanning modules determines that a set of cells should be modified on the occupancy map.
//!
//! This event may be faster than NewOccupancyMap when the matrix is large and the modifcations are sparse
//! \param commands List of cells to modify
//!
void MaceCore::ReplaceOccupancyMapCells(const std::vector<MatrixCellData<double>> &commands)
{

    std::function<void(Eigen::MatrixXd &)> func = [&commands](Eigen::MatrixXd &mat){
        ReplaceCellsInMatrix(mat, commands);
    };

    m_DataFusion->OccupancyMap_GenericOperation(func);

}


/////////////////////////////////////////////////////////////////////////
/// SENSOR MODULE EVENTS
/////////////////////////////////////////////////////////////////////////

void MaceCore::Sensors_UpdatedMapLayer(const string &layerName, mace::maps::BaseGridMap* mapLayer)
{
    MaceLog::Alert("Updated Map layer name: " + layerName);

    m_DataFusion->updateLayeredMapLayer(layerName, mapLayer);

    const shared_ptr<mace::maps::LayeredMap> tmp = m_DataFusion->getLayeredMap();
    MaceLog::Alert(std::to_string(tmp->getXResolution()));
}

/////////////////////////////////////////////////////////////////////////
/// ROS MODULE EVENTS
/////////////////////////////////////////////////////////////////////////

//!
//! \brief ROS_NewLaserScan New laser scan from ROS/Gazebo
//! \param obj Point cloud object
//! \param position Position of sensor
//!
void MaceCore::ROS_NewLaserScan(const octomap::Pointcloud &obj, const CartesianPosition_3D &position)
{
    octomap::Pointcloud copyObj = obj;
    m_DataFusion->insertGlobalObservation(copyObj, position);
    if(m_ROS)
        m_ROS->MarshalCommand(ROSCommands::NEWLY_UPDATED_3D_OCCUPANCY_MAP, 0); // TODO: Parse for vehicle ID
}

//!
//! \brief ROS_NewLaserScan New laser scan from ROS/Gazebo
//! \param obj Point cloud object
//! \param position Position of sensor
//! \param orientation Orientation of sensor
//!
void MaceCore::ROS_NewLaserScan(const octomap::Pointcloud &obj, const CartesianPosition_3D &position, const mace::pose::Rotation_3D &orientation)
{
    octomap::Pointcloud copyObj = obj;
    m_DataFusion->insertObservation(copyObj, position, orientation);
    //Marshal Command To PP and RTA

    /*    ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
    }); *///this is a general publication event, however, no one knows explicitly how to handle

    /*    ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
        ptr->EventVehicle_NewConstructedVehicle(this, systemID);
    });*/ //this one explicitly calls mace_core and its up to you to handle in core

    //    if(m_PathPlanning)
    //        m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_UPDATED_OCCUPANCY_MAP, 0); // TODO: Parse for vehicle ID

    if(m_ROS)
        m_ROS->MarshalCommand(ROSCommands::NEWLY_UPDATED_3D_OCCUPANCY_MAP, 0); // TODO: Parse for vehicle ID

}

void MaceCore::ROS_NewVisionPoseEstimate(const unsigned int &vehicleID, const mace::pose::Pose &pose)
{
    try
    {
        m_VehicleIDToPtr.at(std::to_string(vehicleID))->MarshalCommand(VehicleCommands::TRANSMIT_VISION_POSE_ESTIMATE,pose);
    }
    catch(const std::out_of_range &oor)
    {
        std::cout<<"The vehicle ID is not contained within this MACE instances vehicle map.";
    }

}


/////////////////////////////////////////////////////////////////////////
/// MACE COMMS EVENTS
/////////////////////////////////////////////////////////////////////////


} //END MaceCore Namespace
