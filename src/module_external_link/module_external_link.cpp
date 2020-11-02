#include <set>
#include "module_external_link.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////
///             CONFIGURE
////////////////////////////////////////////////////////////////////////////////////////////////////////

void FinishedMessage(const bool completed, const uint8_t finishCode)
{
    if(completed == false)
    {
        printf("Controller timed out sending message, gave up sending message\n");
    }
    else {
        printf("Controller Received Final ACK with code of %d\n", finishCode);
    }
}

template <typename T>
T* Helper_CreateAndSetUp(ModuleExternalLink* obj, TransmitQueue *queue, uint8_t chan)
{
    T* newController = new T(obj, queue, chan);
    newController->setLambda_DataReceived([obj](const MaceCore::ModuleCharacteristic &moduleFor, const AbstractCommandItem &command){obj->ReceivedCommand(moduleFor, command);});
    newController->setLambda_Finished(FinishedMessage);
    return newController;
}


ModuleExternalLink::ModuleExternalLink() :
    m_HeartbeatController(nullptr), airborneInstance(true), associatedSystemID(254),
    m_VehicleDataTopic("vehicleData"),m_MissionDataTopic("vehicleMission")
{
    m_queue = new TransmitQueue(4000, 3);

    m_Controllers.Add(Helper_CreateAndSetUp<ExternalLink::CommandARM>(this, m_queue, m_LinkChan));
    m_Controllers.Add(Helper_CreateAndSetUp<ExternalLink::CommandLand>(this, m_queue, m_LinkChan));
    m_Controllers.Add(Helper_CreateAndSetUp<ExternalLink::CommandMissionItem>(this, m_queue, m_LinkChan));
    m_Controllers.Add(Helper_CreateAndSetUp<ExternalLink::CommandRTL>(this, m_queue, m_LinkChan));
    m_Controllers.Add(Helper_CreateAndSetUp<ExternalLink::CommandTakeoff>(this, m_queue, m_LinkChan));

    auto controller_GoTo = new ExternalLink::Controller_GoTo(this, m_queue, m_LinkChan);
    controller_GoTo->setLambda_DataReceived([this](const MaceCore::ModuleCharacteristic &sender, const AbstractCommandItem &command){this->ReceivedGoToCommand(sender, command);});
    controller_GoTo->setLambda_Finished(FinishedMessage);
    m_Controllers.Add(controller_GoTo);


    auto controller_SystemMode = new ExternalLink::ControllerSystemMode(this, m_queue, m_LinkChan);
    controller_SystemMode->setLambda_DataReceived([this](const MaceCore::ModuleCharacteristic &sender, const AbstractCommandItem &command){this->ReceivedCommand(sender, command);});
    controller_SystemMode->setLambda_Finished(FinishedMessage);
    m_Controllers.Add(controller_SystemMode);


    auto homeController = new ExternalLink::ControllerHome(this, m_queue, m_LinkChan);
    homeController->setLambda_DataReceived([this](const MaceCore::ModuleCharacteristic &key, const command_item::SpatialHome &home){this->ReceivedHome(key, home);});
    homeController->setLambda_FetchDataFromKey([this](const OptionalParameter<MaceCore::ModuleCharacteristic> &key){return this->FetchHomeFromKey(key);});
    homeController->setLambda_FetchAll([this](const OptionalParameter<MaceCore::ModuleCharacteristic> &module){return this->FetchAllHomeFromModule(module);});
    homeController->setLambda_Finished(FinishedMessage);
    m_Controllers.Add(homeController);

    auto boundaryController = new ExternalLink::ControllerBoundary(this, m_queue, m_LinkChan);

    dynamic_cast<Controllers::DataItem<MaceCore::ModuleCharacteristic, BoundaryNotificationData>*>(boundaryController)->setLambda_DataReceived([this](const MaceCore::ModuleCharacteristic &sender, const BoundaryNotificationData &data){this->ReceivedRemoteBoundaryCharacterstic(sender, data.uniqueIdentifier, data.characteristic); });

    dynamic_cast<Controllers::DataItem<ModuleBoundaryIdentifier, BoundaryItem::BoundaryList>*>(boundaryController)->setLambda_DataReceived([this](const ModuleBoundaryIdentifier &sender, const BoundaryItem::BoundaryList &list){this->ReceivedRemoteBoundary(sender.Module(), sender.BoundaryIdentifier(), list);});
    dynamic_cast<Controllers::DataItem<ModuleBoundaryIdentifier, BoundaryItem::BoundaryList>*>(boundaryController)->setLambda_FetchDataFromKey([this](const OptionalParameter<ModuleBoundaryIdentifier> &sender){return this->FetchBoundaryFromKey(sender); });

    //boundaryController->setLambda_DataReceived([this](const uint8_t &key, const std::shared_ptr<BoundaryItem::BoundaryList> &list){this->ReceivedBoundary(*list);});
    //boundaryController->setLambda_FetchDataFromKey([this](const OptionalParameter<uint8_t> &key){return this->FetchBoundaryFromKey(key);});
    //boundaryController->setLambda_FetchAll([this](const OptionalParameter<MaceCore::ModuleCharacteristic> &module){return this->FetchAllBoundariesFromModule(module);});
    boundaryController->setLambda_Finished(FinishedMessage);
    m_Controllers.Add(boundaryController);

    auto missionController = new ExternalLink::ControllerMission(this, m_queue, m_LinkChan);
    dynamic_cast<Controllers::DataItem<MaceCore::ModuleCharacteristic, MissionItem::MissionKey>*>(missionController)->setLambda_DataReceived([this](const MaceCore::ModuleCharacteristic &sender, const MissionItem::MissionKey &key){this->ReceivedRemoteMissionNotification(sender, key);});
    dynamic_cast<Controllers::DataItem<MissionKey, MissionList>*>(missionController)->setLambda_DataReceived([this](const MissionKey &key, const MissionList &list){UNUSED(key);this->ReceivedMission(list);});
    dynamic_cast<Controllers::DataItem<MissionKey, MissionList>*>(missionController)->setLambda_FetchDataFromKey([this](const OptionalParameter<MissionKey> &key){return this->FetchMissionFromKey(key);});
    dynamic_cast<Controllers::DataItem<MissionKey, MissionList>*>(missionController)->setLambda_FetchAll([this](const OptionalParameter<MaceCore::ModuleCharacteristic> &module){return this->FetchAllMissionFromModule(module);});
    missionController->setLambda_Finished(FinishedMessage);
    m_Controllers.Add(missionController);


}

void ModuleExternalLink::ReceivedCommand(const MaceCore::ModuleCharacteristic &moduleFor, const AbstractCommandItem &command)
{
    uint8_t mavlinkID = 0;
    if(moduleFor.MaceInstance != 0)
    {
        if(!this->getDataObject()->getMavlinkIDFromModule(moduleFor, mavlinkID)) {
            mavlinkID = 0;
        }
    }

    std::shared_ptr<AbstractCommandItem> copy = command.getClone();

    copy->setTargetSystem(mavlinkID);

    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
        ptr->Event_IssueGeneralCommand(this, *copy);
    });

}

void ModuleExternalLink::ReceivedGoToCommand(const MaceCore::ModuleCharacteristic &moduleFor, const AbstractCommandItem &command)
{
    uint8_t mavlinkID = 0;
    if(moduleFor.MaceInstance != 0)
    {
        if(!this->getDataObject()->getMavlinkIDFromModule(moduleFor, mavlinkID)) {
            mavlinkID = 0;
        }
    }

    AbstractCommandItemPtr copy = command.getClone();
    command_item::Action_ExecuteSpatialItem* castObj = copy->as<command_item::Action_ExecuteSpatialItem>();
    castObj->setTargetSystem(mavlinkID);

    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
        ptr->Event_IssueCommandGoTo(this, *castObj);
    });
}

ModuleExternalLink::~ModuleExternalLink()
{

}

std::vector<MaceCore::TopicCharacteristic> ModuleExternalLink::GetEmittedTopics()
{
    std::vector<MaceCore::TopicCharacteristic> topics;

    topics.push_back(this->m_VehicleDataTopic.Characterisic());
    topics.push_back(this->m_MissionDataTopic.Characterisic());

    return topics;
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleExternalLink::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_MissionDataTopic.Name());
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleExternalLink::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    ConfigureMACEStructure(structure);
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);

    std::shared_ptr<MaceCore::ModuleParameterStructure> moduleSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    moduleSettings->AddTerminalParameters("AirborneInstance", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    structure.AddNonTerminal("ModuleParameters", moduleSettings, true);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);

}



//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleExternalLink::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    ConfigureMACEComms(params);

    m_LinkMarshaler->SpecifyAddedModuleAction(m_LinkName, [this](const CommsMACE::Resource &resource){
        this->ExternalModuleAdded(resource);
    });

    m_LinkMarshaler->SpecifyAddedModuleAction(m_LinkName, [this](const CommsMACE::Resource &resource){
        this->ExternalModuleRemoved(resource);
    });

    if(params->HasNonTerminal("ModuleParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> moduleSettings = params->GetNonTerminalValue("ModuleParameters");
        airborneInstance = moduleSettings->GetTerminalValue<bool>("AirborneInstance");

        //m_LinkMarshaler->AddResource(m_LinkName, GROUNDSTATION_STR, associatedSystemID);

        if(airborneInstance == false)
        {

        }
    }

}

//!
//! \brief Event to fire when an external module has been added
//! \param resourceName Name of resource (module) added
//! \param ID ID of module
//!
void ModuleExternalLink::ExternalModuleAdded(const CommsMACE::Resource &resource)
{
    if(strcmp(resource.NameAt(0).c_str(), CommsMACE::MACE_INSTANCE_STR) != 0)
    {
        throw std::runtime_error("The first component of given resource is not the mace instance");
    }

    //If only one component then this is a notification of a MACE instance, not a module.
    if(resource.Size() == 1)
    {
        NewExternalMaceInstance(resource.IDAt(0));
        return;
    }
    else
    {
        CommsMACE::Resource expectedInstance;
        expectedInstance.Set<CommsMACE::MACE_INSTANCE_STR>(0);

        if (!m_LinkMarshaler->HasResource(m_LinkName, {expectedInstance}))
        {
            // We received a notifaction for a module, but not for the corresponding MACE instance.
            m_LinkMarshaler->RequestRemoteResources(m_LinkName);
        }
    }

    MaceCore::ModuleCharacteristic module;
    module.MaceInstance = resource.IDAt(0);
    module.ModuleID = resource.IDAt(1);

    MaceCore::ModuleClasses type;
    const char* moduleTypeName = resource.NameAt(1).c_str();
    if(strcmp(moduleTypeName, CommsMACE::VEHICLE_STR) == 0)
    {
        type = MaceCore::ModuleClasses::VEHICLE_COMMS;
    }
    else if(strcmp(moduleTypeName, CommsMACE::GROUNDSTATION_STR) == 0)
    {
        type = MaceCore::ModuleClasses::GROUND_STATION;
    }
    else if(strcmp(moduleTypeName, CommsMACE::MLSTATION_STR) == 0)
    {
        type = MaceCore::ModuleClasses::ML_STATION;
    }
    else if(strcmp(moduleTypeName, CommsMACE::RTA_STR) == 0)
    {
        type = MaceCore::ModuleClasses::RTA;
    }
    else if(strcmp(moduleTypeName, CommsMACE::EXTERNAL_LINK_STR) == 0)
    {
        type = MaceCore::ModuleClasses::EXTERNAL_LINK;
    }
    else {
//        printf("%s\n", CommsMACE::GROUNDSTATION_STR);
        printf("In ExternalModuleAdded - Unknown module class: %s\n", moduleTypeName);
        throw std::runtime_error("Unknown module class received");
    }

    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
        ptr->Event_NewModule(this, module, type);
    });

    //check if there are tasks and perform them
    if(m_TasksToDoWhenModuleComesOnline.find(module) != m_TasksToDoWhenModuleComesOnline.cend())
    {
        printf("Performing queued tasks for the given module\n");
        for(auto it = m_TasksToDoWhenModuleComesOnline.at(module).cbegin() ; it != m_TasksToDoWhenModuleComesOnline.at(module).cend() ; ++it)
        {
            (*it)();
        }
        m_TasksToDoWhenModuleComesOnline.erase(module);
    }
}

void ModuleExternalLink::ExternalModuleRemoved(const CommsMACE::Resource &resource)
{
    UNUSED(resource);
    throw std::runtime_error("Not Implimented");
}

std::string ModuleExternalLink::createLog(const unsigned int &systemID)
{
    loggerCreated = true;
    std::string logname = this->loggingPath + "/ExternalLinkModule" + std::to_string(systemID) + ".txt";
    std::string loggerName = "ExternalLinkModule_" + std::to_string(systemID);

    return loggerName;
}

void ModuleExternalLink::TransmitMessage(const mace_message_t &msg, const OptionalParameter<MaceCore::ModuleCharacteristic> &target) const
{
    //broadcast
    if(target.IsSet() == false)
    {
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
        return;
    }

    // If the ID is zero, then we are sending message to a generic MACE instance
    // otherwise we are sending to an individual module
    if(target().ModuleID == 0)
    {
        CommsMACE::Resource r;
        r.Set<CommsMACE::MACE_INSTANCE_STR>(target().MaceInstance);
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg, r);
    }
    else
    {
        // Sending to an individual module.
        // Determine the type and send out on digimesh link appropriately.
        MaceCore::ModuleClasses type = this->getDataObject()->getModuleType(target());

        if(type == MaceCore::ModuleClasses::VEHICLE_COMMS)
        {
            CommsMACE::Resource r;
            r.Set<CommsMACE::MACE_INSTANCE_STR, CommsMACE::VEHICLE_STR>(target().MaceInstance, target().ModuleID);
            m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg, r);
        }
        else if(type == MaceCore::ModuleClasses::GROUND_STATION)
        {
            CommsMACE::Resource r;
            r.Set<CommsMACE::MACE_INSTANCE_STR, CommsMACE::GROUNDSTATION_STR>(target().MaceInstance, target().ModuleID);
            m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg, r);
        }
        else if(type == MaceCore::ModuleClasses::ML_STATION)
        {
            CommsMACE::Resource r;
            r.Set<CommsMACE::MACE_INSTANCE_STR, CommsMACE::MLSTATION_STR>(target().MaceInstance, target().ModuleID);
            m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg, r);
        }
        else if(type == MaceCore::ModuleClasses::RTA)
        {
            CommsMACE::Resource r;
            r.Set<CommsMACE::MACE_INSTANCE_STR, CommsMACE::RTA_STR>(target().MaceInstance, target().ModuleID);
            m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg, r);
        }
        else if(type == MaceCore::ModuleClasses::EXTERNAL_LINK)
        {
            CommsMACE::Resource r;
            r.Set<CommsMACE::MACE_INSTANCE_STR, CommsMACE::EXTERNAL_LINK_STR>(target().MaceInstance, target().ModuleID);
            m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg, r);
        }
        else {
            throw std::runtime_error("Unknown target given");
        }
    }
}

MaceCore::ModuleCharacteristic ModuleExternalLink::GetKeyFromSecondaryID(int ID) const
{
    return this->getDataObject()->GetVehicleFromMAVLINKID(ID);
}

MaceCore::ModuleCharacteristic ModuleExternalLink::GetHostKey() const
{
    MaceCore::ModuleCharacteristic key;
    key.ModuleID = 0;
    key.MaceInstance = this->getParentMaceInstanceID();

    return key;
}

///////////////////////////////////////////////////////////////////////////////////////
/// The following are public virtual functions imposed from the Heartbeat Controller
/// Interface via callback functionality.
///////////////////////////////////////////////////////////////////////////////////////
void ModuleExternalLink::cbiHeartbeatController_transmitCommand(const mace_heartbeat_t &heartbeat)
{
    mace_message_t msg;
    mace_msg_heartbeat_encode_chan(this->associatedSystemID,0,m_LinkChan,&msg,&heartbeat);
    TransmitMessage(msg);
}


///////////////////////////////////////////////////////////////////////////////////////
/// The following are related to the Mission Items
///////////////////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::ReceivedMission(const MissionItem::MissionList &list)
{
    std::cout<<"The external link module now has received the entire mission."<<std::endl;

    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
        ptr->ExternalEvent_FinishedRXMissionList(this, list);
    });
}


Controllers::DataItem<MissionKey, MissionList>::FetchKeyReturn ModuleExternalLink::FetchMissionFromKey(const OptionalParameter<MissionKey> &key)
{
    if(key.IsSet() == false)
    {
        throw std::runtime_error("Key not set in fetchdatafromkey function");
    }

    Controllers::DataItem<MissionKey, MissionList>::FetchKeyReturn rtn;

    MissionList list;
    this->getDataObject()->getMissionList(key(), list);
    rtn.push_back(std::make_tuple(key(), list));
    return rtn;
}

Controllers::DataItem<MissionKey, MissionList>::FetchModuleReturn ModuleExternalLink::FetchAllMissionFromModule(const OptionalParameter<MaceCore::ModuleCharacteristic> &module)
{
    Controllers::DataItem<MissionKey, MissionList>::FetchModuleReturn rtn;


    //Function to fetch missions for given module
    auto func = [this, &rtn](MaceCore::ModuleCharacteristic vehicle)
    {
        //fetch mission
        MissionItem::MissionList currentMission;
        bool exists = this->getDataObject()->getCurrentMission(vehicle.ModuleID, currentMission);

        //if exists append, if it doesn't append empty
        if(exists)
        {
            MissionKey key = currentMission.getMissionKey();
            std::vector<std::tuple<MissionKey, MissionList>> vec = {};
            vec.push_back(std::make_tuple(key, currentMission));
            std::tuple<MaceCore::ModuleCharacteristic, std::vector<std::tuple<MissionKey, MissionList>>> tmp = std::make_tuple(vehicle, vec);
            rtn.push_back(tmp);
        }
        else
        {
            rtn.push_back(std::make_tuple(vehicle, std::vector<std::tuple<MissionKey, MissionList>>()));
        }
    };


    //if no module is given then fetch all
    if(module.IsSet() == false || module().ModuleID == 0)
    {
        std::vector<unsigned int> vehicles;
        this->getDataObject()->GetLocalVehicles(vehicles);
        for(auto it = vehicles.cbegin() ; it != vehicles.cend() ; ++it) {
            MaceCore::ModuleCharacteristic vehicle = this->getDataObject()->GetVehicleFromMAVLINKID(*it);
            func(vehicle);
        }
    }
    else {
        func(module());
    }


    return rtn;
}





void ModuleExternalLink::ReceivedHome(const MaceCore::ModuleCharacteristic &moduleAppliedTo, const command_item::SpatialHome &home)
{
    if(this->getDataObject()->HasModuleAsMavlinkID(moduleAppliedTo) == false)
    {
        printf("Received a home position from an unknown vehicle, IGNORING\n");
        RequestRemoteResources();
        return;
    }

    command_item::SpatialHome copy(home);

    uint8_t targetSystem;
    this->getDataObject()->getMavlinkIDFromModule(moduleAppliedTo, targetSystem);
    copy.setTargetSystem(targetSystem);

    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
        ptr->GVEvents_NewHomePosition(this, copy);
    });

    std::shared_ptr<MissionTopic::MissionHomeTopic> missionTopic = std::make_shared<MissionTopic::MissionHomeTopic>();
    missionTopic->setHome(copy);

    MaceCore::TopicDatagram topicDatagram;
    m_MissionDataTopic.SetComponent(missionTopic, topicDatagram);

    //notify listeners of topic
    ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_MissionDataTopic.Name(), moduleAppliedTo, MaceCore::TIME(), topicDatagram);
    });
}

Controllers::DataItem<MaceCore::ModuleCharacteristic, command_item::SpatialHome>::FetchKeyReturn ModuleExternalLink::FetchHomeFromKey(const OptionalParameter<MaceCore::ModuleCharacteristic> &key)
{
    if(key.IsSet() == false)
    {
        throw std::runtime_error("Key not set in fetchdatafromkey function");
    }

    Controllers::DataItem<MaceCore::ModuleCharacteristic, command_item::SpatialHome>::FetchKeyReturn rtn;

    command_item::SpatialHome home = this->getDataObject()->GetVehicleHomePostion(key().ModuleID);
    rtn.push_back(std::make_tuple(key(), home));
    return rtn;
}

Controllers::DataItem<MaceCore::ModuleCharacteristic, command_item::SpatialHome>::FetchModuleReturn ModuleExternalLink::FetchAllHomeFromModule(const OptionalParameter<MaceCore::ModuleCharacteristic> &module)
{
    Controllers::DataItem<MaceCore::ModuleCharacteristic, command_item::SpatialHome>::FetchModuleReturn rtn;


    //Function to fetch missions for given module
    auto func = [this, &rtn](MaceCore::ModuleCharacteristic vehicle)
    {
        //fetch mission
        command_item::SpatialHome home = this->getDataObject()->GetVehicleHomePostion(vehicle.ModuleID);

        std::vector<std::tuple<MaceCore::ModuleCharacteristic, command_item::SpatialHome>> vec = {};
        vec.push_back(std::make_tuple(vehicle, home));
        std::tuple<MaceCore::ModuleCharacteristic, std::vector<std::tuple<MaceCore::ModuleCharacteristic, command_item::SpatialHome>>> tmp = std::make_tuple(vehicle, vec);
        rtn.push_back(tmp);
    };


    //if no module is given then fetch all
    if(module.IsSet() == false || module().ModuleID == 0)
    {
        std::vector<unsigned int> vehicles;
        this->getDataObject()->GetLocalVehicles(vehicles);
        for(auto it = vehicles.cbegin() ; it != vehicles.cend() ; ++it) {
            MaceCore::ModuleCharacteristic vehicle = this->getDataObject()->GetVehicleFromMAVLINKID(*it);
            func(vehicle);
        }
    }
    else {
        func(module());
    }


    return rtn;
}

//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
void ModuleExternalLink::MACEMessage(const std::string &linkName, const mace_message_t &message)
{
    UNUSED(linkName);
    MaceCore::ModuleCharacteristic sender;
    sender.MaceInstance = message.sysid;
    sender.ModuleID = message.compid;


    /// Check if the sender is a known source
    ///   If it's from a generic MACE instance then check if its a known instance
    ///   If it's from a specific module, check if its a known module
    bool knownSource = false;
    if(sender.ModuleID == 0 && this->getDataObject()->KnownMaceInstance(sender.MaceInstance))
    {
        knownSource = true;
    }

    if(sender.ModuleID != 0 && this->getDataObject()->HasModule(sender) == true)
    {
        knownSource = true;
    }


    if(knownSource == false)
    {
        printf("Received a %d message but sending Module is unknown, IGNORING. Mace: %d Module: %d\n", message.msgid, sender.MaceInstance, sender.ModuleID);
        this->RequestRemoteResources();
        return;
    }


    m_Controllers.ForEach<Controllers::IController<mace_message_t, MaceCore::ModuleCharacteristic>>([sender, message](Controllers::IController<mace_message_t, MaceCore::ModuleCharacteristic>* ptr) {
       ptr->ReceiveMessage(&message, sender);
    });

    for(auto it = m_TopicToControllers.cbegin() ; it != m_TopicToControllers.cend() ; ++it)
    {
        it->second->ReceiveMessage(&message, sender);
    }

    this->ParseForData(&message);
}



void ModuleExternalLink::HeartbeatInfo(const MaceCore::ModuleCharacteristic &sender, const mace_heartbeat_t &heartbeatMSG)
{
    unsigned int systemID = heartbeatMSG.mavlinkID;

    CheckAndAddVehicle(sender, systemID);


    DataGenericItem::DataGenericItem_Heartbeat heartbeat;
    heartbeat.setAutopilot(static_cast<Data::AutopilotType>(heartbeatMSG.autopilot));
    heartbeat.setCompanion((heartbeatMSG.mace_companion>0)? true : false);
    heartbeat.setProtocol(static_cast<Data::CommsProtocol>(heartbeatMSG.protocol));
    heartbeat.setExecutionState(static_cast<Data::MissionExecutionState>(heartbeatMSG.mission_state));
    heartbeat.setType(static_cast<Data::SystemType>(heartbeatMSG.type));
    heartbeat.setMavlinkID(heartbeatMSG.mavlinkID);
    //heartbeat.setExecutionState(static_cast<Data::MissionExecutionState>(heartbeatMSG.missionState));
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> ptrHeartbeat = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>(heartbeat);

//    MaceLog::Alert("Heartbeat info received? _external link");
    PublishVehicleData(sender, ptrHeartbeat);
}

void ModuleExternalLink::PublishVehicleData(const MaceCore::ModuleCharacteristic &sender, const std::shared_ptr<Data::ITopicComponentDataObject> &component)
{
    //construct datagram
    MaceCore::TopicDatagram topicDatagram;
    m_VehicleDataTopic.SetComponent(component, topicDatagram);

    //notify listeners of topic
    ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), sender, MaceCore::TIME(), topicDatagram);
    });
}

void ModuleExternalLink::PublishMissionData(const MaceCore::ModuleCharacteristic &sender, const std::shared_ptr<Data::ITopicComponentDataObject> &component)
{
    //construct datagram
    MaceCore::TopicDatagram topicDatagram;
    m_MissionDataTopic.SetComponent(component, topicDatagram);

    //notify listeners of topic
    ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_MissionDataTopic.Name(), sender, MaceCore::TIME(), topicDatagram);
    });
}

///////////////////////////////////////////////////////////////////////////////////////
/// The following are public virtual functions imposed from IModuleCommandExternalLink.
//////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
/// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
/// command and action sequence that accompanies the vheicle. Expect an
/// acknowledgement or an event to take place when calling these items.
////////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::Command_SetGlobalOrigin(const Action_SetGlobalOrigin &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(command);
    UNUSED(sender);
}


void ModuleExternalLink::Command_ExecuteSpatialItem(const Action_ExecuteSpatialItem &goTo, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    unsigned int targetMavlinkSystemID = goTo.getTargetSystem();

    if(targetMavlinkSystemID == 0)
    {
        m_Controllers.Retreive<ExternalLink::Controller_GoTo>()->Broadcast(goTo, sender());
    }
    else {
        MaceCore::ModuleCharacteristic target = getDataObject()->GetVehicleFromMAVLINKID(targetMavlinkSystemID);
        m_Controllers.Retreive<ExternalLink::Controller_GoTo>()->Send(goTo, sender(), target);
    }
}

void ModuleExternalLink::Request_FullDataSync(const int &targetSystem, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    if(sender.IsSet() == false) {
        throw std::runtime_error("no sender given");
    }



    //This first segment causes all the topics to be republished
    mace_message_t msg;
    mace_vehicle_sync_t sync;
    sync.target_system = static_cast<uint8_t>(targetSystem);
    mace_msg_vehicle_sync_encode_chan(sender().MaceInstance, sender().ModuleID, m_LinkChan, &msg, &sync);

    if(targetSystem == 0)
    {
        TransmitMessage(msg);
    }
    else {
        MaceCore::ModuleCharacteristic target = this->getDataObject()->GetVehicleFromMAVLINKID(targetSystem);

        TransmitMessage(msg, target);

        m_Controllers.Retreive<ExternalLink::ControllerHome>()->Request(sender(), target);
    }
}

void ModuleExternalLink::Command_SystemArm(const command_item::ActionArm &systemArm, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    unsigned int targetMavlinkSystemID = systemArm.getTargetSystem();

    if(targetMavlinkSystemID == 0)
    {
        m_Controllers.Retreive<ExternalLink::CommandARM>()->Broadcast(systemArm, sender());
    }
    else {
        MaceCore::ModuleCharacteristic target = getDataObject()->GetVehicleFromMAVLINKID(targetMavlinkSystemID);
        m_Controllers.Retreive<ExternalLink::CommandARM>()->Send(systemArm, sender(), target);
    }
}

void ModuleExternalLink::Command_ChangeSystemMode(const command_item::ActionChangeMode &vehicleMode, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    std::cout<<"We are trying to change the mode in external link"<<std::endl;

    unsigned int targetMavlinkSystemID = vehicleMode.getTargetSystem();
    MaceCore::ModuleCharacteristic target = this->getDataObject()->GetVehicleFromMAVLINKID(targetMavlinkSystemID);
    m_Controllers.Retreive<ExternalLink::ControllerSystemMode>()->Send(vehicleMode, sender(), target);
}

void ModuleExternalLink::Command_VehicleTakeoff(const command_item::SpatialTakeoff &vehicleTakeoff, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    unsigned int targetMavlinkSystemID = vehicleTakeoff.getTargetSystem();

    if(targetMavlinkSystemID == 0)
    {
        m_Controllers.Retreive<ExternalLink::CommandTakeoff>()->Broadcast(vehicleTakeoff, sender());
    }
    else {
        MaceCore::ModuleCharacteristic target = this->getDataObject()->GetVehicleFromMAVLINKID(targetMavlinkSystemID);
        m_Controllers.Retreive<ExternalLink::CommandTakeoff>()->Send(vehicleTakeoff, sender(), target);
    }
}

void ModuleExternalLink::Command_Land(const command_item::SpatialLand &vehicleLand, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    unsigned int targetMavlinkSystemID = vehicleLand.getTargetSystem();

    if(targetMavlinkSystemID == 0)
    {
        m_Controllers.Retreive<ExternalLink::CommandLand>()->Broadcast(vehicleLand, sender());
    }
    else {
        MaceCore::ModuleCharacteristic target = this->getDataObject()->GetVehicleFromMAVLINKID(targetMavlinkSystemID);
        m_Controllers.Retreive<ExternalLink::CommandLand>()->Send(vehicleLand, sender(), target);
    }
}

void ModuleExternalLink::Command_ReturnToLaunch(const command_item::SpatialRTL &vehicleRTL, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    if(vehicleRTL.getTargetSystem() == 0)
    {
        m_Controllers.Retreive<ExternalLink::CommandRTL>()->Broadcast(vehicleRTL, sender());
    }
    else {
        MaceCore::ModuleCharacteristic target = this->getDataObject()->GetVehicleFromMAVLINKID(vehicleRTL.getTargetSystem());
        m_Controllers.Retreive<ExternalLink::CommandRTL>()->Send(vehicleRTL, sender(), target);
    }
}

void ModuleExternalLink::Command_MissionState(const command_item::ActionMissionCommand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    MaceCore::ModuleCharacteristic target = this->getDataObject()->GetVehicleFromMAVLINKID(command.getTargetSystem());

    if(target.ModuleID == 0)
    {
        m_Controllers.Retreive<ExternalLink::CommandMissionItem>()->Broadcast(command, sender());
    }
    else {
        m_Controllers.Retreive<ExternalLink::CommandMissionItem>()->Send(command, sender(), target);
    }
}

void ModuleExternalLink::Command_IssueGeneralCommand(const std::shared_ptr<command_item::AbstractCommandItem> &command)
{
    UNUSED(command);
}

void ModuleExternalLink::Command_ExecuteDynamicTarget(const command_item::Action_DynamicTarget &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(command);
    UNUSED(sender);
}


void ModuleExternalLink::Command_EmitHeartbeat(const command_item::SpatialTakeoff &heartbeat)
{
    UNUSED(heartbeat);
}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way ardupilot works.
/////////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::Command_GetHomePosition(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    std::cout<<"External link module saw a get home position request"<<std::endl;

    MaceCore::ModuleCharacteristic target = this->getDataObject()->GetVehicleFromMAVLINKID(vehicleID);

    m_Controllers.Retreive<ExternalLink::ControllerHome>()->Request(sender(), target);
}

void ModuleExternalLink::Command_SetHomePosition(const command_item::SpatialHome &systemHome, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    MaceCore::ModuleCharacteristic target = this->getDataObject()->GetVehicleFromMAVLINKID(systemHome.getTargetSystem());

    m_Controllers.Retreive<ExternalLink::ControllerHome>()->Send(systemHome, sender(), target);
}

/////////////////////////////////////////////////////////////////////////
/// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
/// This functionality may be pertinent for vehicles not containing a
/// direct MACE hardware module.
/////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::Command_UploadMission(const MissionItem::MissionList &missionList)
{
    MissionItem::MissionList::MissionListStatus status = missionList.getMissionListStatus();

//    MaceCore::ModuleCharacteristic target = this->getDataObject()->GetVehicleFromMAVLINKID(missionList.getVehicleID());

    if(status.state == MissionItem::MissionList::COMPLETE)
    {
        throw std::runtime_error("Not Implimented");
        //m_Controllers.Retreive<ExternalLink::ControllerMission>()->UploadMission(missionList, target);
    }
}

void ModuleExternalLink::Command_GetMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    if(sender.IsSet() == false) {
        throw std::runtime_error("no sender given");
    }

    MaceCore::ModuleCharacteristic target = this->getDataObject()->GetVehicleFromMAVLINKID(key.m_systemID);

    UNUSED(key);
    m_Controllers.Retreive<ExternalLink::ControllerMission>()->RequestMission(key, sender(), target);
}


void ModuleExternalLink::Command_RequestBoundaryDownload(const std::tuple<MaceCore::ModuleCharacteristic, uint8_t> &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    std::cout << "External link is making a request to a remote instance" << std::endl;

    if(sender.IsSet() == false)
    {
        throw std::runtime_error("Sender must be set when requesting a boundary download on external link");
    }

    MaceCore::ModuleCharacteristic target = std::get<0>(data);
    uint8_t ID = std::get<1>(data);

    m_Controllers.Retreive<ExternalLink::ControllerBoundary>()->RequestBoundary(ID, sender(), target);
}

void ModuleExternalLink::Command_SetCurrentMission(const MissionItem::MissionKey &key)
{
    UNUSED(key);

    //    mace_message_t msg;
    //    mace_mission_set_current_t request;
    //    request.
    //    request.mission_creator = key.m_creatorID;
    //    request.mission_id = key.m_missionID;
    //    request.mission_type = (uint8_t)key.m_missionType;
    //    request.target_system = key.m_systemID;
    //    mace_msg_mission_set_current_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&request);
    //    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::Command_GetCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);

}

void ModuleExternalLink::Command_ClearCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::Command_GetOnboardAuto(const int &targetSystem)
{
    UNUSED(targetSystem);

}

void ModuleExternalLink::Command_ClearOnboardAuto(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::Command_GetOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::Command_ClearOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}


//!
//! \brief A new boundary item is available. Notify all known mace instances
//! \param key Local key of boundary. Identifies the boundary on the host machine
//! \param sender Module that generated the boundary
//!
void ModuleExternalLink::NewlyAvailableBoundary(const uint8_t &boundaryKey, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    BoundaryNotificationData newBoundary;
    this->getDataObject()->getCharactersticFromIdentifier(boundaryKey, newBoundary.characteristic);
    newBoundary.uniqueIdentifier = boundaryKey;

    std::vector<MaceCore::ModuleCharacteristic> modules = this->getDataObject()->GetAllRemoteModules();

    std::set<unsigned int> uniqueInstances;
    std::vector<MaceCore::ModuleCharacteristic> uniqueInstanceTargets;
    for(auto it = modules.cbegin() ; it != modules.cend() ; ++it)
    {
        if(uniqueInstances.find(it->MaceInstance) == uniqueInstances.cend()
                && it->MaceInstance != this->getParentMaceInstanceID())
        {
            uniqueInstances.insert(it->MaceInstance);

            ///////////////////
            /// broadcast to instance
            uint8_t remoteMACEInstance = it->MaceInstance;

            MaceCore::ModuleCharacteristic target;
            target.MaceInstance = remoteMACEInstance;
            target.ModuleID = 0;

            uniqueInstanceTargets.push_back(target);
        }
    }
    m_Controllers.Retreive<ExternalLink::ControllerBoundary>()->BroadcastReliable(newBoundary, sender(), uniqueInstanceTargets);
}

void ModuleExternalLink::NewlyAvailableOnboardMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    std::vector<MaceCore::ModuleCharacteristic> modules = this->getDataObject()->GetAllRemoteModules();

    std::set<unsigned int> uniqueInstances;
    for(auto it = modules.cbegin() ; it != modules.cend() ; ++it)
    {
        if(uniqueInstances.find(it->MaceInstance) == uniqueInstances.cend())
        {
            uniqueInstances.insert(it->MaceInstance);

            ///////////////////
            /// broadcast to instance
            unsigned int remoteMACEInstance = it->MaceInstance;

            MaceCore::ModuleCharacteristic target;
            target.MaceInstance = remoteMACEInstance;
            target.ModuleID = 0;

            /// only send if the remote mace instance is known
            /// This is needed because the module may of been notified, but the resouce-indiation message for the instance itself hasn't made it yet
            CommsMACE::Resource r;
            r.Add(CommsMACE::MACE_INSTANCE_STR, remoteMACEInstance);
            if(this->m_LinkMarshaler->HasResource(m_LinkName, r) == true)
            {
                m_Controllers.Retreive<ExternalLink::ControllerMission>()->NotifyOfMission(key, sender(), target);
            }
            else
            {
                printf("MACE Instance is unknown, Requesting for remote resources\n");

                this->m_LinkMarshaler->RequestRemoteResources(m_LinkName, {r});
            }
        }
    }
}

void ModuleExternalLink::NewlyAvailableHomePosition(const command_item::SpatialHome &home, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    m_Controllers.Retreive<ExternalLink::ControllerHome>()->Broadcast(home, sender());
}

//Ken FIX THIS: I dont know if I should pass the pertinent systemID with the key
void ModuleExternalLink::NewlyAvailableMissionExeState(const MissionItem::MissionKey &key)
{
    MissionItem::MissionList list;
    bool validity = this->getDataObject()->getMissionList(key,list);
    if(validity)
    {
        mace_mission_exe_state_t state;
        Data::MissionExecutionState missionState = list.getMissionExeState();
        state.mission_creator = static_cast<uint8_t>(key.m_creatorID);
        state.mission_id = static_cast<uint8_t>(key.m_missionID);
        state.mission_state = static_cast<uint8_t>(missionState);
        state.mission_system = static_cast<uint8_t>(key.m_systemID);
        state.mission_type = static_cast<uint8_t>(key.m_missionType);

        mace_message_t msg;
        mace_msg_mission_exe_state_encode_chan(static_cast<uint8_t>(key.m_systemID),0,m_LinkChan,&msg,&state);
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
    }
}

void ModuleExternalLink::NewlyAvailableModule(const MaceCore::ModuleCharacteristic &module, const MaceCore::ModuleClasses &type)
{
    CommsMACE::Resource MACEInstanceResource;
    MACEInstanceResource.Set<CommsMACE::MACE_INSTANCE_STR>(module.MaceInstance);
    if(m_LinkMarshaler->HasResource(m_LinkName, MACEInstanceResource) == false)
    {
        m_LinkMarshaler->AddResource(m_LinkName, MACEInstanceResource);
    }


    CommsMACE::Resource resource;
    if(type == MaceCore::ModuleClasses::VEHICLE_COMMS)
    {
        resource.Set<CommsMACE::MACE_INSTANCE_STR, CommsMACE::VEHICLE_STR>(module.MaceInstance, module.ModuleID);
    }
    if(type == MaceCore::ModuleClasses::GROUND_STATION)
    {
        resource.Set<CommsMACE::MACE_INSTANCE_STR, CommsMACE::GROUNDSTATION_STR>(module.MaceInstance, module.ModuleID);
    }
    if(type == MaceCore::ModuleClasses::ML_STATION)
    {
        resource.Set<CommsMACE::MACE_INSTANCE_STR, CommsMACE::MLSTATION_STR>(module.MaceInstance, module.ModuleID);
    }
    if(type == MaceCore::ModuleClasses::RTA)
    {
        resource.Set<CommsMACE::MACE_INSTANCE_STR, CommsMACE::RTA_STR>(module.MaceInstance, module.ModuleID);
    }
    if(type == MaceCore::ModuleClasses::EXTERNAL_LINK)
    {
        resource.Set<CommsMACE::MACE_INSTANCE_STR, CommsMACE::EXTERNAL_LINK_STR>(module.MaceInstance, module.ModuleID);
    }


    m_LinkMarshaler->AddResource(m_LinkName, resource);

    if(airborneInstance)
    {
        this->associatedSystemID = module.ModuleID;

        //this function should always be called by an external link connected to ground for now
        //KEN this is a hack...but it will function for now
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->ExternalEvent_UpdateRemoteID(this, 254);
        });
    }
}

void ModuleExternalLink::ReceivedMissionACK(const MissionItem::MissionACK &ack)
{
    mace_mission_ack_t missionACK;
    MissionItem::MissionKey key = ack.getMissionKey();
    missionACK.cur_mission_state = static_cast<uint8_t>(ack.getNewMissionState());
    missionACK.mission_creator = static_cast<uint8_t>(key.m_creatorID);
    missionACK.mission_id = static_cast<uint8_t>(key.m_missionID);
    missionACK.mission_result = static_cast<uint8_t>(ack.getMissionResult());
    missionACK.mission_system = static_cast<uint8_t>(key.m_systemID);
    missionACK.mission_type = static_cast<uint8_t>(key.m_missionType);
    missionACK.prev_mission_state = static_cast<uint8_t>(key.m_missionState);

    mace_message_t msg;
    mace_msg_mission_ack_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&missionACK);
    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
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
void ModuleExternalLink::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{

    if(this->m_TopicToControllers.find(topicName) == m_TopicToControllers.cend())
    {
        throw std::runtime_error("Attempting to send a topic that external link has no knowledge of");
    }

    Controllers::IController<mace_message_t, MaceCore::ModuleCharacteristic> *controller = m_TopicToControllers.at(topicName);
    if(target.IsSet())
    {
        if(controller->ContainsAction(Controllers::Actions::SEND) == false)
        {
            throw std::runtime_error("Attempting to send a topic to a controller that has no send action");
        }
        Controllers::IActionSend<MaceCore::TopicDatagram, MaceCore::ModuleCharacteristic> *sendAction = dynamic_cast<Controllers::IActionSend<MaceCore::TopicDatagram, MaceCore::ModuleCharacteristic>*>(controller);
        sendAction->Send(data, sender, target());
    }
    else {
        if(controller->ContainsAction(Controllers::Actions::BROADCAST) == false)
        {
            throw std::runtime_error("Attempting to broadcast a topic to a controller that has no broadcast action");
        }
        Controllers::IActionBroadcast<MaceCore::TopicDatagram, MaceCore::ModuleCharacteristic> *broadcastAction = dynamic_cast<Controllers::IActionBroadcast<MaceCore::TopicDatagram, MaceCore::ModuleCharacteristic>*>(controller);
        if (broadcastAction)
            broadcastAction->Broadcast(data, sender);
        else
        {
            std::vector<MaceCore::ModuleCharacteristic> remoteModules = this->getDataObject()->GetAllRemoteModules();
            std::vector<MaceCore::ModuleCharacteristic> targets;
            MaceCore::ModuleCharacteristic nextTarget;

            std::set<unsigned int> maceInstances;
            for(auto it = remoteModules.cbegin() ; it != remoteModules.cend() ; ++it)
            {

                if(maceInstances.find(it->MaceInstance) == maceInstances.cend())
                {
                    maceInstances.insert(it->MaceInstance);

                    nextTarget.MaceInstance = it->MaceInstance;
                    nextTarget.ModuleID = 0;

                    targets.push_back(nextTarget);
                }
            }

            Controllers::IActionBroadcastReliable<MaceCore::TopicDatagram, MaceCore::ModuleCharacteristic> *broadcastReliableAction
                    = dynamic_cast<Controllers::IActionBroadcastReliable<MaceCore::TopicDatagram, MaceCore::ModuleCharacteristic>*>(controller);
            broadcastReliableAction->BroadcastReliable(data, sender, targets);
        }
    }
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
void ModuleExternalLink::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(target);

    uint8_t senderID;
    if(!this->getDataObject()->getMavlinkIDFromModule(sender, senderID)) {
        senderID = 0;
    }
    if(airborneInstance)
    {

        //In relevance to the external link module, the module when receiving a new topic should pack that up for transmission
        //to other instances of MACE
        //example read of vehicle data
        if(topicName == m_VehicleDataTopic.Name())
        {
            //get latest datagram from mace_data
            MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), sender);
            //example of how to get data and parse through the components that were updated
            for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
                if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Heartbeat::Name()) {
                    if(!loggerCreated)
                        createLog(senderID);
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(static_cast<uint8_t>(sender.MaceInstance), static_cast<uint8_t>(sender.ModuleID), m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_GPS::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(static_cast<uint8_t>(sender.MaceInstance), static_cast<uint8_t>(sender.ModuleID), m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_FlightMode::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(static_cast<uint8_t>(sender.MaceInstance), static_cast<uint8_t>(sender.ModuleID), m_LinkChan);

                    MaceLog::Alert("Vehicle mode external link: " + component->getFlightModeString());
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_SystemArm::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(static_cast<uint8_t>(sender.MaceInstance), static_cast<uint8_t>(sender.ModuleID), m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == mace::pose_topics::Topic_AgentOrientation::Name()) {
                    mace::pose_topics::Topic_AgentOrientationPtr component = std::make_shared<mace::pose_topics::Topic_AgentOrientation>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getRotationObj()->getMACEMsg(static_cast<uint8_t>(sender.MaceInstance), static_cast<uint8_t>(sender.ModuleID), m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == mace::pose_topics::Topic_GeodeticPosition::Name()) {
                    mace::pose_topics::Topic_GeodeticPositionPtr component = std::make_shared<mace::pose_topics::Topic_GeodeticPosition>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getPositionObj()->getMACEMsg(static_cast<uint8_t>(sender.MaceInstance), static_cast<uint8_t>(sender.ModuleID), m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Battery::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(static_cast<uint8_t>(sender.MaceInstance), static_cast<uint8_t>(sender.ModuleID), m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Text::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(static_cast<uint8_t>(sender.MaceInstance), static_cast<uint8_t>(sender.ModuleID), m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
            }
        } //end of vehicle data topic
        else if(topicName == m_MissionDataTopic.Name())
        {
            for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
                //get latest datagram from mace_data
                MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_MissionDataTopic.Name(), sender);
                if(componentsUpdated.at(i) == MissionTopic::MissionItemCurrentTopic::Name()) {
                    std::shared_ptr<MissionTopic::MissionItemCurrentTopic> component = std::make_shared<MissionTopic::MissionItemCurrentTopic>();
                    m_MissionDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(static_cast<uint8_t>(sender.MaceInstance), static_cast<uint8_t>(sender.ModuleID), m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == MissionTopic::MissionItemReachedTopic::Name()) {
                    std::shared_ptr<MissionTopic::MissionItemReachedTopic> component = std::make_shared<MissionTopic::MissionItemReachedTopic>();
                    m_MissionDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(static_cast<uint8_t>(sender.MaceInstance), static_cast<uint8_t>(sender.ModuleID), m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == MissionTopic::VehicleTargetTopic::Name()) {
                    std::shared_ptr<MissionTopic::VehicleTargetTopic> component = std::make_shared<MissionTopic::VehicleTargetTopic>();
                    m_MissionDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = component->getMACEMsg(static_cast<uint8_t>(sender.MaceInstance), static_cast<uint8_t>(sender.ModuleID), m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
            }
        }
    }//end of if airborne instance

}


//!
//! \brief Function to be called when a new remote boundary characteristic has been recevied by the controller
//! \param sender Module that contains the boundary
//! \param data Data about boundary
//!
void ModuleExternalLink::ReceivedRemoteBoundaryCharacterstic(const MaceCore::ModuleCharacteristic &remoteModule, const uint8_t remoteBoundaryID, const BoundaryItem::BoundaryCharacterisic &characteristic)
{
    m_RemoteBoundaries.insert({ModuleBoundaryIdentifier(remoteModule, remoteBoundaryID), characteristic});
    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
        ptr->ExternalEvent_NewBoundary(this, MaceCore::NewBoundaryData(remoteModule, remoteBoundaryID, characteristic));
    });
}


Controllers::DataItem<ModuleBoundaryIdentifier, BoundaryItem::BoundaryList>::FetchKeyReturn ModuleExternalLink::FetchBoundaryFromKey(const OptionalParameter<ModuleBoundaryIdentifier> &key)
{
    if(key.IsSet() == false)
    {
        throw std::runtime_error("Key not set in FetchBoundaryFromKey function");
    }

    std::cout << "Fetch boundary from key **** " << std::endl;

    Controllers::DataItem<ModuleBoundaryIdentifier, BoundaryItem::BoundaryList>::FetchKeyReturn rtn;

    BoundaryItem::BoundaryList boundary;
    this->getDataObject()->getBoundaryFromIdentifier(key().BoundaryIdentifier(), boundary);
    rtn.push_back(std::make_tuple(key(), boundary));

    return rtn;
}

void ModuleExternalLink::ReceivedRemoteBoundary(const MaceCore::ModuleCharacteristic &remoteModule, uint8_t remoteBoundaryID, const BoundaryItem::BoundaryList &list)
{
    BoundaryItem::BoundaryCharacterisic characteristic = m_RemoteBoundaries.at(ModuleBoundaryIdentifier(remoteModule, remoteBoundaryID));
    std::cout<<"The external link module now has received the entire boundary with " << list.getQueueSize() << " verticies" << std::endl;

    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
        ptr->Event_SetBoundary(this, characteristic, list);
    });
}


void ModuleExternalLink::ReceivedRemoteMissionNotification(const MaceCore::ModuleCharacteristic &remoteModule, const MissionItem::MissionKey &key)
{
    CheckAndAddVehicle(remoteModule, key.m_systemID);

    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
        ptr->ExternalEvent_NewOnboardMission(this, key);
    });
}


//!
//! \brief Procedure to perform when a new MACE instance is added.
//!
//! This method will consult with data object and send out any relevant data to the remote MACE instance
//!
//! \param MaceInstanceID ID of new mace instance.
//!
void ModuleExternalLink::NewExternalMaceInstance(uint8_t maceInstanceID)
{
    printf("New MACE instance with ID of %d detected\n", maceInstanceID);

    MaceCore::ModuleCharacteristic target;
    target.MaceInstance = maceInstanceID;
    target.ModuleID = 0;

    /// Iterate over every local vehicle and do tasks on them
    std::vector<unsigned int> localVehicles = this->getDataObject()->GetLocalVehicles();
    for(auto localVehicle_it = localVehicles.cbegin() ; localVehicle_it != localVehicles.cend() ; ++localVehicle_it)
    {
        /// Local vehicle
        unsigned int vehicleID = *localVehicle_it;
        MaceCore::ModuleCharacteristic sender = this->getDataObject()->GetVehicleFromMAVLINKID(vehicleID);


        /// What to send about the new vehicle to the new MACE instance
        auto func = [this, vehicleID, sender, target]()
        {
            std::vector<MissionItem::MissionKey> internalMissions = this->getDataObject()->getMissionKeysForVehicle(vehicleID);

            for(auto it = internalMissions.cbegin() ; it != internalMissions.cend() ; ++it)
            {
                m_Controllers.Retreive<ExternalLink::ControllerMission>()->NotifyOfMission(*it, sender, target);
            }
        };


        /// Check if this can execute now or if we need to wait until we learn about the module in question.
        if(this->getDataObject()->HasModule(sender) == true)
        {
            func();
        }
        else
        {
            printf("Module is not online yet, add as tasks to perform when online\n");
            if(m_TasksToDoWhenModuleComesOnline.find(sender) == m_TasksToDoWhenModuleComesOnline.cend())
            {
                m_TasksToDoWhenModuleComesOnline.insert({sender, {}});
            }
            m_TasksToDoWhenModuleComesOnline[sender].push_back(func);
        }
    }
}


//!
//! \brief Check if the given systemID is known. If unknown then do steps to add the vehicle to this MACE instance
//! \param sender
//! \param systemID
//!
void ModuleExternalLink::CheckAndAddVehicle(const MaceCore::ModuleCharacteristic &sender, unsigned int systemID)
{
    if(systemIDMap.find(systemID) == systemIDMap.end())
    {
        //this function should always be called by an external link connected to ground for now
        //KEN this is a hack...but it will function for now
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->ExternalEvent_UpdateRemoteID(this, systemID);
        });

        if(!loggerCreated) {
            std::string loggerName = createLog(systemID);
        }
        systemIDMap.insert({systemID,0});

        //The system has yet to have communicated through this module
        //We therefore have to notify the core that there is a new vehicle
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->Events_NewVehicle(this, systemID, sender);
        });

        //Request_FullDataSync(systemID);
    }
}
