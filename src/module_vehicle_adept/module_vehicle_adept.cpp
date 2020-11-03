#include "module_vehicle_adept.h"

ModuleVehicleAdept::ModuleVehicleAdept()
{
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleVehicleAdept::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    UNUSED(ptr);
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleVehicleAdept::ModuleConfigurationStructure() const
{
     MaceCore::ModuleParameterStructure structure;
     return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleVehicleAdept::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    UNUSED(params);
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
void ModuleVehicleAdept::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
 UNUSED(topicName); UNUSED(sender); UNUSED(data); UNUSED(target);
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
void ModuleVehicleAdept::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(topicName); UNUSED(sender); UNUSED(componentsUpdated); UNUSED(target);

//    // listen for all my data, add as I receive:
//    if(componentsUpdated.at(i) == mace::pose_topics::Topic_GeodeticPosition::Name()) {
//        std::shared_ptr<mace::pose_topics::Topic_GeodeticPosition> component = std::make_shared<mace::pose_topics::Topic_GeodeticPosition>();
//        m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

//        // Write Position data to the GUI:
//        writeToFrame_Pos(component->getPositionObj());
//        m_toGUIHandler->sendPositionData(vehicleID, component);
//    }
}

//!
//! \brief receiveFrame Gets new frame data from Adept agent
//!
void ModuleVehicleAdept::receiveFrame()
{

}

//!
//! \brief NewlyAvailableVehicle Subscriber to a newly available vehicle topic
//! \param vehicleID Vehilce ID of the newly available vehicle
//!
void ModuleVehicleAdept::NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(vehicleID); UNUSED(sender);
}

//!
//! \brief NewlyAvailableFrame Subscriber to a new frame
//!
void ModuleVehicleAdept::NewlyAvailableFrame(const uint8_t &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(key); UNUSED(sender);
}

//!
//! \brief createLog Creates a new log to record flight data passing to/from adept agent
//!
void ModuleVehicleAdept::createLog()
{

}
