#ifndef MODULE_INTERFACE_H
#define MODULE_INTERFACE_H

#include <string>
#include <vector>
#include <functional>

#include "mace_core_global.h"

#include "mace_data.h"

#include "module_parameters.h"

#include "topic.h"

#include "module_characteristics.h"

#define BASE_MODULE_LISTENER_ENUMS UPDATE_GLOBAL_ORIGIN

namespace MaceCore
{

//!
//! \brief Base abstract class for a Module
//!
class ModuleBase
{
public:
    virtual ~ModuleBase() = default;

public:

    //!
    //! \brief ModuleTypeToString Get the corresponding string from module type
    //! \param type Module type
    //! \return Module type string
    //!
    static std::string ModuleTypeToString(const ModuleClasses &type)
    {
        switch (type) {
        case ModuleClasses::EXTERNAL_LINK:
            return "ExternalLink";
        case ModuleClasses::GROUND_STATION:
            return "GroundStation";
        case ModuleClasses::PATH_PLANNING:
            return "PathPlanning";
        case ModuleClasses::ROS:
            return "ROS";
        case ModuleClasses::RTA:
            return "RTA";
        case ModuleClasses::SENSORS:
            return "Sensors";
        case ModuleClasses::VEHICLE_COMMS:
            return "VehicleComms";
        default:
            throw std::runtime_error("Unknown module type");
        }
    }

    //!
    //! \brief StringToModuleClass Convert a string to module class enum
    //! \param string Module string
    //! \return Module class enum
    //!
    static ModuleClasses StringToModuleClass(const std::string &string)
    {
        if(string == "ExternalLink")
            return ModuleClasses::EXTERNAL_LINK;
        if(string == "GroundStation")
            return ModuleClasses::GROUND_STATION;
        if(string == "PathPlanning")
            return ModuleClasses::PATH_PLANNING;
        if(string == "ROS")
            return ModuleClasses::ROS;
        if(string == "RTA")
            return ModuleClasses::RTA;
        if(string == "Sensors")
            return ModuleClasses::SENSORS;
        if(string == "VehicleComms")
            return ModuleClasses::VEHICLE_COMMS;
        throw std::runtime_error("Unknown module type");
    }

public:

    ModuleBase() :
        m_HasID(false),
        m_Started(false)
    {

    }

    const static ModuleClasses moduleClass;

    virtual ModuleClasses ModuleClass() const = 0;

    void SetID(int ID)
    {
        m_ID = ID;
        m_HasID = true;
    }

    //!
    //! \brief Determine if ID has ben set.
    //!
    //! Used at startup when some modules may have a "static" ID while others will be dynamically assinged
    //! \return True if module has an ID assigned to it.
    //!
    bool HasID() const
    {
        return m_HasID;
    }

    int GetID() const
    {
        return m_ID;
    }

    ModuleCharacteristic GetCharacteristic() const
    {
        ModuleCharacteristic obj;
        obj.ModuleID = m_ID;
        //obj.Class = ModuleClass();
        obj.MaceInstance = this->getParentMaceInstanceID();
        return obj;
    }


    //!
    //! \brief function that is to kick off the event loop of the module
    //!
    virtual void start() = 0;


    //!
    //! \brief Shutdown the module
    //!
    virtual void shutdown() = 0;


    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<ModuleParameterStructure> ModuleConfigurationStructure() const = 0;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<ModuleParameterValue> &params) = 0;

    //!
    //! \brief OnModulesStarted Fired when all modules have started, indicates the core is ready to marshal data
    //!
    virtual void OnModulesStarted()
    {
        m_Started = true;
    }

    //!
    //! \brief AssignLoggingDirectory
    //! \param path
    //!
    virtual void AssignLoggingDirectory(const std::string &path)
    {
        this->loggingPath = path;
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
    virtual void NewTopicData(const std::string &topicName, const ModuleCharacteristic &sender, const TopicDatagram &data, const OptionalParameter<ModuleCharacteristic> &target = OptionalParameter<ModuleCharacteristic>()) = 0;


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
    virtual void NewTopicSpooled(const std::string &topicName, const ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<ModuleCharacteristic> &target = OptionalParameter<ModuleCharacteristic>()) = 0;

    //!
    //! \brief Get all topics that are to be emited by this module
    //! \return List of topics
    //!
    virtual std::vector<TopicCharacteristic> GetEmittedTopics()
    {
        //TODO - Make pure virtual
        return {};
    }

    //!
    //! \brief setDataObject Replace the data object
    //! \param data New data object to set
    //!
    void setDataObject(const std::shared_ptr<MaceData> &data)
    {
        m_Data = data;
    }

    //!
    //! \brief getDataObject Get the data object
    //! \return Data object
    //!
    std::shared_ptr<const MaceData> getDataObject() const
    {
        return m_Data;
    }


    //!
    //! \brief Set the host MACE instance ID
    //! \param ID identifier for host MACE instance
    //!
    void setParentMaceInstanceID(const uint32_t &ID)
    {
        m_ParentMaceInstanceIDSet = true;
        m_ParentMaceInstanceID = ID;
    }


    //!
    //! \brief Get the host MACE instance ID
    //! \throws std::runtime_error Thrown if no ID has been set.
    //! \return Identifier for host MACE instance
    //!
    uint32_t getParentMaceInstanceID() const
    {
        if(m_ParentMaceInstanceIDSet == false)
        {
            throw std::runtime_error("No Parent ID Set");
        }
        return m_ParentMaceInstanceID;
    }

protected:


    //!
    //! \brief ModuleStarted Determine if the MACE has indicated that the module is capable of running
    //! \return True if MACE instance has indicated the module is good to go
    //!
    bool ModuleStarted() const
    {
        return m_Started;
    }


protected:
    std::string loggingPath;
    bool loggerCreated = false;

private:
    std::shared_ptr<const MaceData> m_Data;

    bool m_HasID;
    int m_ID;

    ///MTB MODULE AUTO ASSIGN
    bool m_ParentMaceInstanceIDSet = false;
    uint32_t m_ParentMaceInstanceID;
    ///

    //!
    //! \brief Variable to indicate if the MACE instance is ready for this module to start it's processing
    //!
    bool m_Started;
};


} //End MaceCore Namespace


#endif // MODULE_INTERFACE_H
