#ifndef CONFIGURATIONREADER_XML_H
#define CONFIGURATIONREADER_XML_H

#include "mace_core/module_parameters.h"

#include "mace_core/abstract_module_base.h"

#include <memory>

#include "mace_core/module_factory.h"

class ConfigurationParseResult
{
public:
    ConfigurationParseResult() :
        success(true)
    {

    }

    ConfigurationParseResult(const std::string &errorStr) :
        success(false),
        error(errorStr)
    {

    }

    bool success;
    std::string error;
    std::vector<std::string> warnings;
};

class ConfigurationReader_XML
{
public:
    ConfigurationReader_XML(const MaceCore::ModuleFactory *factory);


    //!
    //! \brief Parse an given XML file
    //!
    //! \param filename Filename to parse
    //! \return True if parsing was success
    //!
    ConfigurationParseResult Parse(const std::string &filename);

    //!
    //! \brief Confirm that MACE Instance ID has been set
    //! \return True if ID has been set
    //!
    bool HasStaticMaceInstanceID() const;

    //!
    //! \brief Get MACE instance ID
    //! \return MACE instance ID
    //!
    uint32_t GetStaticMaceInstanceID() const;


    //!
    //! \brief Get modules created after parsing
    //! \return List of created modules.
    //!
    std::map<std::shared_ptr<MaceCore::ModuleBase>, std::string> GetCreatedModules() const;


    //!
    //! \brief Get the configuration for a module after parse
    //!
    //! Must be called after Parse is called and returns with a value of true
    //! \param module Pointer to module to get configuration of
    //! \return Configuration for module
    //!
    std::shared_ptr<MaceCore::ModuleParameterValue> GetModuleConfiguration(const std::shared_ptr<MaceCore::ModuleBase> &module);
    //!
    //! \brief Get the global configuration parameters after parse
    //!
    //! Must be called after Parse is called and returns with a value of true
    //! \param module Pointer to module to get configuration of
    //! \return Global Configuration
    //!
    std::shared_ptr<MaceCore::ModuleParameterValue> GetGlobalConfiguration();


private:

    const MaceCore::ModuleFactory *m_Factory;

    std::vector<std::shared_ptr<MaceCore::ModuleBase> > m_Modules;

    bool m_MaceInstanceIDSet;
    uint32_t m_MaceInstance;

    std::map<std::shared_ptr<MaceCore::ModuleBase>, std::string> m_ModuleTypes;
    std::map<std::shared_ptr<MaceCore::ModuleBase>, std::shared_ptr<MaceCore::ModuleParameterValue> > m_Parameters;
    std::shared_ptr<MaceCore::ModuleParameterValue> m_globalParameters;

    //!
    //! \brief Describes the structure of the global parameters
    //! \return Global Parameter Structure
    //!
    std::shared_ptr<MaceCore::ModuleParameterStructure> GetGlobalParamStructure();
};

#endif // CONFIGURATIONREADER_XML_H
