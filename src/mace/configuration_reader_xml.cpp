#include "configuration_reader_xml.h"

#include "pugixml.hpp"
#include "mace_core/module_factory.h"




ConfigurationReader_XML::ConfigurationReader_XML(const MaceCore::ModuleFactory *factory) :
    m_Factory(factory),
    m_MaceInstanceIDSet(false)
{

}


//!
//! \brief Parse all Parameter tags from an XML Node
//! \param node XML node that contains "Parameter" tags
//! \param structure Structure that parsing is expecting for this node
//! \param superParameter Name of nested parameter that is being parsed, used for printouts, empty if top root.
//! \return Resulting ModuleParameterValue object
//!
static std::shared_ptr<MaceCore::ModuleParameterValue> ParseParameters(const pugi::xml_node &node, const std::shared_ptr<MaceCore::ModuleParameterStructure> structure, ConfigurationParseResult &result, const std::string &superParameter = "")
{
    std::shared_ptr<MaceCore::ModuleParameterValue> valueContainer = std::make_shared<MaceCore::ModuleParameterValue>();


    //prepare map of all expected paramters, to be set when they are seen
    std::unordered_map<std::string, bool> seenTerminals;
    std::unordered_map<std::string, bool> nonTerminalsPresentMap;
    std::vector<std::string> seenNonTerminals;
    for(std::string item : structure->getNonTerminalNames())
    {
        nonTerminalsPresentMap.insert({item, false});
    }
    for(std::string item : structure->getTerminalNames())
    {
        seenTerminals.insert({item, false});
    }


    //loop through all "Parameter" Tags
    for (pugi::xml_node parameter = node.child("Parameter"); parameter; parameter = parameter.next_sibling("Parameter"))
    {
        std::string parameterName = parameter.attribute("Name").as_string();

        std::string nestedName = "";
        if(superParameter != "")
            nestedName += superParameter + ".";
        nestedName += parameterName;

        if(structure->TerminalExists(parameterName) == true)
        {
            std::string terminalStringValue = std::string(parameter.child_value());

            std::vector<std::string> allowedValues = structure->getTerminalAllowedEntires(parameterName);
            bool isAllowed;
            if(allowedValues.size() == 0)
                isAllowed = true;
            else
            {
                isAllowed = false;
                for(std::string str : allowedValues)
                {
                    if(str == terminalStringValue)
                    {
                        isAllowed = true;
                        break;
                    }
                }
            }

            if(isAllowed == false)
            {
                std::string allowedValuesStr = "[";
                int i = 0;
                for(std::string str : allowedValues)
                {
                    if(i != 0)
                        allowedValuesStr += ", ";
                    allowedValuesStr += str;
                    i++;
                }
                allowedValuesStr += "]";

                result.error = nestedName + " value of " + terminalStringValue + " does not match one of allowed values of " + allowedValuesStr;
                result.success = false;
                return valueContainer;
            }

            valueContainer->AddTerminalValueFromString(parameterName, terminalStringValue, structure->getTerminalType(parameterName));

            seenTerminals[parameterName] = true;
        }
        else if(structure->NonTerminalExists(parameterName) == true)
        {
            //check if there are multiple entires for the non-terminal
            if(valueContainer->HasNonTerminal(parameterName))
            {
                if(structure->getNonTerminalMultipleEntriesAllowed(parameterName) == true)
                    throw std::runtime_error("Support for multiple entires of same tag not implemented");
                else
                {
                    result.error = nestedName + " is present multiple times";
                    result.success = false;
                    return valueContainer;
                }
            }
            valueContainer->AddNonTerminal(parameterName, ParseParameters(parameter, structure->getNonTerminalStructure(parameterName), result, nestedName));

            nonTerminalsPresentMap[parameterName] = true;
            seenNonTerminals.push_back(parameterName);
        }
        else
        {
            //The paramter in XML file is not defined in the structure, set as warning and continue;
            result.warnings.push_back(nestedName + " paramater present in configuration does not associate to any expected nonterminal/terminal structure");
        }
    }


    //go through an check for any unused paramters
    for(auto it = seenTerminals.cbegin() ; it != seenTerminals.cend() ; ++it)
    {
        std::string nestedName = "";
        if(superParameter != "")
            nestedName += superParameter + ".";
        nestedName += it->first;

        //entry not present, check if required, otherwise add default value
        if(it->second == false)
        {
            if(structure->IsTagRequired(it->first) == true)
            {
                result.error = nestedName + " not present and is marked as required";
                result.success = false;
                return valueContainer;
            }
            else
            {
                std::string defaultValue = structure->getDefaultTerminalValue(it->first);
                if(defaultValue != "")
                {
                    result.warnings.push_back(nestedName + " not set, using default value of " + defaultValue);
                    valueContainer->AddTerminalValueFromString(it->first, defaultValue, structure->getTerminalType(it->first));
                }
            }
        }
    }
    for(auto it = nonTerminalsPresentMap.cbegin() ; it != nonTerminalsPresentMap.cend() ; ++it)
    {
        std::string nestedName = "";
        if(superParameter != "")
            nestedName += superParameter + ".";
        nestedName += it->first;


        //entry not present, check if required, otherwise add default value
        if(it->second == false)
        {
            //if tag that it is unseen it mutually exclusive with a seen tag, then ignore.
            bool handledByMutuallyExclusiveParam = false;
            for(auto itt = seenNonTerminals.cbegin() ; itt != seenNonTerminals.cend() ; ++itt)
            {
                if(structure->IsTagMutuallyExclusive(it->first, *itt) == true)
                {
                    handledByMutuallyExclusiveParam = true;
                    break;
                }
            }
            if(handledByMutuallyExclusiveParam == true)
            {
                continue;
            }

            if(structure->IsTagRequired(it->first) == true)
            {
                result.error = nestedName + " not present and is marked as required";
                result.success = false;
                return valueContainer;
            }
            else
            {
                throw std::runtime_error("Default insertion on nonrequired nonterminal parameters not implimented");
                //result.warnings.push_back(nestedName + " not set, using default value");
                // TODO-PAT: Implement adding default values for non-required NonTerminals
//                valueContainer->AddNonTerminal(it->first, structure->getDefaultNonTerminalValue(it->first));
            }
        }
    }


    return valueContainer;
}


//!
//! \brief Parse an given XML file
//!
//! What parser will be lookign for is dependent on what modules were added with AddModule
//! \param filename Filename to parse
//! \return True if parsing was success
//!
ConfigurationParseResult ConfigurationReader_XML::Parse(const std::string &filename)
{
    ConfigurationParseResult parseProgress;


    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if(!result)
        return ConfigurationParseResult("Not Valid XML File");


    //loop through each "Module" tag in the XML document under "ModuleConfigurations" tag
    pugi::xml_node moduleConfigurationsNode = doc.child("ModuleConfigurations");
    if(moduleConfigurationsNode.attribute("MaceInstance").empty() == false)
    {
        m_MaceInstance = moduleConfigurationsNode.attribute("MaceInstance").as_int();
        m_MaceInstanceIDSet = true;
    }

    pugi::xml_node globalParamModule = moduleConfigurationsNode.child("GlobalParams");
    m_globalParameters = ParseParameters(globalParamModule, GetGlobalParamStructure(), parseProgress);

    for (pugi::xml_node module = moduleConfigurationsNode.child("Module"); module; module = module.next_sibling("Module"))
    {
        //if module is disabled then skip parsing
        if(module.attribute("Enabled").empty() == false)
        {
            if(module.attribute("Enabled").as_bool() == false)
                continue;
        }

        //determine module class, error if unsuccessfull
        std::string moduleClassName = module.attribute("Class").as_string();
        MaceCore::ModuleClasses moduleClass;
        try
        {
            moduleClass = MaceCore::ModuleBase::StringToModuleClass(moduleClassName);
        }
        catch(const std::runtime_error &e)
        {
            parseProgress.success = false;
            parseProgress.error = moduleClassName + " is not valid module class";
            return parseProgress;
        }


        //attempt to create module, error if unsuccesfull
        std::shared_ptr<MaceCore::ModuleBase> newModule;
        std::string moduleType = module.attribute("Type").as_string();
        try
        {
            newModule = m_Factory->Create(moduleClass, moduleType);
        }
        catch (const std::runtime_error &e)
        {
            parseProgress.success = false;
            parseProgress.error = "Unable to create module: " + std::string(e.what());
            return parseProgress;
        }


        //parse parameters
        std::shared_ptr<MaceCore::ModuleParameterStructure> structure = newModule->ModuleConfigurationStructure();
        std::shared_ptr<MaceCore::ModuleParameterValue> moduleValue = ParseParameters(module, structure, parseProgress);


        //if not successfull return
        if(parseProgress.success == false)
            return parseProgress;


        //insert in map
        m_ModuleTypes.insert({newModule, moduleType});
        m_Parameters.insert({newModule, moduleValue});
    }


    return parseProgress;
}



//!
//! \brief Confirm that MACE Instance ID has been set
//! \return True if ID has been set
//!
bool ConfigurationReader_XML::HasStaticMaceInstanceID() const
{
    return m_MaceInstanceIDSet;
}


//!
//! \brief Get MACE instance ID
//! \return MACE instance ID
//!
uint32_t ConfigurationReader_XML::GetStaticMaceInstanceID() const
{
    return m_MaceInstance;
}



//!
//! \brief Get modules created after parsing
//! \return List of created modules.
//!
std::map<std::shared_ptr<MaceCore::ModuleBase>, std::string> ConfigurationReader_XML::GetCreatedModules() const
{
    std::map<std::shared_ptr<MaceCore::ModuleBase>, std::string> map;
    for(auto it = m_ModuleTypes.cbegin(); it != m_ModuleTypes.cend() ; ++it)
        map.insert({it->first, it->second});

    return map;
}


//!
//! \brief Get the configuration for a module after parse
//!
//! Must be called after Parse is called and returns with a value of true
//! \param module Pointer to module to get configuration of
//! \return Configuration for module
//!
std::shared_ptr<MaceCore::ModuleParameterValue> ConfigurationReader_XML::GetModuleConfiguration(const std::shared_ptr<MaceCore::ModuleBase> &module)
{
    return m_Parameters.at(module);
}

//!
//! \brief Get the global configuration parameters after parse
//!
//! Must be called after Parse is called and returns with a value of true
//! \param module Pointer to module to get configuration of
//! \return Global Configuration
//!
std::shared_ptr<MaceCore::ModuleParameterValue> ConfigurationReader_XML::GetGlobalConfiguration()
{
    return m_globalParameters;
}

//!
//! \brief Describes the structure of the global parameters
//! \return Global Parameter Structure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ConfigurationReader_XML::GetGlobalParamStructure()
{
    MaceCore::ModuleParameterStructure globalStructure;
    std::shared_ptr<MaceCore::ModuleParameterStructure> originSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    originSettings->AddTerminalParameters("Latitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    originSettings->AddTerminalParameters("Longitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    originSettings->AddTerminalParameters("Altitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    globalStructure.AddNonTerminal("GlobalOrigin", originSettings, false);

    std::shared_ptr<MaceCore::ModuleParameterStructure> boundarySettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    std::vector<std::string> allowableTypes={"hard", "soft"}    ;
    boundarySettings->AddTerminalParameters("Vertices",MaceCore::ModuleParameterTerminalTypes::LATLNG,true);
    boundarySettings->AddTerminalParameters("Type",MaceCore::ModuleParameterTerminalTypes::STRING,false, "hard", allowableTypes);
    boundarySettings->AddTerminalParameters("Name",MaceCore::ModuleParameterTerminalTypes::STRING,false, "Nameless");
    globalStructure.AddNonTerminal("EnvironmentBoundary", boundarySettings, false);

    globalStructure.AddTerminalParameters("maceID", MaceCore::ModuleParameterTerminalTypes::INT, false);
    return std::make_shared<MaceCore::ModuleParameterStructure>(globalStructure);
}

