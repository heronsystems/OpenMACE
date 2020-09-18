#ifndef MODULE_PARAMETERS_H
#define MODULE_PARAMETERS_H

#include <string>
#include <vector>
#include <unordered_map>
#include <stdexcept>
#include <memory>
#include <iostream>
#include "base/pose/geodetic_position_2D.h"

namespace MaceCore
{

enum class ModuleParameterTerminalTypes
{
    INT,
    DOUBLE,
    STRING,
    BOOLEAN,
    LATLNG
};



class ParameterConversion
{
public:


    template<typename T>
    //!
    //! \brief ConvertFromString Convert parameter from string
    //! \param string String to convert
    //! \return Returned parameter
    //!
    static T ConvertFromString(const std::string &string)
    {
        T value;
        bool success = FromString(string, value);

        if(success == false)
            throw std::runtime_error("Error converting string");

        return value;
    }


private:

    //!
    //! \brief FromString Get integer value from string
    //! \param string String to query
    //! \param value Value to return
    //! \return Success/failure
    //!
    static bool FromString(const std::string &string, int &value)
    {
        try
        {
            value = std::stoi (string);
            return true;
        }
        catch(const std::invalid_argument)
        {
            return false;
        }
    }

    //!
    //! \brief FromString Get double value from string
    //! \param string String to query
    //! \param value Value to return
    //! \return Success/failure
    //!
    static bool FromString(const std::string &string, double &value)
    {
        try
        {
            value = std::stod (string);
            return true;
        }
        catch(const std::invalid_argument)
        {
            return false;
        }
    }

    //!
    //! \brief FromString Get string value from string
    //! \param string String to query
    //! \param value Value to return
    //! \return Success/failure
    //!
    static bool FromString(const std::string &string, std::string &value)
    {
        value = string;
        return true;
    }

    //!
    //! \brief FromString Get bool value from string
    //! \param string String to query
    //! \param value Value to return
    //! \return Success/failure
    //!
    static bool FromString(const std::string &string, bool &value)
    {
        if(string == "true" || string == "True" || string == "TRUE")
        {
            value = true;
            return true;
        }
        if(string == "false" || string == "False" || string == "FALSE")
        {
            value = false;
            return true;
        }
        return false;
    }

    //!
    //! \brief FromString Get latlon vector from string
    //! \param string String to query
    //! \param value Value to return
    //! \return Success/failure
    //!
    static bool FromString(const std::string &string, std::vector<mace::pose::GeodeticPosition_2D> &value)
    {
        std::string errormsg = "Parse error: Enter a series of LatLng vertices formatted as (lat,lon) with any combination of whitespace";
        try
        {
            std::string opener = "(";
            std::string delimeter = ",";
            std::string closer = ")";
            double lat;
            double lng;

            size_t start = string.find(opener, 0) ;
            size_t finish;
            if (start == std::string::npos){
                throw std::runtime_error("No vertices supplied");
            }

            while (start != std::string::npos){
                finish = string.find(delimeter,start);
                if (finish == std::string::npos)
                    throw std::runtime_error(errormsg );
                lat = std::stod(string.substr(start + 1, finish - start - 1));
                start = finish + delimeter.length();

                finish = string.find(closer,start);
                if (finish == std::string::npos)
                    throw std::runtime_error(errormsg);
                lng = std::stod(string.substr(start, finish - start));
                start = string.find(opener,finish);

                value.push_back(mace::pose::GeodeticPosition_2D(lat,lng));
            }
            return true;
        }
        catch(const std::invalid_argument)
        {
            std::cout << errormsg << std::endl;
            return false;
        }
        catch(const std::runtime_error){
            return false;
        }
    }

};





//!
//! \brief Class that holds parameter values for a module
//!
class ModuleParameterValue
{
private:


    template<typename T>
    class SingleParameterValue
    {
    public:
        //!
        //! \brief SingleParameterValue Single parameter value class
        //! \param value Value of parameter
        //!
        SingleParameterValue(const T &value) :
            m_Type(typeid(T))
        {
            m_Value = value;
        }

        //!
        //! \brief GetValue Get value of parameter
        //! \return Value (templated) of parameter
        //!
        T GetValue() const
        {
            return m_Value;
        }

        //!
        //! \brief GetType Get type of parameter
        //! \return Type
        //!
        const std::type_info& GetType() const
        {
            return m_Type;
        }

    private:

        const std::type_info& m_Type;
        T m_Value;
    };

public:

    //!
    //! \brief AddTerminalValueFromString
    //! \param name Terminal name
    //! \param valueStr Terminal string
    //! \param type Terminal type
    //!
    void AddTerminalValueFromString(const std::string &name, const std::string &valueStr, const ModuleParameterTerminalTypes &type)
    {
        switch(type)
        {
        case ModuleParameterTerminalTypes::INT:
        {
            AddTerminalValue(name, ParameterConversion::ConvertFromString<int>(valueStr));
            break;
        }
        case ModuleParameterTerminalTypes::DOUBLE:
        {
            AddTerminalValue(name, ParameterConversion::ConvertFromString<double>(valueStr));
            break;
        }
        case ModuleParameterTerminalTypes::STRING:
        {
            AddTerminalValue(name, ParameterConversion::ConvertFromString<std::string>(valueStr));
            break;
        }
        case ModuleParameterTerminalTypes::BOOLEAN:
        {
            AddTerminalValue(name, ParameterConversion::ConvertFromString<bool>(valueStr));
            break;
        }
        case ModuleParameterTerminalTypes::LATLNG:
        {
            AddTerminalValue(name, ParameterConversion::ConvertFromString<std::vector<mace::pose::GeodeticPosition_2D>>(valueStr));
            break;
        }
        default:
        {
            throw std::runtime_error("Unknown type");
        }
        }
    }

    //!
    //! \brief AddTerminalValue Add int terminal value
    //! \param name Terminal name
    //! \param value Terminal value
    //!
    void AddTerminalValue(const std::string &name, const int &value)
    {
        m_TerminalValues.insert({name, std::make_shared<SingleParameterValue<int> >(SingleParameterValue<int>(value))});
    }

    //!
    //! \brief AddTerminalValue Add double terminal value
    //! \param name Terminal name
    //! \param value Terminal value
    //!
    void AddTerminalValue(const std::string &name, const double &value)
    {
        m_TerminalValues.insert({name, std::make_shared<SingleParameterValue<double> >(SingleParameterValue<double>(value))});
    }

    //!
    //! \brief AddTerminalValue Add string terminal value
    //! \param name Terminal name
    //! \param value Terminal value
    //!
    void AddTerminalValue(const std::string &name, const std::string &value)
    {
        m_TerminalValues.insert({name, std::make_shared<SingleParameterValue<std::string> >(SingleParameterValue<std::string>(value))});;
    }

    //!
    //! \brief AddTerminalValue Add bool terminal value
    //! \param name Terminal name
    //! \param value Terminal value
    //!
    void AddTerminalValue(const std::string &name, const bool &value)
    {
        m_TerminalValues.insert({name, std::make_shared<SingleParameterValue<bool> >(SingleParameterValue<bool>(value))});;
    }

    //!
    //! \brief AddTerminalValue Add latlng terminal value
    //! \param name Terminal name
    //! \param value Terminal value
    //!
    void AddTerminalValue(const std::string &name, const std::vector<mace::pose::GeodeticPosition_2D> &value)
    {
        m_TerminalValues.insert({name, std::make_shared<SingleParameterValue<std::vector<mace::pose::GeodeticPosition_2D>> >(SingleParameterValue<std::vector<mace::pose::GeodeticPosition_2D>>(value))});;
    }
    //!
    //! \brief AddNonTerminal Add module parameter by value
    //! \param name Terminal name
    //! \param value Terminal value
    //!
    void AddNonTerminal(const std::string &name, const std::shared_ptr<ModuleParameterValue> &value)
    {
        m_NonTerminalValues.insert({name, value});
    }

    //!
    //! \brief Determine if given terminal parameter exists
    //! \param name Parameter name
    //! \return True if exists
    //!
    bool HasTerminal(const std::string &name) const
    {
        //check that given parameter exists
        if(m_TerminalValues.find(name) == m_TerminalValues.cend())
            return false;
        return true;
    }


    //!
    //! \brief Determine if given non-terminal parameter exists
    //! \param name Parameter name
    //! \return True if exists
    //!
    bool HasNonTerminal(const std::string &name) const
    {
        if(m_NonTerminalValues.find(name) == m_NonTerminalValues.cend())
            return false;
        return true;
    }


    //!
    //! \brief Get value of a terminal parameter
    //! \param name Name of parameter
    //! \return Value
    //! \throws std::runtime_error Thrown if given terminal does not exists
    //!
    template<typename T>
    T GetTerminalValue(const std::string &name) const
    {
        //check that given parameter exists
        if(m_TerminalValues.find(name) == m_TerminalValues.cend())
            throw std::runtime_error("Not a terminal parameter value");

        const std::shared_ptr<void> basePtr = m_TerminalValues.at(name);

        const SingleParameterValue<T>* ptr = (const SingleParameterValue<T>*)basePtr.get();

        //check that types are correct
        if(ptr->GetType() != typeid(T))
            throw std::runtime_error("Type Missmatch");

        return ptr->GetValue();
    }


    template<typename T>
    //!
    //! \brief GetNonTerminalValue Get value of non temrinal parameter
    //! \param name Name of parameter
    //! \return Value
    //!
    T GetNonTerminalValue(const std::string &name) const
    {
        //check that given parameter exists
        if(m_NonTerminalValues.find(name) == m_NonTerminalValues.cend())
            throw std::runtime_error("Not a terminal parameter value");

        const std::shared_ptr<void> basePtr = m_NonTerminalValues.at(name);

        const SingleParameterValue<T>* ptr = (const SingleParameterValue<T>*)basePtr.get();

        //check that types are correct
        if(ptr->GetType() != typeid(T))
            throw std::runtime_error("Type Missmatch");

        return ptr->GetValue();
    }

    //!
    //! \brief Get the value of a non-terminal parameter
    //! \param name Name of non-terminal parameter
    //! \return Value
    //! \throws std::runtime_error Thrown if given non-terminal does not exists
    //!
    std::shared_ptr<ModuleParameterValue> GetNonTerminalValue(const std::string &name) const
    {
        if(m_NonTerminalValues.find(name) == m_NonTerminalValues.cend())
            throw std::runtime_error("Not a non-terminal parameter value");

        return m_NonTerminalValues.at(name);
    }


private:

    std::unordered_map<std::string, std::shared_ptr<void> > m_TerminalValues;
    std::unordered_map<std::string, std::shared_ptr<ModuleParameterValue> > m_NonTerminalValues;
};


//!
//! \brief Class that describes the structure of a parameters allowed by a module
//!
//! This class will be used to parse incomming settings and ensure the settings are of the form the module is expecting
//!
class ModuleParameterStructure
{
public:



    //!
    //! \brief Add a terminal to known parameters
    //!
    //! A non terminal paramter is an actual piece of data to store as a setting to a module
    //! \param name Name of parameter
    //! \param type Data type expecting
    //! \param required True if value is required
    //! \param defaultValue Value to set if parameter is not required and not present.
    //! \param allowedEntires Descrete set of value that are stricktly allowed
    //!
    void AddTerminalParameters(const std::string &name, const ModuleParameterTerminalTypes &type, bool required = false, const std::string &defaultValue = "", const std::vector<std::string> &allowedEntires = {})
    {
        m_TerminalParams.insert({name, type});
        std::vector<std::string> name_Vec {name};
        m_IsTagRequired.push_back(std::make_tuple(name_Vec, required));
        m_TerminalDefaultValue.insert({name, defaultValue});
        m_TerminalAllowedEntries.insert({name, allowedEntires});
    }


    //!
    //! \brief Add a non terminal to known parameters
    //!
    //! A nonterminal is a "parameter set", it is a catatory which is expected to contain other terminals with actual data.
    //! \param name Name of non terminal parameter set
    //! \param type Expected format of non-terminal
    //! \param required True if required
    //! \param defaultValue Default value to set if not required and not present in settings
    //! \param multipleEntiresAllowed True if multiple entires of this non-terminal are allowed (not implemented)
    //!
    void AddNonTerminal(const std::string &name, const std::shared_ptr<ModuleParameterStructure> &type, bool required = false, const std::shared_ptr<ModuleParameterValue> &defaultValue = std::make_shared<ModuleParameterValue>(), bool multipleEntiresAllowed = false)
    {
        m_NonTerminalParams.insert({name, type});
        std::vector<std::string> name_Vec {name};
        m_IsTagRequired.push_back(std::make_tuple(name_Vec, required));
        m_NonTerminalDefaultValue.insert({name, defaultValue});
        m_NonTerminalMultipleAllowed.insert({name, multipleEntiresAllowed});
    }


    //!
    //! Mutually exclusve set of nonterminal option sets. Such as "UDPParameters" and "SerialParameters"
    //! \param mutExlusiveParams Set of parameters
    //! \param required An option is required
    //!
    void AddMutuallyExclusiveNonTerminal(std::unordered_map<std::string, const std::shared_ptr<ModuleParameterStructure>> mutExlusiveParams, bool required = false) {

        std::vector<std::string> name_Vec;
        for(auto it = mutExlusiveParams.cbegin() ; it != mutExlusiveParams.cend() ; ++it) {
            m_NonTerminalParams.insert({it->first, it->second});
            name_Vec.push_back(it->first);
        }

        m_IsTagRequired.push_back(std::make_tuple(name_Vec, required));
        m_MutuallyExclusiveSets.push_back(name_Vec);
    }

    //!
    //! \brief getTerminalNames Get list of terminal names
    //! \return Terminal names
    //!
    std::vector<std::string> getTerminalNames() const
    {
        std::vector<std::string> keys;
        for(auto it = m_TerminalParams.cbegin() ; it != m_TerminalParams.cend() ; ++it)
            keys.push_back(it->first);
        return keys;
    }

    //!
    //! \brief getTerminalType Get terminal type
    //! \param parameterName Terminal name
    //! \return Terminal type
    //!
    ModuleParameterTerminalTypes getTerminalType(const std::string &parameterName) const
    {
        if(m_TerminalParams.find(parameterName) == m_TerminalParams.cend())
            throw std::runtime_error("Parameter does not exists");

        return m_TerminalParams.at(parameterName);
    }

    //!
    //! \brief getDefaultTerminalValue Get default terminal value of parameter
    //! \param parameterName Terminal name
    //! \return Default terminal value
    //!
    std::string getDefaultTerminalValue(const std::string &parameterName) const
    {
        if(m_TerminalDefaultValue.find(parameterName) == m_TerminalDefaultValue.cend())
            throw std::runtime_error("Parameter does not exists");

        return m_TerminalDefaultValue.at(parameterName);
    }


    //!
    //! \brief Given a pameter return the entires allowed for that parameter
    //! \param parameterName Name of parameter
    //! \return Descrite values allowed. Empty if unrestricted.
    //!
    std::vector<std::string> getTerminalAllowedEntires(const std::string &parameterName) const
    {
        if(m_TerminalAllowedEntries.find(parameterName) == m_TerminalAllowedEntries.cend())
            throw std::runtime_error("Parameter does not exists");

        return m_TerminalAllowedEntries.at(parameterName);
    }

    //!
    //! \brief getNonTerminalNames Return a list of non terminal names
    //! \return List of non terminal names
    //!
    std::vector<std::string> getNonTerminalNames() const
    {
        std::vector<std::string> keys;
        for(auto it = m_NonTerminalParams.cbegin() ; it != m_NonTerminalParams.cend() ; ++it)
            keys.push_back(it->first);
        return keys;
    }

    //!
    //! \brief getNonTerminalStructure Get non terminal structure
    //! \param parameterName Non terminal name
    //! \return Structure of non terminal
    //!
    const std::shared_ptr<ModuleParameterStructure> getNonTerminalStructure(const std::string &parameterName) const
    {
        if(m_NonTerminalParams.find(parameterName) == m_NonTerminalParams.cend())
            throw std::runtime_error("Parameter does not exists");

        return m_NonTerminalParams.at(parameterName);
    }

    //!
    //! \brief getDefaultNonTerminalValue Get non terminal default value
    //! \param parameterName Non terminal name
    //! \return Default parameter value
    //!
    std::shared_ptr<ModuleParameterValue> getDefaultNonTerminalValue(const std::string &parameterName) const
    {
        if(m_NonTerminalDefaultValue.find(parameterName) == m_NonTerminalDefaultValue.cend())
            throw std::runtime_error("Parameter does not exists");

        return m_NonTerminalDefaultValue.at(parameterName);
    }

    //!
    //! \brief getNonTerminalMultipleEntriesAllowed Returns if multiple entries are allowed on a parameter
    //! \param parameterName Non terminal name
    //! \return True: multiple entries allowed
    //!
    bool getNonTerminalMultipleEntriesAllowed(const std::string &parameterName) const
    {
        if(m_NonTerminalMultipleAllowed.find(parameterName) == m_NonTerminalMultipleAllowed.cend())
            throw std::runtime_error("Parameter does not exists");

        return m_NonTerminalMultipleAllowed.at(parameterName);
    }


    //!
    //! \brief returns true if the given parameter name is a terminal
    //! \param paramName Name of parameter
    //! \return true is exists
    //!
    bool TerminalExists(const std::string &paramName) const
    {
        for(auto it = m_TerminalParams.cbegin() ; it != m_TerminalParams.cend() ; ++it)
        {
            if(it->first == paramName)
                return true;
        }

        return false;
    }


    //!
    //! \brief returns true if the given parameter name is a non terminal
    //! \param paramName Name of parameter
    //! \return true is exists
    //!
    bool NonTerminalExists(const std::string &paramName) const
    {
        for(auto it = m_NonTerminalParams.cbegin() ; it != m_NonTerminalParams.cend() ; ++it)
        {
            if(it->first == paramName)
                return true;
        }


        return false;
    }

    //!
    //! \brief Determines if the two tags are mutually exclusive
    //! \param tag1 Tag 1
    //! \param tag2 Tag 2
    //! \return true if mutually exclusive
    //!
    bool IsTagMutuallyExclusive(const std::string &tag1, const std::string &tag2) const
    {
        if(tag1 == tag2)
        {
            return false;
        }
        for(auto it = m_MutuallyExclusiveSets.cbegin() ; it != m_MutuallyExclusiveSets.cend() ; ++it)
        {
            bool has1 = false;
            bool has2 = false;
            for(auto itt = it->cbegin() ; itt != it->cend() ; ++itt)
            {
                if(*itt == tag1)
                {
                    has1 = true;
                }

                if(*itt == tag2)
                {
                    has2 = true;
                }
            }

            if(has1 == true && has2 == true)
            {
                return true;
            }
        }
        return false;
    }

    //!
    //! \brief Return if provided tag is required in the structure
    //! \param paramName Param to look for required status
    //! \return True if required
    //!
    bool IsTagRequired(const std::string &paramName) const
    {
        for(auto it = m_IsTagRequired.cbegin() ; it != m_IsTagRequired.cend() ; ++it)
        {
            //search for paramName in it->first
            std::vector<std::string> paramVec = std::get<0>(*it);
            for(auto jt = paramVec.cbegin() ; jt != paramVec.cend() ; ++jt)
            {
                if(*jt == paramName)
                {
                    return std::get<1>(*it);
                }
            }
        }
        throw std::runtime_error("Given tag was not found");
    }

private:

    std::unordered_map<std::string, std::string> m_TerminalDefaultValue;
    std::unordered_map<std::string, std::shared_ptr<ModuleParameterValue>> m_NonTerminalDefaultValue;

    std::unordered_map<std::string, std::vector<std::string>> m_TerminalAllowedEntries;

    std::vector<std::tuple<std::vector<std::string>, bool> > m_IsTagRequired;

    std::unordered_map<std::string, ModuleParameterTerminalTypes> m_TerminalParams;
    std::unordered_map<std::string, std::shared_ptr<ModuleParameterStructure> > m_NonTerminalParams;

    std::unordered_map<std::string, bool> m_NonTerminalMultipleAllowed;

    std::vector<std::vector<std::string>> m_MutuallyExclusiveSets;

 };

} //End MaceCore Namespace

#endif // MODULE_PARAMETERS_H
