#ifndef AI_TESTPARAMETERIZATION_H
#define AI_TESTPARAMETERIZATION_H

#include <iostream>
#include <map>

#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>

#include "data_generic_item/data_generic_item_test_descriptor.h"

#include "mlagent_test_parameterization.h"

namespace DataGenericItem {

class AI_TestParameterization
{
public:
    AI_TestParameterization() = default;

    AI_TestParameterization(const AI_TestParameterization &copy);

    std::vector<MLAgent_TestParameterization> retrieveAgentTestParameterization(const std::string &ID)
    {
        std::vector<MLAgent_TestParameterization> rtn;
        if(m_Agents.count(ID) > 0)
        {
            std::multimap<std::string, MLAgent_TestParameterization>::iterator beginIT = m_Agents.lower_bound(ID);
            std::multimap<std::string, MLAgent_TestParameterization>::iterator endIT = m_Agents.upper_bound(ID);
            while(beginIT != endIT)
            {
                if(beginIT->first == ID)
                    rtn.push_back(beginIT->second);
                beginIT++;
            }
        }
        return rtn;
    }

    void setFieldINI(const std::string &fieldFileName)
    {
        _fieldINIFile = fieldFileName;
    }

    std::string getFieldINI() const
    {
        return _fieldINIFile;
    }

    bool shouldPauseAfterLoad() const
    {
        return _pauseAfterLoad;
    }

    //!///////////////////////////////////////////////
    //! Operators
    //!//////////////////////////////////////////////
public:
    //!
    //! \brief operator = overloaded assignment operator for DataGenericItem_TestDescriptor.
    //! \param rhs object that is to be copied
    //!

    AI_TestParameterization& operator = (const AI_TestParameterization &rhs)
    {
        m_TestDescriptor = rhs.m_TestDescriptor;
        _fieldINIFile = rhs._fieldINIFile;
        m_TestOrigin = rhs.m_TestOrigin;
        m_Agents = rhs.m_Agents;
        _pauseAfterLoad = rhs._pauseAfterLoad;
        return *this;
    }

    //!
    //! \brief operator == overloaded comparison operator for DataGenericItem_TestDescriptor.
    //! \param rhs object that the data is compared against.
    //! \return true if the obejcts are equal.
    //!
    bool operator == (const AI_TestParameterization &rhs) const{
        if(this->m_TestDescriptor != rhs.m_TestDescriptor){
            return false;
        }
        if(this->_fieldINIFile != rhs._fieldINIFile){
            return false;
        }
        if(this->m_Agents != rhs.m_Agents){
            return false;
        }
        if(this->m_TestOrigin != rhs.m_TestOrigin){
            return false;
        }
        if(this->_pauseAfterLoad != rhs._pauseAfterLoad){
            return false;
        }
        return true;
    }

    //! \brief operator != overloaded comparison operator for DataGenericItem_TestDescriptor.
    //! \param rhs object that the data is compared against.
    //! \return false if the objects are equal.
    //!
    bool operator != (const AI_TestParameterization &rhs) const{
        return !(*this == rhs);
    }
    //!///////////////////////////////////////////////
    //! Member Variables
    //!//////////////////////////////////////////////

public:
    TestDescriptor m_TestDescriptor;
    std::string _fieldINIFile = ""; //this links to the ini file name that parameterizes the field conditions for the explicit agent
    mace::pose::GeodeticPosition_3D m_TestOrigin;
    std::multimap<std::string, MLAgent_TestParameterization> m_Agents;

public:
    bool _pauseAfterLoad = false;
};

} //end of namespace DataGenericItem

#endif // AI_TESTPARAMETERIZATION_H
