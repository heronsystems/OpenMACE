#ifndef AI_AGENT_PARAMETERS_H
#define AI_AGENT_PARAMETERS_H

#include <iostream>
#include <common/common.h>
#include "common/adept_model_types.h"
#include "base/pose/pose_basic_state.h"

#include "base/ini_support/INIHelper.h"
#include "base/ini_support/INIReader.h"

#include "data/dogfight_teams.h"

namespace DataGenericItem {

class AI_AgentParamters
{
public:
    AI_AgentParamters();
    AI_AgentParamters(const AI_AgentParamters &copy);

public:
    // ** SETTERS **
    void setAgentID(const std::string &ID);

    void setAgentTeam(const Data::DogfightTeam &team);

    void setAgentType(const AdeptModelType &type);

    void setAgentFidelity(const bool &fidelity);

    void setUpdatePeriod(const unsigned int &period);

    // ** GETTERS **
    std::string getAgentID() const;

    Data::DogfightTeam getAgentTeam() const;

    AdeptModelType getAgentType() const;

    bool isAgentVirtual() const;

    unsigned int whatIsThePeriod() const;

    // ** INI Loading **
    void populateAgentParamsFromINI(const std::string &agentFilePath);


public:
    void operator = (const AI_AgentParamters &rhs)
    {
        _agentID = rhs._agentID;
        _associatedForce = rhs._associatedForce;
        _agentType = rhs._agentType;
        _virtualAgent = rhs._virtualAgent;
        m_initialConditions = rhs.m_initialConditions;
        _updatePeriod = rhs._updatePeriod;
    }

    bool operator == (const AI_AgentParamters &rhs) const{
        if(this->_agentID != rhs._agentID){
            return false;
        }
        if(this->_associatedForce != rhs._associatedForce){
            return false;
        }
        if(this->_agentType != rhs._agentType){
            return false;
        }
        if(this->_virtualAgent != rhs._virtualAgent){
            return false;
        }
        if(this->m_initialConditions != rhs.m_initialConditions){
            return false;
        }
        if(this->_updatePeriod != rhs._updatePeriod){
            return false;
        }
        return true;
    }

    bool operator != (const AI_AgentParamters &rhs) {
        return !(*this == rhs);
    }

public:

    friend std::ostream &operator<<(std::ostream &out, const AI_AgentParamters &obj)
    {
        UNUSED(obj);
        return out;
    }

public:
    mace::pose::BasicCarState3D m_initialConditions;

private:
    std::string _agentID;
    Data::DogfightTeam _associatedForce = Data::DogfightTeam::UNKNOWN;
    AdeptModelType _agentType;
    bool _virtualAgent = false;
    unsigned int _updatePeriod;
};

} //end of namespace DataGenericItem

#endif // AI_AGENT_PARAMETERS_H
