#ifndef MLAGENT_TESTPARAMETERIZATION_H
#define MLAGENT_TESTPARAMETERIZATION_H

#include <string>

#include "common/adept_model_types.h"

#include "base/pose/pose_basic_state.h"
#include "data/dogfight_teams.h"

namespace DataGenericItem {

class MLAgent_TestParameterization
{
public:
    MLAgent_TestParameterization();
    MLAgent_TestParameterization(const MLAgent_TestParameterization &copy);

public:
    void setAssociatedAgentID(const std::string &ID);
    void setINILocations(const std::string &agentINI, const std::string &tcINI);
    void requestINILocations(std::string &agentINI, std::string &tcINI) const;

    std::string getAgentID() const;
    std::string getAgentINI() const;
    std::string getTestConditionINI() const;

public:
    void operator = (const MLAgent_TestParameterization &rhs)
    {
        _agentID = rhs._agentID;
        _agentINIFile = rhs._agentINIFile;
        _tcINIFile = rhs._tcINIFile;
    }

    bool operator == (const MLAgent_TestParameterization &rhs) const{
        if(this->_agentID != rhs._agentID){
            return false;
        }
        if(this->_agentINIFile != rhs._agentINIFile){
            return false;
        }

        if(this->_tcINIFile != rhs._tcINIFile){
            return false;
        }
        return true;
    }

    bool operator != (const MLAgent_TestParameterization &rhs) const{
        return !(*this == rhs);
    }

public:

    friend std::ostream &operator<<(std::ostream &out, const MLAgent_TestParameterization &obj)
    {
        UNUSED(obj);
        return out;
    }

private:
    std::string _agentID = "";
    std::string _agentINIFile = ""; //this links to the ini file name that parameterizes the test further for the explicit agent
    std::string _tcINIFile = ""; //this links to the ini file name that would parameterize the test conditional criteria
};

} //end of namespace command_item


#endif // MLAGENT_TESTPARAMETERIZATION_H
