#include "mlagent_test_parameterization.h"
namespace DataGenericItem {

MLAgent_TestParameterization::MLAgent_TestParameterization()
{

}

MLAgent_TestParameterization::MLAgent_TestParameterization(const MLAgent_TestParameterization &copy)
{
    _agentID = copy._agentID;
    _agentINIFile = copy._agentINIFile;
    _tcINIFile = copy._tcINIFile;
}


void MLAgent_TestParameterization::setAssociatedAgentID(const std::string &ID)
{
    _agentID = ID;
}

void MLAgent_TestParameterization::setINILocations(const std::string &agentINI, const std::string &tcINI)
{
    _agentINIFile = agentINI;
    _tcINIFile = tcINI;
}

void MLAgent_TestParameterization::requestINILocations(std::string &agentINI, std::string &tcINI) const
{
    agentINI = _agentINIFile;
    tcINI = _tcINIFile;
}

std::string MLAgent_TestParameterization::getAgentID() const
{
    return _agentID;
}

std::string MLAgent_TestParameterization::getAgentINI() const
{
    return _agentINIFile;
}

std::string MLAgent_TestParameterization::getTestConditionINI() const
{
    return _tcINIFile;
}

} //end of namespace DataGenericItem
