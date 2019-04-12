#include "discrete_motion_validity_check.h"

namespace mace {
namespace state_space {

DiscreteMotionValidityCheck::DiscreteMotionValidityCheck(const StateSpacePtr &space):
    AbstractMotionValidityCheck(space), m_StateCheck(nullptr)
{

}

void DiscreteMotionValidityCheck::setStateValidityCheck(const AbstractStateValidityCheckPtr &stateChecker)
{
    this->m_StateCheck = stateChecker;
}

bool DiscreteMotionValidityCheck::isValid(const State *begin, const State *end) const
{
    double distance = m_stateSpace->distanceBetween(begin,end);
    double interval = minCheckDistance / distance; //this value will be >1 if the end state is close to begin
    if(interval > 1.0)
        return true;

    state_space::State* intervalState = m_stateSpace->copyState(begin); //this would allow us to query last known valid state

    for(double i = interval; i < 1.0; i=i+interval)
    {
        //building new interval state, so delete old one
        m_stateSpace->removeState(intervalState);

        m_stateSpace->interpolateStates(begin, end, i, &intervalState);
        if(!m_StateCheck->isValid(intervalState))
        {
            m_stateSpace->removeState(intervalState);
            return false;
        }
    }
    m_stateSpace->removeState(intervalState);
    return true;
}



} //end of namespace state_space
} //end of namespace mace
