#include "space_information.h"

namespace mace {
namespace state_space {

//consider std::move of the shared pointer here
SpaceInformation::SpaceInformation(const StateSpacePtr &space):
    m_stateSpace(space),
    m_stateValidCheck(nullptr), m_motionValidCheck(nullptr),
    m_stateSampler(nullptr), isSetup(false)
{

}

void SpaceInformation::updateStateSpace(const StateSpacePtr &space)
{
    m_stateSpace = space;
    if(m_stateValidCheck)
        m_stateValidCheck->updateStateSpace(m_stateSpace);
    if(m_motionValidCheck)
        m_motionValidCheck->updateStateSpace(m_stateSpace);
    if(m_stateSampler)
        m_stateSampler->updateStateSpace(m_stateSpace);
}

const StateSpacePtr& SpaceInformation::getStateSpace() const
{
    return m_stateSpace;
}

bool SpaceInformation::isStateValid(const State *state) const
{
    return m_stateValidCheck->isValid(state);
}

bool SpaceInformation::isEdgeValid(const State *lhs, const State *rhs) const
{
    if(m_stateValidCheck->isValid(rhs))
    {
        if(m_motionValidCheck->isValid(lhs,rhs))
            return true;
    }
    return false;
}

double SpaceInformation::distanceBetween(const State *lhs, const State *rhs) const
{
    return m_stateSpace->distanceBetween(lhs, rhs);
}

double SpaceInformation::getTraversalCost(const State *begin, const State *end, const bool &neighbor) const
{
    UNUSED(neighbor);

    return m_stateSpace->traversalCost(begin,end);
}

//!
//! \brief getNewState
//! \return
//!
State* SpaceInformation::getNewState() const
{
    return m_stateSpace->getNewState();
}

//!
//! \brief removeState
//! \param state
//!
void SpaceInformation::removeState(State* state) const
{
    m_stateSpace->removeState(state);
}

//!
//! \brief copyState
//! \param state
//! \return
//!
State* SpaceInformation::copyState(const State* state) const
{
    return m_stateSpace->copyState(state);
}

//!
//! \brief removeStates
//! \param states
//!
void SpaceInformation::removeStates(std::vector<State*> states) const
{
    m_stateSpace->removeStates(states);
}

void SpaceInformation::setStateSampler(const StateSamplerPtr &sampler)
{
    m_stateSampler = sampler;
}

StateSamplerPtr SpaceInformation::getStateSampler() const
{
    return m_stateSampler;
}


void SpaceInformation::setStateValidityCheck(const AbstractStateValidityCheckPtr &stateChecker)
{
    this->m_stateValidCheck = stateChecker;
}

void SpaceInformation::setMotionValidityCheck(const AbstractMotionValidityCheckPtr &motionChecker)
{
    this->m_motionValidCheck = motionChecker;
}


} //end of namespace state_space
} //end of namespace mace
