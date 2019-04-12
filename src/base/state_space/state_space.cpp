#include "state_space.h"
namespace mace{
namespace state_space {

StateSpace::StateSpace()
{
    m_name = "";
}

StateSpace::~StateSpace()
{

}

//!
//! \brief getNewState
//! \return
//!
State* StateSpace::getNewState() const
{
    return nullptr;
}

//!
//! \brief removeState
//! \param state
//!
void StateSpace::removeState(State* state) const
{
    delete state;
    state = nullptr;
}

//!
//! \brief copyState
//! \param state
//! \return
//!
State* StateSpace::copyState(const State* state) const
{
    return nullptr;
}

//!
//! \brief removeStates
//! \param states
//!
void StateSpace::removeStates(std::vector<State*> states) const
{
    for(unsigned int i = 0; i < states.size(); i++)
    {
        delete states.at(i);
        states.at(i) = nullptr;
    }
}

bool StateSpace::interpolateStates(const State *begin, const State *end, const double &distance, State** interState)
{
    *interState = begin->getClone();
    return false;
}

double StateSpace::traversalCost(const State *begin, const State *end)
{
    return this->distanceBetween(begin,end);
}

std::vector<State*> StateSpace::getNeighboringStates(const state_space::State* currentState) const
{
    std::vector<State*> rtn;
    return rtn;
}

} //end of namespace state
} //end of namespace mace
